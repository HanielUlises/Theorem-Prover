#pragma once

#include <cstdint>
#include <memory>
#include <memory_resource>
#include <optional>
#include <span>
#include <string_view>
#include <unordered_map>
#include <variant>
#include <vector>
#include <ranges>
#include <expected>
#include <flat_map>
#include <generator>

// Sorts are phantom types: zero-cost sort tags that make ill-sorted term
// construction a compile-time error rather than a runtime one.
// Sort as a runtime variant is kept alongside for heterogeneous containers
// and theory dispatch.

struct SortBool {};
struct SortInt  {};
struct SortReal {};

template<std::size_t N>
struct SortBV {};

using Sort = std::variant<SortBool, SortInt, SortReal>;

// TermType is a discriminant packed into one byte. It drives fast dispatch
// in the SAT/theory layer without full virtual overhead on hot paths.

enum class TermType : std::uint8_t { Var, Constant, Complex };
enum class FormulaKind : std::uint8_t { Atom, Not, And, Or, Implies, Iff, Forall, Exists, Let };
enum class SolverResult : std::uint8_t { SAT, UNSAT, UNKNOWN };

class Term;
class Formula;
class Substitution;

using TermPtr    = std::shared_ptr<const Term>;
using FormulaPtr = std::shared_ptr<const Formula>;

// Unification either succeeds and extends the substitution, or fails with a
// diagnostic string. std::expected makes the two cases explicit at call sites
// without exceptions and without sentinel booleans.

using UnifyResult = std::expected<void, std::string>;

// Substitution is the semantic core of the unifier: an idempotent, acyclic
// map from variable names to terms. Binds enforce the occurs check.
// Composition applies *this to every bound image of other before merging,
// preserving idempotency.
//
// std::flat_map gives sorted contiguous storage: O(log n) lookup with cache-
// friendly linear layout, which matters when substitutions are traversed in
// bulk during propagation sweeps.

class Substitution {
public:
    TermPtr    lookup(std::string_view var_id) const;
    UnifyResult bind  (std::string_view var_id, TermPtr term);
    Substitution compose(const Substitution& other) const;

    bool empty() const noexcept { return bindings_.empty(); }
    void clear()                { bindings_.clear(); }

    // Span view over all bindings for bulk iteration (e.g. model extraction).
    auto entries() const noexcept { return std::span(bindings_.begin(), bindings_.end()); }

private:
    // flat_map: contiguous sorted storage, better cache behaviour than
    // node-based unordered_map for the substitution sizes typical in SMT.
    std::flat_map<std::string, TermPtr> bindings_;
};

// Term is the syntactic object of the logic. Subclasses are immutable after
// construction; all operations return new terms. This makes sharing under
// shared_ptr safe and simplifies the occurs check (no cycles possible).

class Term {
public:
    virtual ~Term() = default;

    virtual TermType type() const noexcept = 0;
    virtual Sort     sort() const noexcept = 0;

    // Unification extends subst to make *this and other equal, or returns an
    // error leaving subst unchanged.
    virtual UnifyResult  unify (const Term& other, Substitution& subst) const = 0;

    // Syntactic pattern match with no variable binding.
    virtual bool         match (const Term& other) const noexcept = 0;

    // Structural equality modulo subst.
    virtual bool         eq    (const Term& other, const Substitution& subst) const noexcept = 0;

    virtual TermPtr      dup   () const = 0;
    virtual TermPtr      apply (const Substitution& subst) const = 0;
    virtual std::string  to_string() const = 0;
};

// VarT<S> is a logic variable of sort S. The sort tag S is a phantom:
// it carries no data but makes VarT<SortInt> and VarT<SortBool> distinct
// types, so cross-sort unification is caught before runtime.

template<typename S>
class VarT final : public Term {
public:
    explicit VarT(std::string id) : id_(std::move(id)) {}

    TermType         type() const noexcept override { return TermType::Var; }
    Sort             sort() const noexcept override { return S{}; }
    std::string_view id()   const noexcept          { return id_; }

    UnifyResult unify(const Term& other, Substitution& subst) const override;
    bool        match(const Term& other)                       const noexcept override;
    bool        eq   (const Term& other, const Substitution& subst) const noexcept override;

    TermPtr     dup  ()                            const override;
    TermPtr     apply(const Substitution& subst)   const override;
    std::string to_string()                        const override { return "?" + id_; }

private:
    std::string id_;
};

// ConstantT<S, V> is a ground term: no variables, no unification other than
// equality of the stored value. V is the machine representation of the sort:
// bool for SortBool, int64_t for SortInt, double for SortReal, etc.

template<typename S, typename V>
class ConstantT final : public Term {
public:
    explicit ConstantT(V val) : val_(std::move(val)) {}

    TermType  type()  const noexcept override { return TermType::Constant; }
    Sort      sort()  const noexcept override { return S{}; }
    const V&  value() const noexcept          { return val_; }

    UnifyResult unify(const Term& other, Substitution& subst) const override;
    bool        match(const Term& other)                       const noexcept override;
    bool        eq   (const Term& other, const Substitution& subst) const noexcept override;

    TermPtr     dup  ()                          const override;
    TermPtr     apply(const Substitution& subst) const override { return dup(); }
    std::string to_string()                      const override;

private:
    V val_;
};

using BoolConst = ConstantT<SortBool, bool>;
using IntConst  = ConstantT<SortInt,  std::int64_t>;
using RealConst = ConstantT<SortReal, double>;

// Complex is function application: a functor symbol applied to an argument
// list. It covers both function terms and predicate atoms (when result sort
// is SortBool). Unification recurses structurally on the argument vector.
//
// Args are stored as a pmr::vector so that during proof search — where
// thousands of temporary complexes are built and discarded — they can be
// allocated from a monotonic arena and freed in bulk rather than one node at
// a time.

class Complex final : public Term {
public:
    using ArgVec = std::pmr::vector<TermPtr>;

    Complex(std::string functor, Sort result_sort, ArgVec args)
        : functor_(std::move(functor))
        , sort_(std::move(result_sort))
        , args_(std::move(args)) {}

    TermType         type()     const noexcept override { return TermType::Complex; }
    Sort             sort()     const noexcept override { return sort_; }
    std::string_view functor()  const noexcept          { return functor_; }
    std::size_t      arity()    const noexcept          { return args_.size(); }

    // Span over args: avoids copying and lets callers use range algorithms
    // directly over the contiguous TermPtr sequence.
    std::span<const TermPtr> args() const noexcept { return args_; }
    const TermPtr& arg(std::size_t i) const        { return args_[i]; }

    UnifyResult unify(const Term& other, Substitution& subst) const override;
    bool        match(const Term& other)                       const noexcept override;
    bool        eq   (const Term& other, const Substitution& subst) const noexcept override;

    TermPtr     dup  ()                            const override;
    TermPtr     apply(const Substitution& subst)   const override;
    std::string to_string()                        const override;

private:
    std::string functor_;
    Sort        sort_;
    ArgVec      args_;
};

// Formula is the propositional/first-order layer. The separation between
// Term and Formula mirrors SMTLIB2 and enables a clean DPLL(T) split: the
// SAT layer reasons about Clause/Literal, the theory solvers reason about
// Term equalities and inequalities. Quantifiers live here, not in Term, so
// the ground term layer stays decision-procedure-friendly.

class Formula {
public:
    virtual ~Formula() = default;

    virtual FormulaKind  kind()      const noexcept = 0;
    virtual FormulaPtr   negate()    const          = 0;
    virtual FormulaPtr   apply(const Substitution&) const = 0;
    virtual std::string  to_string() const          = 0;
    virtual FormulaPtr   dup()       const          = 0;
};

// Atom wraps a Bool-sorted Term as a Formula. It is the bridge between the
// equational theory layer and the propositional SAT layer.

class Atom final : public Formula {
public:
    explicit Atom(TermPtr t) : term_(std::move(t)) {}

    FormulaKind    kind()  const noexcept override { return FormulaKind::Atom; }
    const TermPtr& term()  const noexcept          { return term_; }

    FormulaPtr  negate()                         const override;
    FormulaPtr  apply (const Substitution& subst) const override;
    std::string to_string()                       const override;
    FormulaPtr  dup()                             const override;

private:
    TermPtr term_;
};

// Connective covers all binary/unary propositional connectives. Children are
// stored in a pmr::vector for the same arena-allocation reason as Complex::args.

class Connective final : public Formula {
public:
    using ChildVec = std::pmr::vector<FormulaPtr>;

    Connective(FormulaKind op, ChildVec children)
        : op_(op), children_(std::move(children)) {}

    FormulaKind              kind()     const noexcept override { return op_; }
    std::span<const FormulaPtr> children() const noexcept { return children_; }

    FormulaPtr  negate()                          const override;
    FormulaPtr  apply (const Substitution& subst) const override;
    std::string to_string()                        const override;
    FormulaPtr  dup()                              const override;

private:
    FormulaKind op_;
    ChildVec    children_;
};

// Quantifier binds a list of typed variables over a body formula. Vars are
// VarT<S> instances; their sort is encoded in their dynamic type, so the
// quantifier itself is sort-agnostic.

class Quantifier final : public Formula {
public:
    Quantifier(FormulaKind q, std::vector<TermPtr> vars, FormulaPtr body)
        : q_(q), vars_(std::move(vars)), body_(std::move(body)) {}

    FormulaKind                 kind() const noexcept override { return q_; }
    std::span<const TermPtr>    vars() const noexcept          { return vars_; }
    const FormulaPtr&           body() const noexcept          { return body_; }

    FormulaPtr  negate()                          const override;
    FormulaPtr  apply (const Substitution& subst) const override;
    std::string to_string()                        const override;
    FormulaPtr  dup()                              const override;

private:
    FormulaKind          q_;
    std::vector<TermPtr> vars_;
    FormulaPtr           body_;
};

// Literal is the unit of the SAT layer: a formula atom with a polarity bit.
// Stored by value so Clause is a flat vector of 16-byte records — one cache
// line holds four literals. Polarity as a bit rather than a negated Atom
// avoids an extra heap allocation per literal.

struct Literal {
    FormulaPtr atom;
    bool       polarity;
};

// ClauseStore holds clauses in a flat byte-addressed block with an index
// into starting offsets, approximating the SOA layout used in high-performance
// CDCL solvers (MiniSat, CaDiCaL). The mdspan view exposes the literal matrix
// for bulk operations like watched-literal scanning.

class ClauseStore {
public:
    using LiteralSpan = std::span<const Literal>;

    void         add_clause(std::vector<Literal> lits);
    LiteralSpan  clause(std::size_t idx) const;
    std::size_t  size()  const noexcept { return offsets_.size(); }

    // Two-dimensional span view: clauses × max_width.
    // Useful for SIMD-friendly watched-literal or resolution passes.
    // Padding literals have polarity=false and a null atom.
    auto matrix_view() const noexcept {
        return std::mdspan(literals_.data(), offsets_.size(), max_width_);
    }

private:
    std::vector<Literal>     literals_;
    std::vector<std::size_t> offsets_;
    std::vector<std::size_t> widths_;
    std::size_t              max_width_ = 0;
};

// Model is the satisfying assignment returned by the SAT/SMT layer.
// The assignment maps variable names to ground terms in each active theory.

struct Model {
    Substitution assignment;
};

// SolverAnswer uses std::expected-style fields rather than a union: result
// discriminates, model is populated iff SAT, reason iff UNSAT or UNKNOWN.

struct SolverAnswer {
    SolverResult           result;
    std::optional<Model>   model;
    std::optional<std::string> reason;
};

// Factory namespace keeps construction concise at call sites and decouples
// callers from concrete subclass names.

namespace make {

template<typename S>
inline TermPtr var(std::string id) {
    return std::make_shared<VarT<S>>(std::move(id));
}

inline TermPtr bool_const(bool v)         { return std::make_shared<BoolConst>(v); }
inline TermPtr int_const (std::int64_t v) { return std::make_shared<IntConst>(v);  }
inline TermPtr real_const(double v)       { return std::make_shared<RealConst>(v); }

inline TermPtr app(std::string f, Sort s, Complex::ArgVec args) {
    return std::make_shared<Complex>(std::move(f), std::move(s), std::move(args));
}

inline FormulaPtr atom   (TermPtr t)    { return std::make_shared<Atom>(std::move(t)); }
inline FormulaPtr neg    (FormulaPtr f) { return f->negate(); }

inline FormulaPtr land(Connective::ChildVec fs) {
    return std::make_shared<Connective>(FormulaKind::And, std::move(fs));
}
inline FormulaPtr lor(Connective::ChildVec fs) {
    return std::make_shared<Connective>(FormulaKind::Or, std::move(fs));
}
inline FormulaPtr implies(FormulaPtr a, FormulaPtr b, std::pmr::memory_resource* mr = std::pmr::get_default_resource()) {
    Connective::ChildVec fs(mr);
    fs.push_back(std::move(a));
    fs.push_back(std::move(b));
    return std::make_shared<Connective>(FormulaKind::Implies, std::move(fs));
}
inline FormulaPtr iff(FormulaPtr a, FormulaPtr b, std::pmr::memory_resource* mr = std::pmr::get_default_resource()) {
    Connective::ChildVec fs(mr);
    fs.push_back(std::move(a));
    fs.push_back(std::move(b));
    return std::make_shared<Connective>(FormulaKind::Iff, std::move(fs));
}
inline FormulaPtr forall(std::vector<TermPtr> vs, FormulaPtr body) {
    return std::make_shared<Quantifier>(FormulaKind::Forall, std::move(vs), std::move(body));
}
inline FormulaPtr exists(std::vector<TermPtr> vs, FormulaPtr body) {
    return std::make_shared<Quantifier>(FormulaKind::Exists, std::move(vs), std::move(body));
}

} // namespace make