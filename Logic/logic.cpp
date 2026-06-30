#include "logic.hpp"
#include <algorithm>
#include <format>

// VarT<S>::unify is the base case the Complex::unify recursion bottoms out
// at: unifying a variable either trivially succeeds (same variable on both
// sides), or extends the substitution by binding this variable to whatever
// the other side is. The occurs check is not duplicated here -- it lives
// once, in Substitution::bind, which every binding path funnels through.

template<typename S>
UnifyResult VarT<S>::unify(const Term& other, Substitution& subst) const {
    // If this variable is already bound, unification must go through its
    // current binding rather than rebinding blindly -- rebinding here would
    // silently discard whatever constraint put the first binding in place.
    if (auto resolved = subst.chase(id_)) {
        return (*resolved)->unify(other, subst);
    }
    if (auto other_id = other.var_id_if_var(); other_id && *other_id == id_) {
        return {}; // same unbound variable on both sides: trivially unified
    }
    return subst.bind(id_, other.dup());
}

template<typename S>
bool VarT<S>::match(const Term& other) const noexcept {
    auto other_id = other.var_id_if_var();
    return other_id.has_value() && *other_id == id_;
}

template<typename S>
bool VarT<S>::eq(const Term& other, const Substitution& subst) const noexcept {
    auto resolved = subst.chase(id_);
    if (!resolved) {
        // Unbound: equal only to the same unbound variable.
        auto other_id = other.var_id_if_var();
        return other_id.has_value() && *other_id == id_;
    }
    return (*resolved)->eq(other, subst);
}

template<typename S>
TermPtr VarT<S>::dup() const {
    return std::make_shared<VarT<S>>(id_);
}

template<typename S>
TermPtr VarT<S>::apply(const Substitution& subst) const {
    if (auto resolved = subst.chase(id_)) {
        return (*resolved)->apply(subst); // resolve transitively bound vars
    }
    return dup(); // unbound: substitution leaves this variable untouched
}

// ConstantT<S,V>::unify never touches subst: two ground terms either are
// the same value (success, no binding needed) or they are not (failure).
// This is the other base case Complex::unify's recursion bottoms out at,
// alongside VarT::unify above.

template<typename S, typename V>
UnifyResult ConstantT<S, V>::unify(const Term& other, Substitution& subst) const {
    // Unification must be symmetric: if other is an unbound variable, this
    // constant should bind it, not fail just because the variable happened
    // to land on the right rather than the left of the call. Delegating to
    // other.unify(*this, subst) when other is a variable reuses VarT::unify
    // rather than duplicating its binding logic here.
    if (other.var_id_if_var()) {
        return other.unify(*this, subst);
    }
    const auto* rhs = dynamic_cast<const ConstantT<S, V>*>(&other);
    if (rhs == nullptr) {
        return std::unexpected("sort/type mismatch unifying constant " + to_string() +
                                " with " + other.to_string());
    }
    if (val_ == rhs->val_) return {};
    return std::unexpected("constants differ: " + to_string() + " vs " + rhs->to_string());
}

template<typename S, typename V>
bool ConstantT<S, V>::match(const Term& other) const noexcept {
    const auto* rhs = dynamic_cast<const ConstantT<S, V>*>(&other);
    return rhs != nullptr && val_ == rhs->val_;
}

template<typename S, typename V>
bool ConstantT<S, V>::eq(const Term& other, const Substitution&) const noexcept {
    const auto* rhs = dynamic_cast<const ConstantT<S, V>*>(&other);
    return rhs != nullptr && val_ == rhs->val_;
}

template<typename S, typename V>
TermPtr ConstantT<S, V>::dup() const {
    return std::make_shared<ConstantT<S, V>>(val_);
}

template<typename S, typename V>
std::string ConstantT<S, V>::to_string() const {
    if constexpr (std::is_same_v<V, bool>) {
        return val_ ? "true" : "false";
    } else {
        return std::format("{}", val_);
    }
}

// Explicit instantiations for the sort/value combinations declared in the
// header. Add a line here whenever a new ConstantT<S,V> alias is introduced.
template class ConstantT<SortBool, bool>;
template class ConstantT<SortInt,  std::int64_t>;
template class ConstantT<SortReal, double>;
template class VarT<SortBool>;
template class VarT<SortInt>;
template class VarT<SortReal>;

// Minimal structural implementations to make Complex a complete type for
// linking and testing the unifier above. to_string and dup are needed by
// unify's own diagnostics and occurs-check dup; match/eq/apply are stubbed
// with their correct recursive shape, pending Formula-layer integration.

std::string Complex::to_string() const {
    std::string out = functor_ + "(";
    for (std::size_t i = 0; i < args_.size(); ++i) {
        if (i) out += ", ";
        out += args_[i]->to_string();
    }
    out += ")";
    return out;
}

TermPtr Complex::dup() const {
    Complex::ArgVec copied;
    copied.reserve(args_.size());
    for (const auto& a : args_) copied.push_back(a->dup());
    return std::make_shared<Complex>(functor_, sort_, std::move(copied));
}

bool Complex::match(const Term& other) const noexcept {
    const auto* rhs = dynamic_cast<const Complex*>(&other);
    if (rhs == nullptr || functor_ != rhs->functor_ || args_.size() != rhs->args_.size())
        return false;
    return std::ranges::equal(args_, rhs->args_, [](const TermPtr& a, const TermPtr& b) {
        return a->match(*b);
    });
}

bool Complex::eq(const Term& other, const Substitution& subst) const noexcept {
    const auto* rhs = dynamic_cast<const Complex*>(&other);
    if (rhs == nullptr || functor_ != rhs->functor_ || args_.size() != rhs->args_.size())
        return false;
    return std::ranges::equal(args_, rhs->args_, [&subst](const TermPtr& a, const TermPtr& b) {
        return a->eq(*b, subst);
    });
}

TermPtr Complex::apply(const Substitution& subst) const {
    Complex::ArgVec substituted;
    substituted.reserve(args_.size());
    for (const auto& a : args_) substituted.push_back(a->apply(subst));
    return std::make_shared<Complex>(functor_, sort_, std::move(substituted));
}


// reads as the definition itself ("var occurs in this application iff it
// occurs in some argument")

bool Complex::contains_var(std::string_view var_id) const noexcept {
    return std::ranges::any_of(args_, [var_id](const TermPtr& arg) {
        return arg->contains_var(var_id);
    });
}



std::optional<TermPtr> Substitution::lookup(std::string_view var_id) const {
    auto it = bindings_.find(std::string(var_id));
    if (it == bindings_.end()) return std::nullopt;
    return it->second;
}

// chase follows var -> var -> var chains to their end. Written with
// and_then it reads as the algebra it implements: "look up var_id; if the
// result is itself a variable, look that up too; stop at the first
// non-variable or the first unbound name." No manual loop, no sentinel
// values for "not a variable" — the type system carries that information.

std::optional<TermPtr> Substitution::chase(std::string_view var_id) const {
    return lookup(var_id).and_then([this](const TermPtr& bound) -> std::optional<TermPtr> {
        // var_id_if_var() answers "is this term a variable, and if so what's
        // its id" without needing to know the concrete VarT<S> instantiation.
        // and_then again here chains straight into recursion when bound is
        // itself a variable, and falls through to returning bound unchanged
        // (via the std::nullopt branch's complement) when it's ground.
        auto next_id = bound->var_id_if_var();
        if (!next_id) return bound;
        return chase(*next_id);
    });
}

UnifyResult Substitution::bind(std::string_view var_id, TermPtr term) {
    // Occurs check lives here, the single choke point every bind passes
    // through, so no future call site can accidentally skip it. Without
    // this, X = f(X) would silently succeed and produce an infinite term
    // the first time apply() tried to substitute X back in.
    if (term->contains_var(var_id)) {
        return std::unexpected(
            "occurs check failed: variable " + std::string(var_id) +
            " occurs in " + term->to_string());
    }
    bindings_.insert_or_assign(std::string(var_id), std::move(term));
    return {};
}

// Complex::unify is where the monadic shape earns its keep. Unifying two
// n-ary applications is "unify the functors, then unify each argument pair
// in turn, threading the substitution through and stopping at the first
// failure." That is precisely and_then-chaining over UnifyResult: each step
// either extends subst and hands control to the next step, or short-
// circuits with the first error encountered, and every later step is
// skipped automatically rather than needing an explicit early-return.

UnifyResult Complex::unify(const Term& other, Substitution& subst) const {
    // Same symmetry concern as ConstantT::unify: if other is an unbound
    // variable, it should bind to this Complex rather than fail just
    // because the variable is on the right-hand side of this call.
    if (other.var_id_if_var()) {
        return other.unify(*this, subst);
    }

    const auto* rhs = dynamic_cast<const Complex*>(&other);
    if (rhs == nullptr) {
        return std::unexpected(
            "cannot unify Complex with non-Complex term: " + other.to_string());
    }
    if (functor_ != rhs->functor_ || args_.size() != rhs->args_.size()) {
        return std::unexpected(
            "functor/arity mismatch: " + to_string() + " vs " + rhs->to_string());
    }

    // Fold and_then over the argument pairs. UnifyResult is void-valued on
    // success, so each step's only job is "did this succeed", and and_then
    // chains exactly that: run the next unification only if every prior one
    // succeeded, propagating the first failure untouched.
    UnifyResult result; // starts as success
    for (std::size_t i = 0; i < args_.size() && result.has_value(); ++i) {
        result = result.and_then([&, i]() -> UnifyResult {
            return args_[i]->unify(*rhs->args_[i], subst);
        });
    }
    return result;
}