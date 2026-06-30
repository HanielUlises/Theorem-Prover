#include "logic.hpp"

// Substitution::lookup is the base case of the optional-monad chain: a name
// either resolves to a term or it doesn't. Every monadic walk in this file
// bottoms out here.

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
        if (bound->type() != TermType::Var) {
            return bound;
        }
        // bound is itself a variable; recurse via its own id. The concrete
        // VarT<S> instantiation isn't known here, only that it's a Term of
        // kind Var, so the id is recovered through a dynamic cast — this is
        // the one place dynamic_cast is acceptable, since chase is off the
        // hot unification path and runs only on already-bound variables.
        if (auto chained = dynamic_cast<const VarT<SortBool>*>(bound.get()))
            return chase(chained->id());
        if (auto chained = dynamic_cast<const VarT<SortInt>*>(bound.get()))
            return chase(chained->id());
        if (auto chained = dynamic_cast<const VarT<SortReal>*>(bound.get()))
            return chase(chained->id());
        return bound; // unknown Var specialisation: stop rather than guess
    });
}

UnifyResult Substitution::bind(std::string_view var_id, TermPtr term) {
    // Occurs check belongs here rather than at the call site: every bind
    // goes through this function, so this is the single choke point that
    // can never be bypassed by a future caller forgetting to check.
    // (Full occurs-check traversal omitted here — wire in Term::contains
    // once written; this sketch shows the monadic shape, not the full body.)
    bindings_.insert_or_assign(std::string(var_id), std::move(term));
    return {}; // std::expected<void, ...> default-constructs to success
}

// Complex::unify is where the monadic shape earns its keep. Unifying two
// n-ary applications is "unify the functors, then unify each argument pair
// in turn, threading the substitution through and stopping at the first
// failure." That is precisely and_then-chaining over UnifyResult: each step
// either extends subst and hands control to the next step, or short-
// circuits with the first error encountered, and every later step is
// skipped automatically rather than needing an explicit early-return.

UnifyResult Complex::unify(const Term& other, Substitution& subst) const {
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