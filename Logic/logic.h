#ifndef _LOGIC_H
#define _logic_H

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "ctype.h"

enum TermType{Var_T, Constant_T, Complex_T};
class Substitution;
class Var;

class Term{
    public:
        virtual ~Term();
        virtual TermType gettype() const = 0;
        virtual int unify_me (const Term &, Substitution &) const = 0;
        virtual int match (const Term &) const = 0;
        virtual Term *dup () const = 0;
};

#endif
