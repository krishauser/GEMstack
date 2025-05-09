"""Defines an abstract base class for predicates and utility functions for
working with predicates.

The PredicateBase class is the base class for all predicates and can be
composed, serialized, and deserialized.  You can compose predicates using
the `and_`, `all_`, `or_`, `any_`, and `not_` functions.  You can also
compare numeric predicates to one another using ==, !=, <, <=, >, and >=.

Predicates can be placed in this folder, and they need a unique name. By
default, each predicate is named after its class name.  You can override
this by implementing the name() method.

To serialize and deserialize, use the `serialize_predicate` and
`deserialize_predciate` functions.
"""

from __future__ import annotations
from types import UnionType
from ...state import AllState
from typing import Callable,List,Dict,Any


class PredicateBase:
    """A predicate is a stateless function that returns a value
    as a function of state.

    It may take a constructor with one or more arguments, but each
    of these arguments must be JSON-serializable, e.g., a str, int,
    float, list, or dict.
    
    It is important to implement name() as a unique and persistent
    name, because it will be used to identify the predicate in logic
    and planning.
    """
    @classmethod
    def name(cls) -> str:
        return cls.__name__
    
    def args(self) -> list:
        """Returns a list of arguments that were passed to the constructor.
        Default empty list."""
        return []

    def value_type(self):
        """Returns the type of the value returned by the predicate.
        Default bool."""
        return bool

    def value(self, state: AllState) -> Any:
        return NotImplementedError()
    
    def __eq__(self, rhs) -> PredicateBase:
        return EqPredicate(self,rhs)

    def __neq__(self, rhs) -> PredicateBase:
        return not_(EqPredicate(self,rhs))

    def __gt__(self, rhs) -> PredicateBase:
        return not_(GTPredicate(self,rhs))

    def __ge__(self, rhs) -> PredicateBase:
        return not_(GEPredicate(self,rhs))

    def __lt__(self, rhs) -> PredicateBase:
        return not_(LTPredicate(self,rhs))

    def __le__(self, rhs) -> PredicateBase:
        return not_(LEPredicate(self,rhs))


class UpdatingPredicateBase(PredicateBase):
    """A predicate that contains internal state ONLY to help make the
    calculation of the predicate more efficient.  The internal state
    is updated by the update() method.

    Note that the value returned by value() MUST not be different 
    depending on the internal state.  For example, you could implement
    a cache.  But you could not implement a predicate that returns
    a different value depending on the number of times it has been
    called.
    """
    def __init__(self):
        self._internal_value = None

    def update(self, state: AllState, internal_value : Any) -> Any:
        return NotImplementedError()

    def value(self, state: AllState):
        self._internal_value = self.update(state, self._internal_value)
        return self._internal_value


class LambdaPredicate(PredicateBase):
    """A predicate that calls a user-defined function.  Note that
    because we can't serialize functions, this predicate cannot
    be serialized."""
    def __init__(self, f : Callable[[AllState],Any]):
        self.f = f

    @classmethod
    def name(cls) -> str:
        return 'lambda'

    def args(self) -> list:
        return [self.f]

    def value(self, state: AllState) -> Any:
        return self.f(state)


class AndPredicate(PredicateBase):
    """A predicate that is the logical AND of a list of predicates."""
    def __init__(self, *predicates : PredicateBase):
        self.predicates = predicates

    @classmethod
    def name(cls) -> str:
        return 'and'

    def args(self) -> list:
        return self.predicates

    def value_type(self):
        return bool

    def value(self, state: AllState) -> bool:
        return all(p.value(state) for p in self.predicates)


class OrPredicate(PredicateBase):
    """A predicate that is the logical OR of a list of predicates."""
    def __init__(self, *predicates : PredicateBase):
        self.predicates = predicates

    @classmethod
    def name(cls) -> str:
        return 'or'

    def args(self) -> list:
        return self.predicates

    def value_type(self):
        return bool

    def value(self, state: AllState) -> bool:
        return any(p.value(state) for p in self.predicates)


class NotPredicate(PredicateBase):
    """A predicate that is the logical NOT of another predicate."""
    def __init__(self, predicate : PredicateBase):
        self.predicate = predicate

    @classmethod
    def name(cls) -> str:
        return 'not'

    def args(self) -> list:
        return [self.predicate]

    def value_type(self):
        return bool

    def value(self, state: AllState) -> bool:
        return not self.predicate.value(state)


class EqPredicate(PredicateBase):
    """A predicate that is the result of lhs == rhs."""
    def __init__(self, lhs : Any, rhs : Any):
        self.lhs = lhs
        self.rhs = rhs

    @classmethod
    def name(cls) -> str:
        return 'eq'

    def args(self) -> list:
        return [self.lhs,self.rhs]

    def value_type(self):
        return bool

    def value(self, state: AllState) -> bool:
        if isinstance(self.lhs,PredicateBase):
            lhs = self.lhs.value(state)
        else:
            lhs = self.lhs
        if isinstance(self.rhs,PredicateBase):
            rhs = self.rhs.value(state)
        else:
            rhs = self.rhs
        return lhs == rhs


class GTPredicate(PredicateBase):
    """A predicate that is the result of lhs > rhs."""
    def __init__(self, lhs : Any, rhs : Any):
        self.lhs = lhs
        self.rhs = rhs

    @classmethod
    def name(cls) -> str:
        return '>'

    def args(self) -> list:
        return [self.lhs,self.rhs]

    def value_type(self):
        return bool

    def value(self, state: AllState) -> bool:
        if isinstance(self.lhs,PredicateBase):
            lhs = self.lhs.value(state)
        else:
            lhs = self.lhs
        if isinstance(self.rhs,PredicateBase):
            rhs = self.rhs.value(state)
        else:
            rhs = self.rhs
        return lhs > rhs


class GEPredicate(PredicateBase):
    """A predicate that is the result of lhs >= rhs."""
    def __init__(self, lhs : Any, rhs : Any):
        self.lhs = lhs
        self.rhs = rhs

    @classmethod
    def name(cls) -> str:
        return '>='

    def args(self) -> list:
        return [self.lhs,self.rhs]

    def value_type(self):
        return bool

    def value(self, state: AllState) -> bool:
        if isinstance(self.lhs,PredicateBase):
            lhs = self.lhs.value(state)
        else:
            lhs = self.lhs
        if isinstance(self.rhs,PredicateBase):
            rhs = self.rhs.value(state)
        else:
            rhs = self.rhs
        return lhs >= rhs



class LTPredicate(PredicateBase):
    """A predicate that is the result of lhs < rhs."""
    def __init__(self, lhs : Any, rhs : Any):
        self.lhs = lhs
        self.rhs = rhs

    @classmethod
    def name(cls) -> str:
        return '<'
    
    def args(self) -> list:
        return [self.lhs,self.rhs]

    def value_type(self):
        return bool

    def value(self, state: AllState) -> bool:
        if isinstance(self.lhs,PredicateBase):
            lhs = self.lhs.value(state)
        else:
            lhs = self.lhs
        if isinstance(self.rhs,PredicateBase):
            rhs = self.rhs.value(state)
        else:
            rhs = self.rhs
        return lhs < rhs


class LEPredicate(PredicateBase):
    """A predicate that is the result of lhs <= rhs."""
    def __init__(self, lhs : Any, rhs : Any):
        self.lhs = lhs
        self.rhs = rhs

    @classmethod
    def name(cls) -> str:
        return '<='
    
    def args(self) -> list:
        return [self.lhs,self.rhs]

    def value_type(self):
        return bool

    def value(self, state: AllState) -> bool:
        if isinstance(self.lhs,PredicateBase):
            lhs = self.lhs.value(state)
        else:
            lhs = self.lhs
        if isinstance(self.rhs,PredicateBase):
            rhs = self.rhs.value(state)
        else:
            rhs = self.rhs
        return lhs <= rhs


def and_(a : PredicateBase, b : PredicateBase) -> PredicateBase:
    """Returns a predicate that is the logical AND of two predicates."""
    return AndPredicate(a,b)


def all_(preds : List[PredicateBase]) -> PredicateBase:
    """Returns a predicate that is the logical AND of one or more predicates."""
    assert len(preds) > 0
    return AndPredicate(*preds)


def or_(a : PredicateBase, b : PredicateBase) -> PredicateBase:
    """Returns a predicate that is the logical OR of two predicates."""
    return OrPredicate(a,b)


def any_(preds : List[PredicateBase]) -> PredicateBase:
    """Returns a predicate that is the logical OR of one or more predicates."""
    assert len(preds) > 0
    return OrPredicate(*preds)


def not_(a : PredicateBase) -> PredicateBase:
    """Returns a predicate that is the logical NOT of a predicate."""
    return NotPredicate(a)



def all_predicate_types() -> Dict[str,Any]:
    """Returns a dict of all predicate types in this folder, indexed by name"""
    import os
    registry = dict()
    folder,_ = os.path.split(__file__)
    for f in os.listdir(folder):
        if f.endswith('.py') and f not in ['__init__.py']:
            module = __import__('GEMstack.knowledge.predicates.'+f[:-3])
            for m in vars(module):
                if isinstance(m,PredicateBase) and m is not PredicateBase and m is not UpdatingPredicateBase:
                    name = m.name()
                    registry[name] = m
    return registry


def serialize_predicate(predicate : PredicateBase) -> Dict[str,Any]:
    """Serializes a predicate to a dict"""
    args = predicate.args()
    for i,a in enumerate(args):
        if isinstance(a,PredicateBase):
            args[i] = serialize_predicate(a)
    return {'type':predicate.name(),'args':args}


def deserialize_predicate(d : Dict[str,Any], all_preds = None) -> PredicateBase:
    """Instantiates a predicate from a dict"""
    if all_preds is None:
        all_preds = all_predicate_types()
    if 'type' not in d:
        return ValueError("Predicate dict must have 'type' key")
    if d['type'] not in all_preds:
        print("Known predicates:",list(all_preds.keys()))
        return ValueError("Unknown predicate type "+d['type'])
    args = d['args']
    #detect compound predicates
    for i,a in enumerate(args):
        if isinstance(a,dict) and 'type' in a:
            args[i] = deserialize_predicate(a,all_preds)
    return all_preds[d['type']](*args)


def deserialize_predicates(l : List[Dict[str,Any]]) -> List[PredicateBase]:
    """Instantiates a list of predicates from a list of dicts.

    Slightly faster than deserializing each predicate individually.
    """
    all_preds = all_predicate_types()
    return [deserialize_predicate(d,all_preds) for d in l]


def pprint(predicate: PredicateBase) -> str:
    """Pretty prints a predicate."""
    if isinstance(predicate,AndPredicate):
        return 'and('+', '.join(pprint(p) for p in predicate.predicates)+')'
    elif isinstance(predicate,OrPredicate):
        return 'or('+', '.join(pprint(p) for p in predicate.predicates)+')'
    elif isinstance(predicate,NotPredicate):
        return 'not('+pprint(predicate.predicate)+')'
    elif isinstance(predicate,EqPredicate):
        return pprint(predicate.lhs)+' == '+pprint(predicate.rhs)
    elif isinstance(predicate,GTPredicate):
        return pprint(predicate.lhs)+' > '+pprint(predicate.rhs)
    elif isinstance(predicate,GEPredicate):
        return pprint(predicate.lhs)+' >= '+pprint(predicate.rhs)
    elif isinstance(predicate,LTPredicate):
        return pprint(predicate.lhs)+' < '+pprint(predicate.rhs)
    elif isinstance(predicate,LEPredicate):
        return pprint(predicate.lhs)+' <= '+pprint(predicate.rhs)
    else:
        return predicate.name()+'('+', '.join(pprint(a) for a in predicate.args())+')'