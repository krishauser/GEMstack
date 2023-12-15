from dataclasses import dataclass, field
from ..utils.serialization import register
from typing import Union,Dict

@dataclass
@register
class PredicateValues:
	values : Dict[str,Union[bool,int]] = field(default_factory=dict)   #the current values of nonzero predicates
	durations : Dict[str,float] = field(default_factory=dict)          #how long the predicate has been at the current value
	
	def get_value(self, name : str) -> Union[bool,int]:
		return self.values.get(name,False)

	def get_duration(self, name : str) -> float:
		return self.durations.get(name,0)
 