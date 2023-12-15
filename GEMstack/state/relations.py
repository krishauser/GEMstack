from dataclasses import dataclass
from ..utils.serialization import register
from enum import Enum
from collections import defaultdict
from typing import List,Optional


class EntityRelationEnum(Enum):
    WITHIN = 0                  # obj1 is within lane / region obj2
    STOPPING_AT = 1             # obj1 is stopping / waiting at lane / curve obj2
    VISIBLE = 2                 # obj1 is visible to obj2
    AWARE = 3                   # obj1 is aware of obj2
    FOLLOWING = 4               # obj1 follows obj2
    PASSING = 5                 # obj1 is passing obj2, intending for obj2 to yield
    YIELDING = 6                # obj1 is yielding to obj2, allowing obj2 to pass
    MERGING_AHEAD = 7           # obj1 is merging / lane changing ahead of a obj2
    MERGING_BEHIND = 8          # obj1 is merging / lane changing behind of obj2


@dataclass
@register
class EntityRelation:
    type : EntityRelationEnum
    obj1 : str                  # Named object in the scene. '' indicates ego-vehicle
    obj2 : str                  # Named object in the scene. '' indicates ego-vehicle
    

class EntityRelationGraph:
    """A dynamic graph of relations which allows for more efficient queries
    than just a list of EntityRelations.
    """
    def __init__(self, scene = None):
        self.scene = scene
        self.relations_by_type = defaultdict(list)
        self.relations_by_obj1 = defaultdict(list)
        self.relations_by_obj2 = defaultdict(list)
        self.relations_by_obj1_type = defaultdict(list)
        self.relations_by_obj2_type = defaultdict(list)
        
    def add(self, relation : EntityRelation) -> None:
        """Adds a relation."""
        self.relations_by_type[relation.type.value].append(relation)
        self.relations_by_obj1[relation.obj1].append(relation)
        self.relations_by_obj2[relation.obj2].append(relation)
        if self.scene is not None:
            item1 = self.scene.get_entity(relation.obj1) if relation.obj1 != '' else 'VehicleState'
            item2 = self.scene.get_entity(relation.obj2) if relation.obj2 != '' else 'VehicleState'
            self.relations_by_obj1_type[item1.__class__.__name__].append(relation)
            self.relations_by_obj2_type[item2.__class__.__name__].append(relation)
    
    def update(self, relations : List[EntityRelation]) -> None:
        """Adds multiple relations."""
        for r in relations:
            self.add(r)
    
    def get(self,
            relation : EntityRelationEnum = None,
            obj1 : str = None,
            obj2 : str = None,
            obj1_type : str = None,
            obj2_type : str = None) -> List[EntityRelation]:
        """Query for relations.  You can provide relation type, object names,
        or object types, and this will return all relations matching the
        specified items.
        """
        if obj1 is not None:
            candidates = self.relations_by_obj1.get(obj1,[])
        elif obj2 is not None:
            candidates = self.relations_by_obj2.get(obj2,[])
        elif obj1_type is not None:
            candidates = self.relations_by_obj1_type.get(obj1_type,[])
        elif obj2_type is not None:
            candidates = self.relations_by_obj2_type.get(obj2_type,[])
        elif relation is not None:
            candidates = self.relations_by_type.get(relation,[])
        else:
            return sum(self.relations_by_type.values(),[])
        res = []
        for c in candidates:
            if (relation is None or c.relation == relation) and (obj1 is None or c.obj1 == obj1) and (obj2 is None or c.obj2 == obj2):
                res.append(c)
        return res
        