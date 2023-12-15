from dataclasses import dataclass
from ..utils.serialization import register
from .physical_object import ObjectPose
from enum import Enum
from typing import List,Optional

class AgentIntentEnum(Enum):
    IDLE = 0
    NORMAL = 1
    STOPPING = 2                # stopping at a stop sign / light / crossing
    LANE_CHANGE_LEFT = 3        # performing a lane change
    LANE_CHANGE_RIGHT = 4       # performing a lane change
    LEFT_TURN = 5               # performing a left turn
    RIGHT_TURN = 6              # performing a right turn
    U_TURN = 7                  # performing a u-turn
    CROSSING = 8                # executing a crossing 


@dataclass
@register
class AgentIntent:
    intent : AgentIntentEnum                    # the class of this intent
    path : Optional[List[ObjectPose]]           # the predicted (x,y,heading) path associated with this intent
    uncertainty_fwd : Optional[List[float]]     # uncertainty in the predicted path in the forward direction
    uncertainty_side : Optional[List[float]]    # uncertainty in the predicted path in the sideways direction
    uncertainty_heading : Optional[List[float]] # uncertainty in the predicted path's heading


@dataclass
@register
class AgentIntentMixture:
    """A collection of intents.  Each intent has an associated likelihood
    (likelihoods sum to 1).
    """
    predictions : List[AgentIntent]
    likelihoods : List[float]
