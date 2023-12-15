from dataclasses import dataclass
from ..utils.serialization import register
from enum import Enum

class WeatherConditionEnum(Enum):
    SUN = 0
    OVERCAST = 1
    RAIN = 2
    SNOW = 3
    DUST = 4


class SurfaceConditionEnum(Enum):
    DRY = 0
    WET = 1
    MUD = 2
    SNOW = 3
    ICE = 4


@dataclass
@register
class EnvironmentState:
    weather : WeatherConditionEnum = WeatherConditionEnum.SUN
    surface : SurfaceConditionEnum = SurfaceConditionEnum.DRY
    wind_speed : float = 0
    surface_severity : float = 0
