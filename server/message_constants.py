from enum import Enum

# note: intentionally redefining the enum here to separately deploy server
class MissionEnum(Enum):
    IDLE = 0            # not driving, no mission
    DRIVE = 1           # normal driving with routing
    DRIVE_ROUTE = 2     # normal driving with a fixed route
    TELEOP = 3          # manual teleop control
    RECOVERY_STOP = 4   # abnormal condition detected, must stop now
    ESTOP = 5           # estop pressed, must stop now

# note: this is primitive config, should be replaced with a more robust auth system
class ClientRole(str, Enum):
    WEBAPP = "webapp"
    SERVER = "server"
    GEMSTACK = "gemstack"
    UNKNOWN = "unknown"

# define message types
class MessageType(str, Enum):
    # client registration
    REGISTER = "register"
    REGISTRATION_RESPONSE = "registration_response"

    # requests from clients
    SUMMON = "summon"

    # responses from server
    SUMMON_RESPONSE = "summon_response"
    ERROR = "error"

    # events
    LAUNCH_EVENT = "launch_event"
