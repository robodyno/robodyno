from .can_bus import CanBus
try:
    from .webots import Webots
except:
    Webots = None

from enum import Enum

class InterfaceType(Enum):
        """Robodyno device type and uuid pairs."""
        Unknown = 0x00
        CanBus = 0x01
        Webots = 0x02

def GET_IFACE_TYPE(iface):
    if isinstance(iface, CanBus):
        return InterfaceType.CanBus
    elif isinstance(iface, Webots):
        return InterfaceType.CanBus.Webots
    else:
        return InterfaceType.Unknown