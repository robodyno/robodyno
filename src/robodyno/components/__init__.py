from .brands import *
from ..interfaces import InterfaceType, GET_IFACE_TYPE
import sys
if sys.version_info < (3, 10):
    from importlib_metadata import entry_points
else:
    from importlib.metadata import entry_points
__thismodule = sys.modules[__name__]

__components = {}

from .can_bus.can_bus_device import CanBusDevice
can_eps = entry_points(group='robodyno.components.can_bus')
for ep in can_eps:
    try:
        __components[ep.name].update({InterfaceType.CanBus: ep.load()})
    except:
        __components.update({ep.name: {InterfaceType.CanBus: ep.load()}})

try:
    from .webots.webots_device import WebotsDevice
    wb_eps = entry_points(group='robodyno.components.webots')
    for ep in wb_eps:
        try:
            __components[ep.name].update({InterfaceType.Webots: ep.load()})
        except:
            __components.update({ep.name: {InterfaceType.Webots: ep.load()}})
except:
    __Webots = None

def component_factory(cls_name):
    try:
        return lambda iface, *args, **kwargs: __components[cls_name][GET_IFACE_TYPE(iface)](iface, *args, **kwargs)
    except:
        raise TypeError('The Class is invalid with this interface.')

for component in __components:
    setattr(__thismodule, component, component_factory(component))