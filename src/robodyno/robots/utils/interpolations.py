import time
from math import fabs

def linear_interpolation(_from, _to, speed = None, duration = 0):
    """Linear Interpolation Algorism
    
    Args:
        _from: start value
        _to: end value
        speed: (/s), motion speed
        duration: (s), motion duration when speed not set
    
    Yield:
        interpolated value
    """
    start_t = time.time()
    dur = duration
    if speed:
        dur = fabs((_to - _from) / speed)
    while dur > 0:
        dt = time.time() - start_t
        if dt > dur:
            break
        yield((_to - _from) * dt / dur + _from)
    yield(_to)