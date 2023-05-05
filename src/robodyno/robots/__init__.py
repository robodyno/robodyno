import sys
if sys.version_info < (3, 10):
    from importlib_metadata import entry_points
else:
    from importlib.metadata import entry_points
thismodule = sys.modules[__name__]

display_eps = entry_points(group='robodyno.robots')
for ep in display_eps:
    setattr(thismodule, ep.name, ep.load())
