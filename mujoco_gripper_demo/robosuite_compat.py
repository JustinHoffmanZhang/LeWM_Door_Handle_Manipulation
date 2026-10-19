"""Re-export the upstream box model with local type information.

robosuite 1.5.2 does not annotate its optional joint specifications or the
XML element returned by get_obj(). The companion .pyi describes the API used
by this demo; the runtime class remains the original robosuite BoxObject.
"""

from robosuite.models.objects import BoxObject as BoxObject
