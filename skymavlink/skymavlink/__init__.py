"""SkyMAVLink: pure-Python pymavlink library for ArduPilot missions.

    class SkyMAVLink(_Flight, _Motion, _Commands)   # this file
        _Flight(_Core)                              # flight.py
        _Motion(_Core)                               # motion.py
        _Commands(_Core)                             # commands.py
        _Core                                        # core.py

All three mixins inherit _Core; the diamond collapses through MRO onto one
_Core.__init__. See README.md for the public API and CLAUDE.md for the rules
that keep this library consistent (frames, tick loop, transport, safety).
"""

from .commands import _Commands
from .core import _Core
from .flight import _Flight
from .motion import _Motion


class SkyMAVLink(_Flight, _Motion, _Commands):
    """High-level MAVLink interface for ArduPilot missions. See README.md."""


__all__ = ['SkyMAVLink']
