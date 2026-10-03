"""The robot registry: `load(name) -> RobotProfile`.

One import site for every per-robot value in the tree. Anything that needs to
know a frame, a topic or a dimension asks for a profile by name; nothing else
should name a robot.
"""
from .base import RobotProfile
from .spot import SPOT
from .turtlebot4 import TURTLEBOT4

PROFILES = {profile.name: profile for profile in (SPOT, TURTLEBOT4)}

# THERE IS NO DEFAULT ROBOT, deliberately. Every caller says which robot it is
# talking to, or it does not run.
#
# A default is the wrong shape for this value. Getting it wrong is not an error
# anywhere: the node would come up, subscribe to topics nobody publishes and
# look up frames that do not exist, and a TF lookup for a missing frame never
# raises -- it just never resolves. The symptom is a stack that starts cleanly
# and then does nothing, which is the most expensive kind of failure to debug on
# a robot that is standing there powered on.


def load(name) -> RobotProfile:
    """A profile by name, or a ValueError naming the ones that exist.

    Both "no robot given" and "a robot I do not know" are refused, loudly, at
    the point the name arrives -- see the note above about why a fallback would
    be worse than a crash.
    """
    key = str(name or "").strip().lower()
    known = ", ".join(sorted(PROFILES))
    if not key:
        raise ValueError(
            f"no robot given. Pass robot:=<name> (launch), -p robot:=<name> "
            f"(the bridge) or ROBOT=<name> (the launcher scripts). "
            f"Known robots: {known}. There is deliberately no default: the "
            f"wrong robot's frames do not error, they just never resolve.")
    try:
        return PROFILES[key]
    except KeyError:
        raise ValueError(
            f"unknown robot {name!r}; known robots: {known}. Add one as "
            f"ceai/robots/<name>.py and register it here."
        ) from None


def names():
    """Known robot names, sorted -- for help text and error messages."""
    return sorted(PROFILES)


__all__ = ["RobotProfile", "PROFILES", "load", "names"]
