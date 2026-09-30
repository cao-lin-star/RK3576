"""Keep a user's session localization point separate from saved dock metadata."""
import math


def matches_manual_initial(request, observed, request_stamp_ns, stamp_ns, frame):
    """Reject queued/pre-request AMCL samples and unrelated localization updates.

    This is an acknowledgement plausibility check, not a localization-accuracy
    guarantee. The saved return target remains the user's requested pose, never
    a subsequent AMCL correction. Larger corrections need a new user selection.
    """
    if frame != 'map' or request_stamp_ns <= 0 or stamp_ns < request_stamp_ns:
        return False
    try:
        values = [float(p[k]) for p in (request, observed) for k in ('x', 'y', 'yaw')]
        if not all(math.isfinite(v) for v in values):
            return False
        angle = observed['yaw'] - request['yaw']
        return (math.hypot(observed['x'] - request['x'], observed['y'] - request['y']) <= .5
                and abs(math.atan2(math.sin(angle), math.cos(angle))) <= .5)
    except (KeyError, TypeError, ValueError):
        return False
