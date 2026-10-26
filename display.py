"""
LCD display formatting utilities.
Pure functions — no hardware or project-module imports.
"""


def fit_line(text, width=20):
    if len(text) > width:
        return text[:width]
    return text + (" " * (width - len(text)))


def format_rowing_time(seconds):
    total_seconds = int(seconds)
    hours = total_seconds // 3600
    minutes = (total_seconds % 3600) // 60
    seconds = total_seconds % 60
    return "TIME:%02d:%02d:%02d" % (hours, minutes, seconds)


def format_spm(spm):
    if spm <= 0:
        return "SPM:--"
    return "SPM:%d" % int(round(spm))


def format_pace(speed_ms):
    """Format pace per 500 m as m:ss.0 or m:ss.5, to the half second.

    One stroke moves the pace by ~5 s, so tenths are noise; whole seconds
    would lose too much.
    """
    if speed_ms is None or speed_ms <= 0:
        return "PACE:--"
    # Round once, in whole half seconds: rounding the seconds on their own
    # turns 2:59.8 into "2:60.0" instead of "3:00.0".
    halves = int(round(1000.0 / speed_ms))
    return "PACE:%d:%02d.%d/500m" % (halves // 120, (halves // 2) % 60, 5 * (halves % 2))
