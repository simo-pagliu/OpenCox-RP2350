"""
LCD display formatting utilities.
Pure functions — no hardware or project-module imports.
"""


def fit_line(text, width=20):
    if len(text) > width:
        return text[:width]
    return text + (" " * (width - len(text)))


def format_uptime(ms):
    total_seconds = ms // 1000
    hours = total_seconds // 3600
    minutes = (total_seconds % 3600) // 60
    seconds = total_seconds % 60
    if hours > 0:
        return "UP %02d:%02d:%02d" % (hours, minutes, seconds)
    return "UP %02d:%02d" % (minutes, seconds)


def trim_number(value):
    text = "%.2f" % value
    if text.startswith("-0"):
        text = "-" + text[2:]
    elif text.startswith("0"):
        text = text[1:]
    return text


def format_accel(accel):
    return "A x%s y%s z%s" % (
        trim_number(accel.get("x", 0.0)),
        trim_number(accel.get("y", 0.0)),
        trim_number(accel.get("z", 0.0)),
    )


def format_speed(speed_ms):
    if speed_ms is None:
        return "S:--"
    return "S:%04.2f" % speed_ms


def format_spm(spm):
    if spm <= 0:
        return "SPM:--"
    return "SPM:%d" % int(round(spm))


def format_pace(speed_ms):
    """Format pace as mm:ss.decimals for 500 m."""
    if speed_ms is None or speed_ms <= 0:
        return "P:--"
    time_500m_s = 500.0 / speed_ms
    minutes = int(time_500m_s // 60)
    seconds = time_500m_s % 60
    return "P%02d:%06.3f" % (minutes, seconds)


def format_distance(meters):
    if meters < 0:
        return "D:--"
    if meters < 1000:
        return "D:%dM" % int(meters)
    km = int(meters // 1000)
    m = int(meters % 1000)
    return "D:%d.%03dK" % (km, m)
