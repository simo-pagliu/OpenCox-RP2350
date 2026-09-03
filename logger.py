"""SD card logging helpers for the mixed Pico log format."""
import utime
import os
import gps as _gps

# ---------------------------------------------------------------------------
# SD state globals (set/cleared by main.py during init and on errors)
# ---------------------------------------------------------------------------
sd_status = "SD INIT"
events_log_file = None
events_log_path = None

MIXED_LOG_HEADER = (
    "record_type,date,time,uptime_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,"
    "speed_mps,spm,gps_lat,gps_lon,distance_m,stroke_flag,sats,hdop,fix_quality,course_deg,"
    "catch_duration_ms,exit_duration_ms,shape_0,shape_1,shape_2,shape_3,shape_4,"
    "stroke_duration_ms\n"
)

# Number of stroke-shape points on each S row (see stroke_detection.PicoStrokeDetector).
SHAPE_POINT_COUNT = 5

# Every row carries the full header width; helpers below fill the columns that
# belong to their record type and leave the rest empty. No column ever means
# two different things depending on the record type -- an earlier version of
# the S row put a UART byte count in speed_mps and the GPS fix flag in
# stroke_flag, which made every diagnostic row look like a stroke carrying a
# nonsense speed to anything selecting rows by stroke_flag.
COLUMN_COUNT = len(MIXED_LOG_HEADER.strip().split(","))


# ---------------------------------------------------------------------------
# Log index helpers
# ---------------------------------------------------------------------------

def next_log_index(sd_root, prefix):
    highest = 0
    for entry in os.listdir(sd_root):
        if not entry.startswith(prefix):
            continue
        suffix = entry[len(prefix):]
        digits = ""
        for ch in suffix:
            if ch >= "0" and ch <= "9":
                digits += ch
            else:
                break
        if not digits:
            continue
        try:
            number = int(digits)
        except ValueError:
            continue
        if number > highest:
            highest = number
    return highest + 1


# ---------------------------------------------------------------------------
# Error classification
# ---------------------------------------------------------------------------

def detect_sd_error_message(error_text):
    text = str(error_text).lower()
    if "full" in text or "space" in text or "enospc" in text or "disk" in text:
        return "SD FULL"
    return "SD ERR"


# ---------------------------------------------------------------------------
# Log writers
# ---------------------------------------------------------------------------


# Column indices into the header above, so each writer names the columns it
# fills instead of counting commas.
COL_RECORD_TYPE = 0
COL_DATE = 1
COL_TIME = 2
COL_UPTIME_MS = 3
COL_ACCEL_X = 4
COL_GYRO_X = 7
COL_SPEED_MPS = 10
COL_SPM = 11
COL_LAT = 12
COL_LON = 13
COL_DISTANCE_M = 14
COL_STROKE_FLAG = 15
COL_SATS = 16
COL_HDOP = 17
COL_FIX_QUALITY = 18
COL_COURSE_DEG = 19
COL_CATCH_DURATION_MS = 20
COL_EXIT_DURATION_MS = 21
COL_SHAPE_0 = 22
COL_STROKE_DURATION_MS = 27


def _blank_row(record_type, uptime_ms):
    row = [""] * COLUMN_COUNT
    row[COL_RECORD_TYPE] = record_type
    row[COL_UPTIME_MS] = "%d" % int(uptime_ms)
    return row


def log_accel_row(uptime_ms, accel, gyro, stroke_flag):
    """Build a raw IMU sample row.

    Carries only what the sample itself measured. stroke_flag marks the
    sample on which a catch was detected; the metrics describing the stroke
    as a whole live on their own S row, since they are not properties of any
    one 100 Hz sample.
    """
    row = _blank_row("A", uptime_ms)
    for i, axis in enumerate(("x", "y", "z")):
        row[COL_ACCEL_X + i] = "%.4f" % accel.get(axis, 0.0)
        row[COL_GYRO_X + i] = "%.4f" % gyro.get(axis, 0.0)
    row[COL_STROKE_FLAG] = "1" if stroke_flag else "0"
    return ",".join(row) + "\n"


def log_stroke_row(uptime_ms, stroke):
    """Build a stroke row from a PicoStrokeDetector.get_stroke_record() dict.

    One row per completed stroke. uptime_ms is the catch that *ended* the
    stroke, not the one that started it, so the column stays monotonic for a
    reader streaming the file in order; the stroke spans
    [uptime_ms - stroke_duration_ms, uptime_ms].
    """
    row = _blank_row("S", uptime_ms)
    row[COL_STROKE_DURATION_MS] = "%d" % int(stroke["duration_ms"])
    row[COL_SPM] = "%.2f" % stroke["spm"]

    catch_ms = stroke.get("catch_duration_ms")
    if catch_ms is not None:
        row[COL_CATCH_DURATION_MS] = "%d" % int(catch_ms)
    exit_ms = stroke.get("exit_duration_ms")
    if exit_ms is not None:
        row[COL_EXIT_DURATION_MS] = "%d" % int(exit_ms)

    shape = stroke.get("shape")
    if shape is not None:
        for i in range(min(SHAPE_POINT_COUNT, len(shape))):
            row[COL_SHAPE_0 + i] = "%.4f" % shape[i]
    return ",".join(row) + "\n"


def log_gps_row(gps_timestamp, speed_ms, spm, distance_m, uptime_ms):
    """Build a GPS CSV row, reading position/quality fields from the gps module."""
    row = _blank_row("G", uptime_ms)
    if _gps.gps_date:
        row[COL_DATE] = "%02d/%02d/%02d" % (_gps.gps_date[2], _gps.gps_date[1], _gps.gps_date[0])
    row[COL_TIME] = "%.6f" % (gps_timestamp if gps_timestamp is not None else 0.0)
    row[COL_SPEED_MPS] = "%.4f" % (speed_ms if speed_ms is not None else 0.0)
    row[COL_SPM] = "%.2f" % (spm if spm is not None and spm > 0 else 0.0)
    row[COL_LAT] = "%.8f" % (_gps.gps_lat if _gps.gps_lat is not None else 0.0)
    row[COL_LON] = "%.8f" % (_gps.gps_lon if _gps.gps_lon is not None else 0.0)
    row[COL_DISTANCE_M] = "%.3f" % (distance_m if distance_m is not None else 0.0)
    if _gps.gps_sats not in (None, ""):
        row[COL_SATS] = "%s" % _gps.gps_sats
    if _gps.gps_hdop is not None:
        row[COL_HDOP] = "%.2f" % _gps.gps_hdop
    if _gps.gps_quality not in (None, ""):
        row[COL_FIX_QUALITY] = "%s" % _gps.gps_quality
    if _gps.gps_course is not None:
        row[COL_COURSE_DEG] = "%.2f" % _gps.gps_course
    return ",".join(row) + "\n"


def mixed_log_header():
    return MIXED_LOG_HEADER
