"""SD card logging helpers for the mixed Pico log format."""
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
    "speed_mps,spm,gps_lat,gps_lon,distance_m,stroke_flag,sats,hdop,fix_quality,course_deg\n"
)

# Every row carries the full header width; helpers below fill the columns that
# belong to their record type and leave the rest empty. No column ever means
# two different things depending on the record type.
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


def _blank_row(record_type, uptime_ms):
    row = [""] * COLUMN_COUNT
    row[COL_RECORD_TYPE] = record_type
    row[COL_UPTIME_MS] = "%d" % int(uptime_ms)
    return row


def log_accel_row(uptime_ms, accel, gyro, stroke_flag):
    """Build a raw IMU sample row.

    Carries only what the sample itself measured. stroke_flag marks the
    sample on which the SPM detector completed a catch.
    """
    row = _blank_row("A", uptime_ms)
    for i, axis in enumerate(("x", "y", "z")):
        row[COL_ACCEL_X + i] = "%.4f" % accel.get(axis, 0.0)
        row[COL_GYRO_X + i] = "%.4f" % gyro.get(axis, 0.0)
    row[COL_STROKE_FLAG] = "1" if stroke_flag else "0"
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
