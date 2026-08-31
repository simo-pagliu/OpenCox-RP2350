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
    "catch_duration_ms,exit_duration_ms,shape_0,shape_1,shape_2,shape_3,shape_4\n"
)

# Number of stroke-shape points appended to each A row (see stroke_detection.PicoStrokeDetector).
SHAPE_POINT_COUNT = 5


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


def log_accel_row(uptime_ms, accel, gyro, stroke_flag, catch_duration_ms=None, exit_duration_ms=None, shape=None):
    """Build an accel CSV row.

    catch_duration_ms/exit_duration_ms and shape are normally blank; they're
    only filled in on the specific sample where PicoStrokeDetector reports
    that value as newly available (see catch_duration_available(),
    exit_duration_available(), stroke_shape_available()).
    """
    shape_fields = [""] * SHAPE_POINT_COUNT
    if shape is not None:
        for i in range(min(SHAPE_POINT_COUNT, len(shape))):
            shape_fields[i] = "%.4f" % shape[i]

    fields = [
        "A",
        "",
        "",
        "%d" % int(uptime_ms),
        "%.4f" % accel.get("x", 0.0),
        "%.4f" % accel.get("y", 0.0),
        "%.4f" % accel.get("z", 0.0),
        "%.4f" % gyro.get("x", 0.0),
        "%.4f" % gyro.get("y", 0.0),
        "%.4f" % gyro.get("z", 0.0),
        "",
        "",
        "",
        "",
        "",
        "1" if stroke_flag else "0",
        "",
        "",
        "",
        "",
        "%d" % int(catch_duration_ms) if catch_duration_ms is not None else "",
        "%d" % int(exit_duration_ms) if exit_duration_ms is not None else "",
    ] + shape_fields
    return ",".join(fields) + "\n"


def log_gps_row(gps_timestamp, speed_ms, spm, distance_m, uptime_ms):
    """Build a GPS CSV row, reading position/quality fields from the gps module."""
    date_str = ""
    if _gps.gps_date:
        date_str = "%02d/%02d/%02d" % (_gps.gps_date[2], _gps.gps_date[1], _gps.gps_date[0])
    lat_val = _gps.gps_lat if _gps.gps_lat is not None else 0.0
    lon_val = _gps.gps_lon if _gps.gps_lon is not None else 0.0
    spm_val = spm if spm is not None and spm > 0 else 0.0
    timestamp_val = gps_timestamp if gps_timestamp is not None else 0.0
    sats_val = _gps.gps_sats if _gps.gps_sats not in (None, "") else ""
    hdop_val = "%.2f" % _gps.gps_hdop if _gps.gps_hdop is not None else ""
    quality_val = _gps.gps_quality if _gps.gps_quality not in (None, "") else ""
    course_val = "%.2f" % _gps.gps_course if _gps.gps_course is not None else ""
    time_str = "%.6f" % timestamp_val
    fields = [
        "G",
        date_str,
        time_str,
        "%d" % int(uptime_ms),
        "",
        "",
        "",
        "",
        "",
        "",
        "%.4f" % (speed_ms if speed_ms is not None else 0.0),
        "%.2f" % spm_val,
        "%.8f" % lat_val,
        "%.8f" % lon_val,
        "%.3f" % (distance_m if distance_m is not None else 0.0),
        "",
        sats_val,
        hdop_val,
        quality_val,
        course_val,
        "",
        "",
    ] + [""] * SHAPE_POINT_COUNT
    return ",".join(fields) + "\n"


def log_status_row(uptime_ms):
    """Build a GPS diagnostic row, written regardless of fix state.

    Uses record_type "S", which analysis_pipeline/process_csv.py already
    recognizes and skips, so no column/header changes are needed. The
    accel/gyro/lat/lon columns are unused for this row type; the otherwise-
    idle speed_mps, spm and stroke_flag columns carry the raw UART byte
    count, NMEA sentence count, and fix flag respectively, so a field test
    is diagnosable from the CSV alone: byte count stuck at 0 means nothing
    is arriving on the UART at all (wiring/power/baud); byte count rising
    but sentence count stuck at 0 means bytes arrive but never form a valid
    NMEA sentence (wrong baud, or the module is not a talking NMEA UART).
    """
    date_str = ""
    if _gps.gps_date:
        date_str = "%02d/%02d/%02d" % (_gps.gps_date[2], _gps.gps_date[1], _gps.gps_date[0])
    timestamp_val = _gps.gps_time if _gps.gps_time is not None else 0.0
    time_str = "%.6f" % timestamp_val
    sats_val = _gps.gps_sats if _gps.gps_sats not in (None, "") else ""
    hdop_val = "%.2f" % _gps.gps_hdop if _gps.gps_hdop is not None else ""
    quality_val = _gps.gps_quality if _gps.gps_quality not in (None, "") else ""
    course_val = "%.2f" % _gps.gps_course if _gps.gps_course is not None else ""
    fields = [
        "S",
        date_str,
        time_str,
        "%d" % int(uptime_ms),
        "",
        "",
        "",
        "",
        "",
        "",
        "%d" % _gps.gps_raw_byte_count,
        "%d" % _gps.gps_sentence_count,
        "",
        "",
        "",
        "1" if _gps.gps_fix else "0",
        sats_val,
        hdop_val,
        quality_val,
        course_val,
        "",
        "",
    ] + [""] * SHAPE_POINT_COUNT
    return ",".join(fields) + "\n"


def mixed_log_header():
    return MIXED_LOG_HEADER
