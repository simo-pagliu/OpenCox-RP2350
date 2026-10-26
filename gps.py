"""
GPS state, NMEA parsing, UBX packet building, and distance tracking.
All GPS global state lives here so other modules can import this module
and read/write state via attribute access (e.g. gps.gps_fix).
"""
import utime
import math

# ---------------------------------------------------------------------------
# GPS state globals
# ---------------------------------------------------------------------------
gps_buffer = ""
gps_fix = False
gps_quality = None
gps_sats = None         # satellites used in the position solution (GGA)
gps_sats_in_view = None  # satellites the receiver can see (GSV), non-zero well before a fix
gps_lat = None
gps_lon = None
gps_time = None
gps_date = None
gps_speed_ms = None     # Speed over ground (RMC) in m/s
gps_course = None       # Course over ground in degrees
gps_hdop = None         # Horizontal Dilution of Precision
gps_last_sentence = ""
gps_rx_ms = 0
gps_sentence_count = 0   # sentences that passed their NMEA checksum
gps_bad_sentence_count = 0  # sentences rejected by the checksum (truncation/corruption)
gps_raw_byte_count = 0  # total bytes ever read from the GPS UART, valid or not
gps_seen_data = False

# Distance tracking
total_distance_m = 0.0
total_rowing_s = 0.0    # GPS time covered by the steps added to total_distance_m
last_gps_lat = None
last_gps_lon = None
last_gps_time = None

# Recent speeds for the LCD pace average: (gps_time, speed_ms), one per fix.
SPEED_HISTORY_S = 10  # longest span speed_average_ms/_between() can average over
speed_history = []


# ---------------------------------------------------------------------------
# NMEA coordinate / float parsing
# ---------------------------------------------------------------------------

def parse_nmea_coordinate(raw_value, hemisphere):
    if not raw_value or not hemisphere:
        return None
    try:
        if hemisphere in ("N", "S"):
            degrees = int(raw_value[:2])
            minutes = float(raw_value[2:])
        elif hemisphere in ("E", "W"):
            degrees = int(raw_value[:3])
            minutes = float(raw_value[3:])
        else:
            return None
    except (ValueError, IndexError):
        return None
    decimal = degrees + (minutes / 60.0)
    if hemisphere in ("S", "W"):
        decimal = -decimal
    return decimal


def parse_nmea_float(raw_value):
    if raw_value is None:
        return None
    value = raw_value.strip()
    if not value:
        return None
    if "*" in value:
        value = value.split("*", 1)[0]
    try:
        return float(value)
    except ValueError:
        return None


def nmea_checksum_valid(sentence):
    """Validate an NMEA sentence's trailing *XX checksum.

    A sentence with no checksum is rejected rather than trusted: when the
    UART overruns, the surviving head of a sentence is indistinguishable
    from a complete but unchecksummed one, and every sentence the receiver
    is configured to emit carries a checksum.
    """
    star = sentence.rfind("*")
    if star < 1 or star + 3 > len(sentence):
        return False
    try:
        expected = int(sentence[star + 1:star + 3], 16)
    except ValueError:
        return False
    checksum = 0
    for char in sentence[1:star]:
        checksum ^= ord(char)
    return checksum == expected


def parse_gps_time(time_str):
    if not time_str:
        return None
    try:
        hours = int(time_str[:2])
        minutes = int(time_str[2:4])
        seconds = float(time_str[4:])
        if not (0 <= hours <= 23 and 0 <= minutes <= 59 and 0.0 <= seconds < 60.0):
            return None
        return hours * 3600 + minutes * 60 + seconds
    except (ValueError, IndexError):
        return None


def parse_gps_date(date_str):
    if not date_str:
        return None
    try:
        day = int(date_str[:2])
        month = int(date_str[2:4])
        year = int(date_str[4:6])
        return (year, month, day)
    except (ValueError, IndexError):
        return None


# ---------------------------------------------------------------------------
# UBX packet building
# ---------------------------------------------------------------------------

def ubx_checksum(payload):
    ck_a = 0
    ck_b = 0
    for b in payload:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([ck_a, ck_b])


def build_ubx_packet(msg_class, msg_id, payload):
    length = len(payload)
    header = bytes([0xB5, 0x62, msg_class, msg_id, length & 0xFF, (length >> 8) & 0xFF])
    body = bytes([msg_class, msg_id, length & 0xFF, (length >> 8) & 0xFF]) + payload
    return header + payload + ubx_checksum(body)


# UBX-CFG-MSG message ids for the standard NMEA talker (message class 0xF0).
NMEA_MSG_CLASS = 0xF0
NMEA_MSG_GGA = 0x00
NMEA_MSG_GLL = 0x01
NMEA_MSG_GSA = 0x02
NMEA_MSG_GSV = 0x03
NMEA_MSG_RMC = 0x04
NMEA_MSG_VTG = 0x05


def build_cfg_msg_packet(nmea_msg_id, rate):
    """Build a UBX-CFG-MSG packet setting one NMEA sentence's output rate.

    rate is sentences emitted per navigation solution; 0 disables the
    sentence entirely.
    """
    payload = bytes([NMEA_MSG_CLASS, nmea_msg_id & 0xFF, rate & 0xFF])
    return build_ubx_packet(0x06, 0x01, payload)


def build_cfg_rate_packet(update_hz):
    if update_hz < 1:
        update_hz = 1
    meas_rate_ms = int(1000 / update_hz)
    if meas_rate_ms < 100:
        meas_rate_ms = 100
    payload = bytes([
        meas_rate_ms & 0xFF,
        (meas_rate_ms >> 8) & 0xFF,
        0x01, 0x00,  # navRate = 1
        0x01, 0x00,  # timeRef = GPS
    ])
    return build_ubx_packet(0x06, 0x08, payload)


# ---------------------------------------------------------------------------
# GPS state update (NMEA sentence dispatch)
# ---------------------------------------------------------------------------

def update_gps_state(sentence):
    global gps_fix, gps_quality, gps_sats, gps_sats_in_view, gps_lat, gps_lon
    global gps_rx_ms, gps_sentence_count, gps_bad_sentence_count, gps_last_sentence
    global gps_seen_data, gps_time, gps_date, gps_speed_ms, gps_course, gps_hdop

    if not sentence.startswith("$"):
        return
    if not nmea_checksum_valid(sentence):
        # Parsing a truncated sentence is worse than dropping it: the field
        # positions still line up, so a half-received GGA would be read as
        # authoritative fix state.
        gps_bad_sentence_count += 1
        return
    gps_sentence_count += 1
    gps_last_sentence = sentence
    gps_rx_ms = utime.ticks_ms()
    if not gps_seen_data:
        gps_seen_data = True
        print("GPS data received")
    parts = sentence.split(",")
    if len(parts) < 2:
        return
    message_type = parts[0][3:]

    # Only RMC, GGA and GSV are parsed: main.py switches GSA, VTG and GLL
    # off, since they only repeat what RMC and GGA carry. Until that config
    # is applied they still arrive, and are ignored.
    if message_type == "GGA" and len(parts) >= 8:
        gps_quality = parts[6] or gps_quality
        gps_sats = parts[7] or gps_sats
        lat = parse_nmea_coordinate(parts[2], parts[3])
        lon = parse_nmea_coordinate(parts[4], parts[5])
        if lat is not None:
            gps_lat = lat
        if lon is not None:
            gps_lon = lon
        gps_fix = parts[6] not in ("", "0")
        gps_time = parse_gps_time(parts[1] if len(parts) > 1 else None)
        hdop = parse_nmea_float(parts[8] if len(parts) > 8 else None)
        if hdop is not None:
            gps_hdop = hdop

    elif message_type == "RMC" and len(parts) >= 8:
        if parts[2] == "A":
            gps_fix = True
        elif parts[2] == "V":
            gps_fix = False
        lat = parse_nmea_coordinate(parts[3], parts[4])
        lon = parse_nmea_coordinate(parts[5], parts[6])
        if lat is not None:
            gps_lat = lat
        if lon is not None:
            gps_lon = lon
        gps_time = parse_gps_time(parts[1] if len(parts) > 1 else None)
        gps_date = parse_gps_date(parts[9] if len(parts) > 9 else None)
        speed_knots = parse_nmea_float(parts[7] if len(parts) > 7 else None)
        if speed_knots is not None:
            gps_speed_ms = speed_knots * 0.514444
            if parts[2] == "A" and gps_time is not None:
                record_speed(gps_time, gps_speed_ms)
        # Course over ground.
        course = parse_nmea_float(parts[8] if len(parts) > 8 else None)
        if course is not None:
            gps_course = course

    elif message_type == "GSV" and len(parts) >= 4:
        # GSV field 3 is satellites in VIEW, a different quantity from GGA's
        # satellites used in the solution, so it has its own variable. In
        # view is the useful number while acquiring: it climbs long before
        # the fix, so it tells you the receiver is making progress.
        gps_sats_in_view = parts[3] or gps_sats_in_view


def gps_feed_bytes(chunk):
    global gps_buffer, gps_last_sentence, gps_raw_byte_count
    if not chunk:
        return
    gps_raw_byte_count += len(chunk)
    try:
        text = chunk.decode("utf-8", "ignore")
    except Exception:
        return
    for char in text:
        if char == "$" and gps_buffer:
            sentence = gps_buffer.strip()
            update_gps_state(sentence)
            gps_last_sentence = sentence
            gps_buffer = "$"
            continue
        if char in ("\r", "\n"):
            if gps_buffer:
                sentence = gps_buffer.strip()
                update_gps_state(sentence)
                gps_last_sentence = sentence
                gps_buffer = ""
            continue
        if len(gps_buffer) < 256:
            gps_buffer += char
        else:
            # Buffer overflow: malformed/no-newline data; reset to recover
            gps_buffer = ""


# ---------------------------------------------------------------------------
# Distance tracking (Haversine)
# ---------------------------------------------------------------------------

def calculate_distance_traveled(rowing, min_step_s=0.0):
    """Move the distance baseline to the current position.

    The step is only added to total_distance_m when rowing, and its duration
    to total_rowing_s, so the two always cover the same stretches. The
    baseline moves either way: if it stayed put while drifting on the current
    or manoeuvring, all of that would be added in one jump on the first step
    after rowing resumed.

    No step is taken until the fix is min_step_s newer than the baseline.
    Summing five 0.2 s steps a second would add every fix's position jitter
    into the distance.
    """
    global total_distance_m, total_rowing_s, last_gps_lat, last_gps_lon, last_gps_time
    if gps_lat is None or gps_lon is None:
        return total_distance_m
    # 0.05 s of slack: fix times are float sums, so 1.0 s can come out as
    # 0.99999. A negative gap (midnight rollover) takes the step.
    if gps_time is not None and last_gps_time is not None and 0 <= gps_time - last_gps_time < min_step_s - 0.05:
        return total_distance_m
    if rowing and last_gps_lat is not None and last_gps_lon is not None:
        lat1 = last_gps_lat * 3.1415926535 / 180.0
        lon1 = last_gps_lon * 3.1415926535 / 180.0
        lat2 = gps_lat * 3.1415926535 / 180.0
        lon2 = gps_lon * 3.1415926535 / 180.0
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = (dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * (dlon / 2) ** 2
        a = max(0.0, min(1.0, a))  # Clamp against float precision drift
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        total_distance_m += 6371000 * c
        # A negative gap (midnight rollover) adds no time.
        if gps_time is not None and last_gps_time is not None and gps_time > last_gps_time:
            total_rowing_s += gps_time - last_gps_time
    last_gps_lat = gps_lat
    last_gps_lon = gps_lon
    last_gps_time = gps_time
    return total_distance_m


def sync_distance_baseline():
    global last_gps_lat, last_gps_lon, last_gps_time
    if gps_lat is None or gps_lon is None:
        return
    last_gps_lat = gps_lat
    last_gps_lon = gps_lon
    last_gps_time = gps_time


# ---------------------------------------------------------------------------
# Speed averaging
# ---------------------------------------------------------------------------

def record_speed(fix_time, speed_ms):
    if speed_history:
        last_time = speed_history[-1][0]
        if fix_time == last_time:
            return
        if fix_time < last_time:
            # Midnight rollover or a receiver restart: the old times no
            # longer compare with the new ones.
            speed_history.clear()
    speed_history.append((fix_time, speed_ms))
    while speed_history[0][0] <= fix_time - SPEED_HISTORY_S:
        speed_history.pop(0)


def speed_average_ms(window_s):
    """Mean speed of the fixes in the last window_s seconds, or None.

    The window ends at the newest fix, not at the current time, so fixes
    from before a gap in the fix are never mixed with the ones after it.
    """
    if not speed_history:
        return None
    oldest = speed_history[-1][0] - window_s
    total = 0.0
    count = 0
    for fix_time, speed_ms in speed_history:
        if fix_time > oldest:
            total += speed_ms
            count += 1
    return total / count


def speed_average_between(start_time, end_time):
    """Mean speed of the fixes after start_time up to end_time, or None.

    Both are fix times, so consecutive spans share no fix.
    """
    total = 0.0
    count = 0
    for fix_time, speed_ms in speed_history:
        if start_time < fix_time <= end_time:
            total += speed_ms
            count += 1
    if not count:
        return None
    return total / count


# ---------------------------------------------------------------------------
# Formatting helpers (GPS-domain: coordinates, course, timestamps)
# ---------------------------------------------------------------------------

def format_coord(decimal_value, hemisphere_positive, hemisphere_negative):
    if decimal_value is None:
        return "--"
    hemisphere = hemisphere_positive if decimal_value >= 0 else hemisphere_negative
    return "%.4f%s" % (abs(decimal_value), hemisphere)


def format_course(course):
    if course is None:
        return "--"
    return "%.1f\u00b0" % course


# ---------------------------------------------------------------------------
# Debug / console status lines (not used in main loop, useful for REPL)
# ---------------------------------------------------------------------------

def gps_console_line():
    lat_text = format_coord(gps_lat, "N", "S") if gps_lat is not None else "--"
    lon_text = format_coord(gps_lon, "E", "W") if gps_lon is not None else "--"
    quality = gps_quality if gps_quality not in (None, "") else "-"
    sats = gps_sats if gps_sats not in (None, "") else "--"
    fix = "1" if gps_fix else "0"
    return "GPS fix=%s q=%s sats=%s lat=%s lon=%s" % (fix, quality, sats, lat_text, lon_text)


def gps_state_line():
    quality = gps_quality if gps_quality not in (None, "") else "-"
    sats = gps_sats if gps_sats not in (None, "") else "--"
    in_view = gps_sats_in_view if gps_sats_in_view not in (None, "") else "--"
    hdop = "%.1f" % gps_hdop if gps_hdop is not None else "--"
    course = format_course(gps_course)
    if gps_fix:
        status = "FIX"
    elif gps_sentence_count > 0 and utime.ticks_diff(utime.ticks_ms(), gps_rx_ms) < 5000:
        status = "RX"
    else:
        status = "NO"
    return "GPS %s Q%s S%s/%s H%s C%s" % (status, quality, sats, in_view, hdop, course)


def gps_pos_line():
    if gps_lat is None or gps_lon is None:
        if gps_sentence_count > 0 and utime.ticks_diff(utime.ticks_ms(), gps_rx_ms) < 5000:
            return "LAT/LON waiting"
        return "LAT/LON --"
    return "%s %s" % (format_coord(gps_lat, "N", "S"), format_coord(gps_lon, "E", "W"))
