from machine import Pin, I2C, SPI, UART
import utime
import sdcard
import os
from mpu6050 import MPU6050
from i2c_lcd import I2cLcd
from spm_detection import SpmDetector
from pace import StrokePace
import gps
import logger
import display

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
SPI_BAUD = 50000
SERIAL_BAUD = 115200

# SD card on SPI1: SCK=GP10, MOSI=GP11, MISO=GP8, CS=GP9
SD_SPI_ID = 1
SD_SCK_PIN = 10
SD_MOSI_PIN = 11
SD_MISO_PIN = 8
SD_CS_PIN = 9

# LCD: SDA=12, SCL=13 (I2C0)
LCD_I2C_ID = 0
LCD_SDA_PIN = 12
LCD_SCL_PIN = 13

# MPU6050: SDA=14, SCL=15 (I2C1)
MPU_I2C_ID = 1
MPU_SDA_PIN = 14
MPU_SCL_PIN = 15

MPU_SAMPLE_RATE_HZ = 100  # the SPM detector is tuned for 100 Hz
MPU_DLPF_CFG = 3

LOG_FLUSH_INTERVAL_MS = 200
LCD_INTERVAL_MS = 500
GPS_UPDATE_HZ = 5  # the NEO-6M's maximum
GPS_MAX_READS_PER_LOOP = 4
GPS_READ_CHUNK_BYTES = 64

MIN_SPM = 14
# Above 50, a mid-cycle peak misclassified as "big" 1.1-1.2 s after a real
# catch (~52-55 SPM) would pass the detector's min-interval check.
SPM_MAX = 50

# Accelerometer axes the SPM detector reads, as weights (x, y, z) on the
# sensor's axes: it gets wx*ax + wy*ay + wz*az. It needs the acceleration
# along the boat's direction of travel, so this depends on the mounting. The
# board sits on the footrest tilted ~40 deg about the sensor's X axis, which
# puts the fore-aft direction in the Y-Z plane: (0, cos 40, sin 40).
# A single axis is e.g. (0, 1, 0); any mix of two or all three axes works,
# and a negative weight flips an axis. The weights are normalised, so
# (0, 1, 1) means 45 deg between Y and Z. See the README, "Mounting and the
# SPM axis".
SPM_AXIS_WEIGHTS = (0.0, 0.766, 0.643)

# Below this the boat is drifting on the current or manoeuvring, not rowing.
MIN_ROWING_SPEED_MS = 1.0

# The LCD pace is the mean GPS speed over the last PACE_AVG_STROKES strokes,
# catch to catch, set at each catch and held until the next (see pace.py).
# PACE_AVG_STROKES strokes at MIN_SPM must fit in gps.SPEED_HISTORY_S
# (2 at 14 SPM: 8.6 s of 10).
PACE_AVG_STROKES = 2
# Once no catch has come for a stroke at MIN_SPM: the plain average of the
# last PACE_FALLBACK_WINDOW_S, refreshed every PACE_FALLBACK_INTERVAL_MS.
PACE_FALLBACK_WINDOW_S = 5
PACE_FALLBACK_INTERVAL_MS = 1000

ACCEL_LOG_INTERVAL_MS = 1000 // MPU_SAMPLE_RATE_HZ
DISTANCE_UPDATE_INTERVAL_MS = 1000  # one distance step per second, not per fix
STORAGE_RESERVE_BYTES = 128 * 1024

# ---------------------------------------------------------------------------
# Stroke detection
# ---------------------------------------------------------------------------
# The firmware computes SPM only, from the boat's fore-aft acceleration
# (SPM_AXIS_WEIGHTS): every stroke shows one deep dip at the catch and a
# sharp rise in the drive, so it gives an accurate stroke period. Its catches
# are also what the A rows' stroke_flag marks, and any analysis beyond SPM
# happens in OpenCox-WebUI from the raw A rows.
spm_detector = SpmDetector(min_spm=MIN_SPM, max_spm=SPM_MAX)
stroke_pace = StrokePace(PACE_AVG_STROKES, spm_detector.max_interval_ms,
                         PACE_FALLBACK_WINDOW_S, PACE_FALLBACK_INTERVAL_MS)

# The weights normalised to a unit vector, so the detector gets g.
_spm_axis_norm = sum(w * w for w in SPM_AXIS_WEIGHTS) ** 0.5
if _spm_axis_norm == 0:
    raise ValueError("SPM_AXIS_WEIGHTS needs at least one non-zero weight")
SPM_AXIS = tuple(w / _spm_axis_norm for w in SPM_AXIS_WEIGHTS)


def detect_stroke(accel_data, current_time_ms):
    """Feed one sample to the SPM detector; True when it completed a catch."""
    accel_x = accel_data.get("x", 0.0)
    accel_y = accel_data.get("y", 0.0)
    accel_z = accel_data.get("z", 0.0)
    fore_aft = SPM_AXIS[0] * accel_x + SPM_AXIS[1] * accel_y + SPM_AXIS[2] * accel_z
    catch_detected = spm_detector.update(fore_aft, current_time_ms)
    if catch_detected:
        spm_value = spm_detector.get_spm()
        if spm_value > 0:
            print("Catch: %d SPM" % spm_value)
        else:
            print("First catch detected")

    return catch_detected


def get_spm():
    return spm_detector.get_spm()


def rowing_speed_ms(speed_ms):
    """speed_ms, or None when it's unknown or too slow to be rowing."""
    if speed_ms is None or speed_ms < MIN_ROWING_SPEED_MS:
        return None
    return speed_ms


# ---------------------------------------------------------------------------
# Hardware init: I2C, LCD, MPU6050
# ---------------------------------------------------------------------------
lcd = None
try:
    lcd_i2c = I2C(LCD_I2C_ID, sda=Pin(LCD_SDA_PIN), scl=Pin(LCD_SCL_PIN), freq=400000)
    lcd_devices = lcd_i2c.scan()
    print("LCD I2C devices found:", [hex(i) for i in lcd_devices])
    if 0x27 in lcd_devices:
        lcd_address = 0x27
    elif 0x3F in lcd_devices:
        lcd_address = 0x3F
    else:
        raise OSError("no LCD found at 0x27 or 0x3F")
    lcd = I2cLcd(lcd_i2c, lcd_address, 4, 20)
    lcd.clear()
    print("LCD initialized successfully at 0x%02X." % lcd_address)
except Exception as e:
    lcd = None
    print("LCD unavailable; continuing without LCD:", e)

mpu = None
try:
    mpu_i2c = I2C(MPU_I2C_ID, sda=Pin(MPU_SDA_PIN), scl=Pin(MPU_SCL_PIN), freq=400000)
    print("MPU I2C devices found:", [hex(i) for i in mpu_i2c.scan()])
    mpu = MPU6050(mpu_i2c, sample_rate_hz=MPU_SAMPLE_RATE_HZ, dlpf_cfg=MPU_DLPF_CFG)
    print("MPU6050 initialized successfully.")
    print("MPU6050 sample rate: %.1fHz (DLPF=%d)" % (mpu.sample_rate_hz, mpu.dlpf_cfg))
    print("SPM detection ready (fore-aft acceleration).")
except Exception as e:
    mpu = None
    print("MPU6050 initialization failed:", e)

# ---------------------------------------------------------------------------
# Hardware init: SD card
# ---------------------------------------------------------------------------
sd = None
logging_started = False
logging_disabled = False
logging_root = None

print("=== SD INIT START ===")

try:
    cs = Pin(SD_CS_PIN, Pin.OUT, value=1)
    # Many SD modules have no pull-up resistors of their own, and the card
    # releases MISO whenever CS is high, so nothing may hold that
    # line. Enable the internal pull-up before the pin is handed to the SPI
    # peripheral: the pad's pull setting survives the change of function.
    Pin(SD_MISO_PIN, Pin.IN, Pin.PULL_UP)
    spi = SPI(SD_SPI_ID, baudrate=SPI_BAUD, sck=Pin(SD_SCK_PIN), mosi=Pin(SD_MOSI_PIN), miso=Pin(SD_MISO_PIN))
    sd = sdcard.SDCard(spi, cs)
    os.mount(sd, "/sd")
    print("SD card mounted.")
    print("Files on SD:", os.listdir("/sd"))
    logger.sd_status = "SD"
except Exception as e:
    sd = None
    print("SD card init failed:", e)
    logger.sd_status = "OB"


def storage_has_headroom(root):
    try:
        statvfs = getattr(os, "statvfs", None)
        if statvfs is None:
            return None
        stats = statvfs(root)
        free_bytes = stats[0] * stats[4]
        return free_bytes >= STORAGE_RESERVE_BYTES
    except OSError as e:
        print("Storage check failed on %s:" % root, e)
        return None

def open_session_log(root, prefix, source):
    try:
        headroom = storage_has_headroom(root)
        if headroom is None:
            return False, "KO"
        if not headroom:
            print("Storage nearly full on", root)
            return False, "FL"
        log_index = logger.next_log_index(root, prefix)
        filename = "%s%03d.csv" % (prefix, log_index)
        logger.events_log_path = "/%s" % filename if root == "/" else "%s/%s" % (root, filename)
        logger.events_log_file = open(logger.events_log_path, "w")
        logger.events_log_file.write(logger.mixed_log_header())
        logger.events_log_file.flush()
        print("Session mixed logging to", logger.events_log_path)
        logger.sd_status = source
        return True, ""
    except Exception as e:
        print("Session log file open failed on %s:" % source, e)
        if logger.events_log_file:
            try:
                logger.events_log_file.close()
            except Exception:
                pass
        logger.events_log_file = None
        logger.events_log_path = None
        return False, "KO"


def setup_session_log(flash_only=False):
    global logging_started, logging_disabled, logging_root
    if logging_started or logging_disabled:
        return logging_started
    targets = []
    if sd and not flash_only:
        targets.append(("/sd", "rowing_log", "SD"))
    targets.append(("/", "pico_fallback_log", "OB"))

    failure_codes = []
    for root, prefix, source in targets:
        opened, failure_code = open_session_log(root, prefix, source)
        if opened:
            logging_root = root
            logging_started = True
            return True
        failure_codes.append(failure_code)

    logger.sd_status = "FL" if "FL" in failure_codes else "KO"
    logging_disabled = True
    return False


def switch_to_flash_log():
    global logging_started, logging_disabled, logging_root
    if logger.events_log_file:
        try:
            logger.events_log_file.close()
        except OSError:
            pass
    logger.events_log_file = None
    logger.events_log_path = None
    logging_started = False
    logging_disabled = False
    logging_root = None
    return setup_session_log(flash_only=True)

if sd:
    try:
        print("Files on SD after setup:", os.listdir("/sd"))
    except Exception as e:
        print("Could not list SD files:", e)

# ---------------------------------------------------------------------------
# Hardware init: GPS UART + UBX configuration
# ---------------------------------------------------------------------------
GPS_UART_RXBUF_BYTES = 1024  # ~1.07 s of headroom at 9600 baud

gps_uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1), rxbuf=GPS_UART_RXBUF_BYTES)
print("GPS UART initialized.")
print("GPS waiting for data...")

# Sentences per navigation solution (CFG-MSG counts fixes, not seconds).
# RMC carries time, position, speed and course, so it comes with every fix.
# GGA adds fix quality, satellites and HDOP once a second, and GSV
# (satellites in view, only used by the REPL status line) every 5 s. GSA,
# VTG and GLL only repeat what RMC and GGA carry, and gps.py doesn't parse
# them. At 5 Hz this is ~480 bytes/s, half of what 9600 baud carries; the
# receiver's default set would not fit at 5 Hz.
GPS_NMEA_RATES = (
    (gps.NMEA_MSG_RMC, 1),
    (gps.NMEA_MSG_GGA, GPS_UPDATE_HZ),
    (gps.NMEA_MSG_GSV, 5 * GPS_UPDATE_HZ),
    (gps.NMEA_MSG_GSA, 0),
    (gps.NMEA_MSG_VTG, 0),
    (gps.NMEA_MSG_GLL, 0),
)

try:
    gps_uart.write(gps.build_cfg_rate_packet(GPS_UPDATE_HZ))
    utime.sleep(0.1)
    for nmea_msg_id, nmea_rate in GPS_NMEA_RATES:
        gps_uart.write(gps.build_cfg_msg_packet(nmea_msg_id, nmea_rate))
        utime.sleep(0.1)
    # "sent", not "configured": the receiver's UBX-ACK is never read back,
    # so this only reports what went out on the wire.
    print("GPS config sent: %dHz, NMEA RMC every fix, GGA 1/s, GSV every 5 s, GSA/VTG/GLL off" % GPS_UPDATE_HZ)
except Exception as e:
    print("GPS UBX config failed:", e)

utime.sleep(1)
while gps_uart.any():
    gps.gps_feed_bytes(gps_uart.read())

setup_session_log()

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
last_lcd_ms = utime.ticks_ms()
last_flush_ms = last_lcd_ms
last_gps_logged_time = None  # GPS time of the last fix written as a G row
last_storage_check_ms = last_lcd_ms
first_fix_seen = False  # sticky: set once on the first real fix, never cleared

# Startup (SD, GPS config) takes longer than the 0.85 s the FIFO holds, so it
# has overflowed by now. Start the loop on an empty, aligned FIFO.
fifo_overflows_seen = 0
if mpu:
    try:
        mpu.reset_fifo()
    except OSError as e:
        print("MPU FIFO reset failed:", e)

while True:
    # Read GPS - bounded to prevent blocking
    for _ in range(GPS_MAX_READS_PER_LOOP):
        if not gps_uart.any():
            break
        chunk = gps_uart.read(GPS_READ_CHUNK_BYTES)
        if not chunk:
            break
        gps.gps_feed_bytes(chunk)

    now_ms = utime.ticks_ms()
    has_gps_fix = gps.gps_fix and gps.gps_lat is not None and gps.gps_lon is not None

    if logging_started and utime.ticks_diff(now_ms, last_storage_check_ms) >= 1000:
        last_storage_check_ms = now_ms
        if not storage_has_headroom(logging_root):
            print("Stopping log before storage is full")
            if logging_root == "/sd":
                switch_to_flash_log()
            else:
                logger.sd_status = "FL"
                logging_started = False
                logging_disabled = True
                if logger.events_log_file:
                    logger.events_log_file.close()
                logger.events_log_file = None

    # first_fix_seen only anchors the distance baseline. Accel logging is not
    # gated on it: a cold start can take minutes, and a session without a fix
    # still records its strokes.
    if has_gps_fix and not first_fix_seen:
        gps.sync_distance_baseline()
        first_fix_seen = True

    # Read all buffered samples from the MPU FIFO
    if mpu:
        try:
            samples = mpu.read_fifo()
        except OSError as e:
            print("MPU I2C error:", e)
            samples = []
        if mpu.fifo_overflows != fifo_overflows_seen:
            fifo_overflows_seen = mpu.fifo_overflows
            print("MPU FIFO overflowed (loop stalled > 0.85 s): reset, samples lost")
        if samples:
            # The newest sample in the FIFO was captured at ~now_ms and the
            # batch is evenly spaced at the configured rate, so timestamp
            # backwards from the read. This re-anchors to the real clock on
            # every batch, so a failed read or a FIFO overflow can't put the
            # A rows out of step with the G rows.
            batch_start_ms = utime.ticks_add(now_ms, -(len(samples) - 1) * ACCEL_LOG_INTERVAL_MS)
            for i, sample in enumerate(samples):
                ts = utime.ticks_add(batch_start_ms, i * ACCEL_LOG_INTERVAL_MS)
                stroke_flag = detect_stroke(sample['accel'], ts)
                if stroke_flag:
                    stroke_pace.catch(ts)
                if logger.events_log_file:
                    try:
                        logger.events_log_file.write(logger.log_accel_row(
                            ts, sample['accel'], sample['gyro'], 1 if stroke_flag else 0
                        ))
                    except OSError as e:
                        print("Log write error:", e)
                        logger.events_log_file = None
                        logging_started = False
                        logger.sd_status = "KO"
                        switch_to_flash_log()
            # No timeout reset for spm_detector: it clears its SPM itself
            # once a stroke is overdue, and its adaptive big/small peak
            # levels should carry over across a brief pause rather than
            # being wiped and forced to re-bootstrap on every stop.

    # Distance and rowing time, in one-second steps even at 5 Hz. Gated on
    # the raw GPS speed, not a smoothed one: in steady rowing it almost never
    # dips below 1 m/s within a stroke, so the gate costs no real distance or
    # time.
    if has_gps_fix:
        gps.calculate_distance_traveled(
            rowing_speed_ms(gps.gps_speed_ms) is not None, DISTANCE_UPDATE_INTERVAL_MS / 1000.0)

    # One G row per GPS fix, keyed on the fix's own time.
    if logger.events_log_file:
        try:
            if has_gps_fix and gps.gps_time is not None and gps.gps_time != last_gps_logged_time:
                logger.events_log_file.write(logger.log_gps_row(gps.gps_time, gps.gps_speed_ms, get_spm(), gps.total_distance_m, now_ms))
                last_gps_logged_time = gps.gps_time
            # Flush periodically
            if utime.ticks_diff(now_ms, last_flush_ms) >= LOG_FLUSH_INTERVAL_MS:
                logger.events_log_file.flush()
                last_flush_ms = now_ms
        except OSError as e:
            print("Log write error:", e)
            logger.events_log_file = None
            logging_started = False
            logger.sd_status = "KO"
            switch_to_flash_log()

    # LCD
    if lcd and utime.ticks_diff(now_ms, last_lcd_ms) >= LCD_INTERVAL_MS:
        last_lcd_ms = now_ms
        stroke_pace.update(now_ms)
        pace_speed_ms = rowing_speed_ms(stroke_pace.speed_ms) if has_gps_fix else None
        try:
            # Status values are two letters: with all three the row is
            # exactly 20 characters.
            lcd.move_to(0, 0)
            lcd.putstr(display.fit_line("MEM:%s ACC:%s GPS:%s" % (logger.sd_status, "OK" if mpu else "KO", "OK" if has_gps_fix else "NO")))
            lcd.move_to(0, 1)
            lcd.putstr(display.fit_line("%s DIST:%.0fm" % (display.format_spm(get_spm()), gps.total_distance_m or 0)))
            lcd.move_to(0, 2)
            lcd.putstr(display.fit_line(display.format_pace(pace_speed_ms)))
            # Rowing time, not time since power-on: it only runs while the
            # distance does (see "Distance" above).
            lcd.move_to(0, 3)
            lcd.putstr(display.fit_line(display.format_rowing_time(gps.total_rowing_s)))
        except OSError as e:
            print("LCD I2C error:", e)

    utime.sleep_ms(0)
