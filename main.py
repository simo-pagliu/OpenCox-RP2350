from machine import Pin, I2C, SPI, UART
import utime
import sdcard
import os
from mpu6050 import MPU6050
from i2c_lcd import I2cLcd
from stroke_detection import PicoStrokeDetector
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

MPU_SAMPLE_RATE_HZ = 100
MPU_DLPF_CFG = 3

LOG_FLUSH_INTERVAL_MS = 200
LCD_INTERVAL_MS = 500
GPS_UPDATE_HZ = 1
GPS_MAX_READS_PER_LOOP = 4
GPS_READ_CHUNK_BYTES = 64

MIN_SPM = 14
MAX_SPM = 55
STROKE_MAX_INTERVAL_MS = 60000 / MIN_SPM  # ~4286 ms at 14 SPM

ACCEL_LOG_INTERVAL_MS = 10       # 100 Hz
GPS_LOG_INTERVAL_MS = 1000       # 1 Hz
DISTANCE_UPDATE_INTERVAL_MS = 1000
STORAGE_RESERVE_BYTES = 128 * 1024

# ---------------------------------------------------------------------------
# Stroke detection
# ---------------------------------------------------------------------------
stroke_detector = PicoStrokeDetector(min_spm=MIN_SPM, max_spm=MAX_SPM)


def calculate_accel_magnitude(accel_data):
    x = accel_data.get("x", 0.0)
    y = accel_data.get("y", 0.0)
    z = accel_data.get("z", 0.0)
    return (x**2 + y**2 + z**2) ** 0.5


def detect_stroke(accel_data, current_time_ms):
    accel_x = accel_data.get("x", 0.0)
    accel_y = accel_data.get("y", 0.0)
    accel_z = accel_data.get("z", 0.0)
    catch_detected = stroke_detector.detect_stroke(accel_x, accel_y, accel_z, current_time_ms)
    if stroke_detector.exit_detected():
        print("Blade exit detected")
    if catch_detected:
        spm_value = stroke_detector.get_spm()
        if spm_value > 0:
            print("Catch: %d SPM" % spm_value)
        else:
            print("First catch detected")
    return catch_detected


def get_spm():
    return stroke_detector.get_spm()


def get_adaptive_threshold():
    return 1.0


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
    print("Stroke detection ready (catch/exit acceleration-change detection).")
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
    # The module has no pull-up resistors of its own (confirmed by the vendor)
    # and the card releases MISO whenever CS is high, so nothing holds that
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

# Sentences per navigation solution. GLL is switched off because nothing
# parses it and it is pure UART load; every other sentence here feeds a
# column of the log or a field on the display.
GPS_NMEA_RATES = (
    (gps.NMEA_MSG_GGA, 1),
    (gps.NMEA_MSG_RMC, 1),
    (gps.NMEA_MSG_GSA, 1),
    (gps.NMEA_MSG_GSV, 1),
    (gps.NMEA_MSG_VTG, 1),
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
    print("GPS config sent: %dHz, NMEA GGA/RMC/GSA/GSV/VTG on, GLL off" % GPS_UPDATE_HZ)
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
last_gps_write_ms = last_lcd_ms
last_storage_check_ms = last_lcd_ms
first_fix_seen = False  # sticky: set once on the first real fix, never cleared

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

    # first_fix_seen only anchors the distance baseline. Accel logging is
    # deliberately NOT gated on it: a cold start can take minutes, and gating
    # the IMU on the fix meant a session where the receiver never resolved a
    # position produced a file with no stroke data in it at all.
    if has_gps_fix and not first_fix_seen:
        gps.sync_distance_baseline()
        first_fix_seen = True

    # Read all buffered samples from MPU FIFO (100Hz)
    if mpu:
        try:
            samples = mpu.read_fifo()
        except OSError as e:
            print("MPU I2C error:", e)
            samples = []
        if samples:
            # The newest sample in the FIFO was captured at ~now_ms and the
            # batch is evenly spaced at the configured rate, so timestamp
            # backwards from the read. This re-anchors to the real clock on
            # every batch. The old scheme advanced a counter by exactly 10 ms
            # per sample and never resynced, so one failed read or FIFO
            # overflow left the A rows permanently out of step with the G rows
            # in the same file.
            batch_start_ms = utime.ticks_add(now_ms, -(len(samples) - 1) * ACCEL_LOG_INTERVAL_MS)
            for i, sample in enumerate(samples):
                ts = utime.ticks_add(batch_start_ms, i * ACCEL_LOG_INTERVAL_MS)
                stroke_flag = detect_stroke(sample['accel'], ts)
                if logger.events_log_file:
                    try:
                        logger.events_log_file.write(logger.log_accel_row(
                            ts, sample['accel'], sample['gyro'], 1 if stroke_flag else 0
                        ))
                        # This catch closed the previous stroke, so its
                        # metrics are now final: emit them as one S row
                        # rather than smearing them across whichever A rows
                        # each value happened to become available on.
                        if stroke_detector.stroke_record_available():
                            logger.events_log_file.write(logger.log_stroke_row(
                                ts, stroke_detector.get_stroke_record()
                            ))
                    except OSError as e:
                        print("Log write error:", e)
                        logger.events_log_file = None
                        logging_started = False
                        logger.sd_status = "KO"
                        switch_to_flash_log()
            # Stroke timeout is measured against the newest sample, which is
            # now_ms. The old latest_ts double-counted: last_accel_ms had
            # already been advanced by the whole batch before another
            # (len - 1) intervals were added on top.
            if stroke_detector.last_stroke_time_ms > 0 and utime.ticks_diff(now_ms, stroke_detector.last_stroke_time_ms) > STROKE_MAX_INTERVAL_MS:
                stroke_detector.reset()
    
    # Distance
    if has_gps_fix:
        gps.calculate_distance_traveled()

    # Write GPS to SD at 1Hz
    if logger.events_log_file:
        try:
            if has_gps_fix and utime.ticks_diff(now_ms, last_gps_write_ms) >= GPS_LOG_INTERVAL_MS:
                logger.events_log_file.write(logger.log_gps_row(gps.gps_time, gps.gps_speed_ms, get_spm(), gps.total_distance_m, now_ms))
                last_gps_write_ms = now_ms
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
        try:
            lcd.move_to(0, 0)
            lcd.putstr(display.fit_line("MEM:%s ACC:%s GPS:%s" % (logger.sd_status, "OK" if mpu else "FAIL", "FIX" if has_gps_fix else "NO")))
            lcd.move_to(0, 1)
            lcd.putstr(display.fit_line("SPM:%d DIST:%.0fm" % (get_spm(), gps.total_distance_m or 0)))
            lcd.move_to(0, 2)
            lcd.putstr(display.fit_line("SPD:%.1fm/s" % (gps.gps_speed_ms or 0)))
            lcd.move_to(0, 3)
            # used/in view: during a cold start the first number sits at 0
            # while the second climbs, which is the only on-boat sign that
            # the receiver is acquiring rather than dead.
            lcd.putstr(display.fit_line("SAT:%s/%s HDOP:%.1f" % (
                gps.gps_sats or "--", gps.gps_sats_in_view or "--", gps.gps_hdop or 0)))
        except OSError as e:
            print("LCD I2C error:", e)

    utime.sleep_ms(0)
