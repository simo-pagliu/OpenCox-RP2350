# OpenCox RP2350

MicroPython firmware for OpenCox, a rowing telemetry logger. It runs on a
Raspberry Pi Pico 2 W (RP2350) mounted in the boat, shows stroke rate,
distance, pace and rowing time live on a small LCD, and records the
accelerometer and GPS to a microSD card as CSV for later analysis.

Part of [OpenCox](https://github.com/simo-pagliu/OpenCox):

| Repo | Contents |
|------|----------|
| **OpenCox-RP2350** (this one) | Firmware for the logger |
| [OpenCox-WebUI](https://github.com/simo-pagliu/OpenCox-WebUI) | Web page that analyses the logs |

## Features

- **Stroke rate (SPM)** from the boat's fore-aft acceleration: every stroke
  shows a deep deceleration at the catch and a sharp rise in the drive. SPM
  is the median of the last 3 stroke intervals.
- **Pace per 500 m** from the GPS speed, averaged over the last 2 strokes and
  updated once per stroke, shown to the half second.
- **Distance and rowing time** that only count while the boat moves at
  1 m/s or more, so stops and drifting don't inflate them.
- **Logging** of the accelerometer and gyroscope at 100 Hz and of every GPS
  fix at 5 Hz, to one CSV file per session on the microSD card. Without a
  card it logs to the Pico's own flash instead.

## Hardware

- Raspberry Pi Pico 2 W (RP2350)
- MPU6050 accelerometer / gyroscope
- u-blox NEO-6M GPS (e.g. a GY-NEO6MV2 board), configured to 5 Hz at startup
- microSD card breakout (SPI), with a FAT32 card of 32 GB or less
- 20x4 I2C character LCD with a PCF8574 backpack (optional: the firmware runs
  without it)

## Wiring

All signals are on the left-hand side of the Pico (physical pins 1–20).
Power comes from pins 36 and 38 on the right, feeding a 3V3 rail and a GND
rail that every module connects to.

| Module | Module pin | Pico physical pin | GPIO |
|--------|-----------|-------------------|------|
| GPS | RX | 1 | GP0 (UART0 TX) |
| GPS | TX | 2 | GP1 (UART0 RX) |
| microSD | MISO | 11 | GP8 (SPI1 RX) |
| microSD | CS | 12 | GP9 |
| microSD | CLK | 14 | GP10 (SPI1 SCK) |
| microSD | MOSI | 15 | GP11 (SPI1 TX) |
| LCD | SDA | 16 | GP12 (I2C0) |
| LCD | SCL | 17 | GP13 (I2C0) |
| MPU6050 | SDA | 19 | GP14 (I2C1) |
| MPU6050 | SCL | 20 | GP15 (I2C1) |
| all | VCC | 36 | 3V3(OUT) |
| all | GND | 38 | GND |

### Pull-up resistors

Many SD breakouts have no pull-up resistors of their own. Fit 10k from each
of these lines to the 3V3 rail:

- **MISO** (GP8): the card releases this line whenever CS is high.
- **CS** (GP9): keeps the card deselected while the Pico boots and GP9 is
  still an input.

Don't fit one on CLK: it is driven by the Pico, and the SD specification
doesn't call for it.

### Notes

- Physical pin 13 is GND, between SD CS (pin 12) and SD CLK (pin 14). A wire
  one row off lands on ground and fails silently.
- GPS RX goes to the Pico's TX and vice versa.
- On the Pico 2 W, GP23/24/25/29 belong to the wireless chip. Nothing here
  uses them.

## Installation

1. Install MicroPython for the Raspberry Pi Pico 2 W (the `RPI_PICO2_W`
   build from [micropython.org](https://micropython.org/download/RPI_PICO2_W/);
   tested with v1.29.0): hold BOOTSEL while plugging the board in, and copy
   the `.uf2` file to the drive that appears.
2. Copy every `.py` file in this repository to the root of the Pico's
   filesystem, for example with
   [mpremote](https://docs.micropython.org/en/latest/reference/mpremote.html):

   ```bash
   mpremote cp *.py :
   ```

   Thonny or the MicroPico extension for VS Code work too.
3. Reset the board. `main.py` runs automatically at every power-up.

## Use

Insert the microSD card and power the logger. It starts a new log file at
every power-up and logs until it is switched off. The LCD shows:

```
MEM:SD ACC:OK GPS:OK
SPM:20 DIST:1234m
PACE:2:05.5/500m
TIME:00:12:34
```

| Field | Meaning |
|-------|---------|
| `MEM` | `SD`: logging to the card. `OB`: no card, logging to the Pico's flash. `FL`: storage full. `KO`: logging failed |
| `ACC` | `OK` / `KO`: the MPU6050 was found |
| `GPS` | `OK`: GPS fix. `NO`: no fix yet |
| `SPM` | Strokes per minute; `--` when no stroke for a few seconds |
| `DIST` | Distance rowed, counted while moving at 1 m/s or more |
| `PACE` | Time per 500 m over the last 2 strokes; when no stroke has been detected for a few seconds, the average of the last 5 s, updated every second |
| `TIME` | Rowing time, counted like the distance |

From a standing start, SPM appears after a few strokes (the first stroke has
no deceleration at the catch to detect), and the per-stroke pace one stroke
later: see [Known limitations](#known-limitations).

To analyse a session, take the card out and open its `rowing_logNNN.csv` in
[OpenCox-WebUI](https://github.com/simo-pagliu/OpenCox-WebUI). The Pico's flash holds
only about 2.5 MB, so a log without a card fills it within minutes; the
firmware stops before it is full.

## Configuration

Settings are constants at the top of `main.py`:

| Constant | Default | What it does |
|----------|---------|--------------|
| `SPM_AXIS_WEIGHTS` | `(0.0, 0.766, 0.643)` | Accelerometer axes along the boat's direction of travel; depends on how the board is mounted (see below) |
| `MIN_SPM`, `SPM_MAX` | 14, 50 | Range of plausible stroke rates |
| `MIN_ROWING_SPEED_MS` | 1.0 | Speed below which distance and time stop counting |
| `PACE_AVG_STROKES` | 2 | Strokes the LCD pace averages over |
| `GPS_UPDATE_HZ` | 5 | GPS fix rate (5 Hz is the NEO-6M's maximum) |

Pin assignments are set there too.

### Mounting and the SPM axis

The stroke detector reads the acceleration along the boat's direction of
travel, built from the accelerometer axes as `wx·ax + wy·ay + wz·az` with the
weights in `SPM_AXIS_WEIGHTS`. The weights are normalised, so only their
proportions matter, and a negative weight flips an axis.

The default is for a board on the footrest, tilted about 40° about the
sensor's X axis, with X across the boat: the direction of travel then lies in
the sensor's Y-Z plane, at (0, cos 40°, sin 40°).

| Mounting | `SPM_AXIS_WEIGHTS` |
|---|---|
| Tilted ~40° about X, X across the boat (default) | `(0.0, 0.766, 0.643)` |
| Tilted about X, angle unknown | `(0, 1, 1)` |
| Flat, Y along the boat | `(0, 1, 0)` |
| Flat, X along the boat | `(1, 0, 0)` |
| Any other | the direction of travel in sensor axes, e.g. `(0.2, 0.7, 0.6)` |

Choose the signs so that the catch shows up as a dip in the combined signal:
the detector is tuned for that. All-zero weights are rejected at startup.

To find the weights for a new mounting:

- **From gravity:** with the boat still and level, and X across the boat, the
  tilt is `atan2(−ay, az)` and the weights are `(0, cos tilt, sin tilt)`.
- **From a log:** plot a minute of rowing and use the axes that show the deep
  dip at every catch, weighted by how deep the dip is on each.

On two test sessions (about 2.5 h of rowing), the 40° combination missed
0.3% and 1.5% of strokes against an offline reference of every catch, Y
alone 5.4% and 3.9%, and Z alone 15% and 24%. None counted extra strokes.

## Log format

One CSV file per session, written to the card as `rowing_logNNN.csv`, or to
the Pico's flash as `pico_fallback_logNNN.csv` without a card. The first
column, `record_type`, tells the two kinds of row apart:

| Type | Rate | Carries |
|------|------|---------|
| `A` | 100 Hz | Accelerometer, gyroscope, stroke flag |
| `G` | 5 Hz, every GPS fix | Date and time, position, speed, SPM, distance, fix quality |

Every row has all the columns; each type fills its own and leaves the rest
empty.

| Column | Rows | Unit / format |
|--------|------|---------------|
| `record_type` | all | `A` or `G` |
| `date` | G | `dd/mm/yy`, UTC, from the GPS |
| `time` | G | seconds since midnight UTC of the GPS fix |
| `uptime_ms` | all | milliseconds since the board started |
| `accel_x`, `accel_y`, `accel_z` | A | g |
| `gyro_x`, `gyro_y`, `gyro_z` | A | °/s |
| `speed_mps` | G | GPS speed over ground, m/s |
| `spm` | G | strokes per minute shown at that moment, 0 if none |
| `gps_lat`, `gps_lon` | G | decimal degrees |
| `distance_m` | G | distance rowed so far, m (as on the LCD) |
| `stroke_flag` | A | 1 on the sample where a catch was detected, else 0 |
| `sats` | G | satellites used |
| `hdop` | G | horizontal dilution of precision |
| `fix_quality` | G | GPS fix quality (GGA) |
| `course_deg` | G | course over ground, degrees |

The detector recognises a catch by the sharp rise in the drive, so
`stroke_flag` sits about 1 s after the deceleration of the catch itself, by
the same amount on every stroke.

## Code layout

| File | Purpose |
|------|---------|
| `main.py` | Entry point: settings, hardware setup, main loop, LCD and logging |
| `spm_detection.py` | Stroke detection and SPM from the fore-aft acceleration |
| `pace.py` | LCD pace, averaged over the last strokes and updated at each catch |
| `gps.py` | NMEA parsing, GPS configuration (UBX), distance and rowing time |
| `mpu6050.py` | MPU6050 driver with FIFO sampling |
| `logger.py` | CSV rows and log file naming |
| `display.py` | LCD text formatting |
| `sdcard.py` | SD card driver (from micropython-lib) |
| `i2c_lcd.py`, `lcd_api.py` | HD44780 LCD over a PCF8574 I2C backpack |

## Known limitations

Neither of these gets in the way of using the logger, but both could be
better:

- **Pace at a standing start.** The per-stroke pace needs three detected
  catches in a row. Until then the LCD holds the 5-second average taken just
  before the first catch, which still counts the seconds at rest. For roughly
  the first 5–10 s of a start the pace reads `--` or a pace much slower than
  the boat's, and the first per-stroke values still include the slow
  strokes of the start.
- **Pace stability.** The pace comes from the GPS speed, which is noisy
  while rowing. Averaging whole strokes removes the surge within each
  stroke, but in steady rowing the pace still moves about. Better antenna
  placement and receiver tuning (see the roadmap) could help.

## Roadmap

- **Rigid mount and enclosure.** A proper mount fixed to the boat, instead
  of a board strapped to the footrest, removes the noise of the board moving
  on its own.
- **GPS antenna placement.** Temporary mounting is partially obscured by the screen
- **Find the SPM axis automatically.** Gravity, read while the boat is still,
  gives the tilt; the direction of travel is the horizontal direction with
  the strongest acceleration while rowing.
- **Write the logger's settings into the log**: firmware version,
  `SPM_AXIS_WEIGHTS` and GPS rate, so analysis software knows how a log was
  recorded. OpenCox-WebUI checks the CSV header strictly, so this changes both
  repositories together.
- **GPS receiver tuning** for use on water: explore implementation of dynamic platform model and
  the satellite elevation mask.

## License

OpenCox RP2350 is source-available under the
[PolyForm Noncommercial License 1.0.0](LICENSE.md). You may use, study,
change and share it, and build loggers with it, for any noncommercial
purpose (personal use, hobby projects, research, teaching, clubs and other
noncommercial organisations), as long as the license and its
`Required Notice` line come with any copy you pass on. Commercial use is not
covered by this license: to use OpenCox commercially, ask the author for
permission by writing an email to pagliuca.simone01@gmail.com.

`sdcard.py`, `lcd_api.py` and `i2c_lcd.py` come from other projects and keep
their own MIT licenses: see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
