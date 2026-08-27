# OpenCox RP2350

MicroPython firmware for the OpenCox rowing telemetry logger. Runs on a
Raspberry Pi Pico 2 W (RP2350) and records synchronised accelerometer, GPS and
stroke data to a microSD card as CSV.

Part of [OpenCox](https://github.com/simo-pagliu/OpenCox):

| Repo | Contents |
|------|----------|
| **OpenCox-RP2350** (this one) | Pico 2 W firmware |
| [OpenCoxUI](https://github.com/simo-pagliu/OpenCoxUI) | Web app and analysis pipeline for the logs |

## Hardware

- Raspberry Pi Pico 2 W (RP2350)
- MPU6050 accelerometer / gyroscope
- NMEA GPS module on UART
- microSD card breakout (SPI), card 32 GB or smaller
- 20x4 I2C character LCD (optional — the firmware runs without it)

## Wiring

All signals are on the left-hand side of the board (physical pins 1–20).
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

The SD breakout has no pull-up resistors of its own. Fit 10k from each of
these lines to the 3V3 rail:

- **MISO** (GP8) — the card releases this line whenever CS is high
- **CS** (GP9) — holds the card deselected while the Pico boots and GP9 is
  still an input

Do **not** fit one on CLK. It is host-driven, the SD specification does not
call for it, and it only adds load once the SPI clock is raised.

### Notes

- Physical pin 13 is GND, sitting between SD CS (pin 12) and SD CLK (pin 14).
  Leave it empty; a wire one row off lands on ground and fails silently.
- GPS RX goes to the Pico's TX and vice versa.
- On the Pico 2 W, GP23/24/25/29 belong to the CYW43 wireless chip. Nothing
  here touches them.

## Layout

| File | Purpose |
|------|---------|
| `main.py` | Entry point: hardware init, main loop, logging cadence |
| `sdcard.py` | Official MicroPython SD card driver (micropython-lib) |
| `gps.py` | NMEA parsing, UBX configuration, distance tracking |
| `mpu6050.py` | Accelerometer/gyro driver with FIFO sampling |
| `stroke_detection.py` | Catch and exit detection from acceleration slope |
| `logger.py` | CSV row builders and log-file naming |
| `display.py` | LCD text formatting helpers |
| `i2c_lcd.py`, `lcd_api.py` | HD44780-over-PCF8574 LCD driver |
| `flash_logs.py` | Utility: list and delete fallback logs from Pico flash |
| `gps_baud_scan.py` | Utility: find the GPS module's baud rate |

## Deploying

Copy every `.py` file to the root of the Pico's filesystem. With
[mpremote](https://docs.micropython.org/en/latest/reference/mpremote.html):

```bash
mpremote cp *.py :
```

Or open the folder in Thonny and upload each file to the device. `main.py`
runs automatically on boot.

## Log format

One CSV per session, written to `/sd` as `rowing_logNNN.csv`. If the SD card
is unavailable the firmware falls back to Pico flash as
`/pico_fallback_logNNN.csv`.

Rows are tagged by `record_type` in the first column:

| Type | Rate | Carries |
|------|------|---------|
| `A` | 100 Hz | Accelerometer, gyroscope, stroke flag |
| `G` | 1 Hz | Position, speed, SPM, distance, satellites, HDOP |
| `S` | 1 Hz | Diagnostics: raw UART byte count, NMEA sentence count, fix flag |

`S` rows are written regardless of fix state, so a session with no GPS lock is
still diagnosable from the CSV alone: a byte count stuck at zero means nothing
is arriving on the UART, while a rising byte count with a sentence count stuck
at zero means bytes arrive but never form a valid NMEA sentence.

## License

MIT
