"""Diagnostic: scan GPS baud rates across multiple UART pin pairs.

Run directly on the Pico (e.g. via Thonny "Run current script") with a live
terminal attached. For each (UART id, rx pin) combo below, listens on every
candidate baud rate in turn and reports how many raw bytes were received and
a decoded sample. Works indoors, with no fix and no sky view required -- a
powered GPS module streams NMEA text continuously regardless of fix state.

This version also checks a second UART peripheral on different pins, to
isolate a damaged/faulty Pico input pin from a dead module or broken wire:
RX pins are inputs, so it's safe to add a second jumper from the GPS
module's TX line straight to GP5 (in addition to its existing connection to
GP1) -- no electrical conflict, since nothing is driving GP1/GP5 from the
Pico side. Leave the tx= pins below unconnected/dangling; they're only
required by the UART constructor, not by this receive-only test.

If GP5 (UART1) picks up bytes that GP1 (UART0) never does, the fault is
isolated to GP1 (or its wire) specifically, not the GPS module. If both
show zero, the fault is upstream of the Pico (module TX pad/output, or the
wire feeding it) and a USB-TTL adapter is the next diagnostic step.
"""
from machine import Pin, UART
import utime

CANDIDATE_BAUDS = [4800, 9600, 19200, 38400, 57600, 115200]
# (uart_id, tx_pin, rx_pin, label) -- add a jumper from GPS TX to GP5 to
# populate the second row without touching the existing GP0/GP1 wiring.
UART_CONFIGS = [
    (0, 0, 1, "UART0 GP0(tx)/GP1(rx) -- existing wiring"),
    (1, 4, 5, "UART1 GP4(tx)/GP5(rx) -- add a jumper GPS-TX -> GP5"),
]
LISTEN_MS = 3000
SAMPLE_BYTES = 120


def scan_baud(uart_id, tx_pin, rx_pin, baud):
    uart = UART(uart_id, baudrate=baud, tx=Pin(tx_pin), rx=Pin(rx_pin))
    utime.sleep_ms(50)  # let the UART settle after reconfiguration
    total_bytes = 0
    sample = b""
    start_ms = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), start_ms) < LISTEN_MS:
        if uart.any():
            chunk = uart.read()
            if chunk:
                total_bytes += len(chunk)
                if len(sample) < SAMPLE_BYTES:
                    sample += chunk[: SAMPLE_BYTES - len(sample)]
        else:
            utime.sleep_ms(10)
    return total_bytes, sample


def looks_like_nmea(sample):
    try:
        text = sample.decode("utf-8", "ignore")
    except Exception:
        return False
    return "$GP" in text or "$GN" in text or "$GL" in text or "$GA" in text


def main():
    print("=== GPS baud/pin scan: %d ms per rate ===" % LISTEN_MS)
    print("(module should be powered and idle; no fix/sky view needed)\n")
    best = None
    any_bytes_anywhere = False
    for uart_id, tx_pin, rx_pin, label in UART_CONFIGS:
        print("--- %s ---" % label)
        for baud in CANDIDATE_BAUDS:
            try:
                total_bytes, sample = scan_baud(uart_id, tx_pin, rx_pin, baud)
            except Exception as e:
                print("%6d baud: error: %s" % (baud, e))
                continue
            if total_bytes > 0:
                any_bytes_anywhere = True
            tag = ""
            if total_bytes > 0 and looks_like_nmea(sample):
                tag = "  <-- looks like valid NMEA text"
                if best is None:
                    best = (uart_id, tx_pin, rx_pin, baud)
            print("%6d baud: %5d bytes  sample=%r%s" % (baud, total_bytes, sample[:60], tag))
        print()

    if best is not None:
        uart_id, tx_pin, rx_pin, baud = best
        print("Best candidate: UART(%d, tx=Pin(%d), rx=Pin(%d)) at %d baud" % (uart_id, tx_pin, rx_pin, baud))
    elif any_bytes_anywhere:
        print("Bytes arrived somewhere, but nothing looked like valid NMEA text.")
        print("Check the raw samples above for a clue (garbled = likely wrong baud).")
    else:
        print("Zero bytes on every pin pair and every baud rate.")
        print("This points upstream of the Pico entirely: the module's TX")
        print("output, or the wire feeding it, rather than a damaged Pico pin.")


if __name__ == "__main__":
    main()
