"""Utility to list Pico flash contents and delete fallback log files.

Run directly on the Pico (e.g. via Thonny "Run current script", or
`mpremote run flash_logs.py`) to list and interactively delete the
pico_fallback_log*.csv files written by main.py when the SD card is
unavailable.
"""
import os

LOG_ROOT = "/"
LOG_PREFIX = "pico_fallback_log"
LOG_SUFFIX = ".csv"


def list_flash_logs():
    names = [
        name for name in os.listdir(LOG_ROOT)
        if name.startswith(LOG_PREFIX) and name.endswith(LOG_SUFFIX)
    ]
    names.sort()
    return names


def list_flash_files():
    names = list(os.listdir(LOG_ROOT))
    names.sort()
    return names


def print_flash_files():
    names = list_flash_files()
    if not names:
        print("Pico flash is empty.")
        return
    print("Files on Pico flash (%d):" % len(names))
    for name in names:
        try:
            size = os.stat("/" + name)[6]
            print("  %s (%d bytes)" % (name, size))
        except OSError as e:
            print("  %s (stat failed: %s)" % (name, e))


def delete_flash_logs(names=None, confirm=True):
    if names is None:
        names = list_flash_logs()
    if not names:
        print("No flash logs to delete.")
        return
    if confirm:
        answer = input("Delete %d log file(s) from flash? [y/N]: " % len(names))
        if answer.strip().lower() != "y":
            print("Aborted.")
            return
    for name in names:
        path = "/" + name
        try:
            os.remove(path)
            print("Deleted", path)
        except OSError as e:
            print("Failed to delete", path, e)


if __name__ == "__main__":
    print_flash_files()
    logs = list_flash_logs()
    if logs:
        delete_flash_logs(logs)
