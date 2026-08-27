from utime import sleep_ms

from lcd_api import LcdApi


class I2cLcd(LcdApi):
    LCD_CHR = 1
    LCD_CMD = 0
    LCD_BACKLIGHT = 0x08
    ENABLE = 0x04

    def __init__(self, i2c, addr, num_lines, num_columns):
        super().__init__(num_lines, num_columns)
        self.i2c = i2c
        self.addr = addr
        self.backlight = self.LCD_BACKLIGHT
        self._hal_write_init_nibble(0x03)
        sleep_ms(5)
        self._hal_write_init_nibble(0x03)
        sleep_ms(5)
        self._hal_write_init_nibble(0x03)
        sleep_ms(1)
        self._hal_write_init_nibble(0x02)
        sleep_ms(1)

        cmd = self.LCD_FUNCTION | self.LCD_4BIT_MODE
        if num_lines > 1:
            cmd |= self.LCD_2LINE
        if num_columns > 15:
            cmd |= self.LCD_5X8DOTS
        self.hal_write_command(cmd)
        self.display_control = self.LCD_DISPLAY_ON | self.LCD_CURSOR_OFF | self.LCD_BLINK_OFF
        self.hal_write_command(self.LCD_DISPLAY_CTRL | self.display_control)
        self.clear()
        self.entry_mode = self.LCD_ENTRY_LEFT | self.LCD_ENTRY_SHIFT_DECREMENT
        self.hal_write_command(self.LCD_ENTRY_MODE | self.entry_mode)

    def hal_write_command(self, cmd):
        self._write_byte(cmd, self.LCD_CMD)

    def hal_write_data(self, data):
        self._write_byte(data, self.LCD_CHR)

    def clear(self):
        super().clear()
        sleep_ms(2)

    def backlight_on(self):
        self.backlight = self.LCD_BACKLIGHT
        self._expander_write(0)

    def backlight_off(self):
        self.backlight = 0x00
        self._expander_write(0)

    def _hal_write_init_nibble(self, nibble):
        byte = (nibble << 4) | self.backlight
        self._expander_write(byte)
        self._pulse_enable(byte)

    def _write_byte(self, value, mode):
        high = mode | (value & 0xF0) | self.backlight
        low = mode | ((value << 4) & 0xF0) | self.backlight
        self._expander_write(high)
        self._pulse_enable(high)
        self._expander_write(low)
        self._pulse_enable(low)

    def _expander_write(self, data):
        try:
            self.i2c.writeto(self.addr, bytes([data]))
        except OSError:
            pass

    def _pulse_enable(self, data):
        self._expander_write(data | self.ENABLE)
        sleep_ms(1)
        self._expander_write(data & ~self.ENABLE)
        sleep_ms(1)