class LcdApi:
    LCD_CLR = 0x01
    LCD_HOME = 0x02
    LCD_ENTRY_MODE = 0x04
    LCD_DISPLAY_CTRL = 0x08
    LCD_SHIFT = 0x10
    LCD_FUNCTION = 0x20
    LCD_CGRAM = 0x40
    LCD_DDRAM = 0x80

    LCD_ENTRY_LEFT = 0x02
    LCD_ENTRY_SHIFT_DECREMENT = 0x00
    LCD_ENTRY_RIGHT = 0x00
    LCD_ENTRY_SHIFT_INCREMENT = 0x01

    LCD_DISPLAY_ON = 0x04
    LCD_DISPLAY_OFF = 0x00
    LCD_CURSOR_ON = 0x02
    LCD_CURSOR_OFF = 0x00
    LCD_BLINK_ON = 0x01
    LCD_BLINK_OFF = 0x00

    LCD_MOVE_LEFT = 0x00
    LCD_MOVE_RIGHT = 0x04

    LCD_8BIT_MODE = 0x10
    LCD_4BIT_MODE = 0x00
    LCD_2LINE = 0x08
    LCD_1LINE = 0x00
    LCD_5X10DOTS = 0x04
    LCD_5X8DOTS = 0x00

    def __init__(self, num_lines, num_columns):
        self.num_lines = num_lines
        self.num_columns = num_columns
        self.cursor_x = 0
        self.cursor_y = 0
        self.display_control = self.LCD_DISPLAY_ON | self.LCD_CURSOR_OFF | self.LCD_BLINK_OFF
        self.entry_mode = self.LCD_ENTRY_LEFT | self.LCD_ENTRY_SHIFT_DECREMENT
        self._line_offsets = (0x00, 0x40, 0x14, 0x54)

    def clear(self):
        self.hal_write_command(self.LCD_CLR)
        self.move_to(0, 0)

    def show_cursor(self):
        self.display_control |= self.LCD_CURSOR_ON
        self.hal_write_command(self.LCD_DISPLAY_CTRL | self.display_control)

    def hide_cursor(self):
        self.display_control &= ~self.LCD_CURSOR_ON
        self.hal_write_command(self.LCD_DISPLAY_CTRL | self.display_control)

    def blink_cursor(self):
        self.display_control |= self.LCD_BLINK_ON
        self.hal_write_command(self.LCD_DISPLAY_CTRL | self.display_control)

    def no_blink_cursor(self):
        self.display_control &= ~self.LCD_BLINK_ON
        self.hal_write_command(self.LCD_DISPLAY_CTRL | self.display_control)

    def display_on(self):
        self.display_control |= self.LCD_DISPLAY_ON
        self.hal_write_command(self.LCD_DISPLAY_CTRL | self.display_control)

    def display_off(self):
        self.display_control &= ~self.LCD_DISPLAY_ON
        self.hal_write_command(self.LCD_DISPLAY_CTRL | self.display_control)

    def putchar(self, char):
        if char == "\n":
            self.cursor_x = 0
            self.cursor_y = (self.cursor_y + 1) % self.num_lines
            self.move_to(self.cursor_x, self.cursor_y)
            return
        self.hal_write_data(ord(char))
        self.cursor_x += 1
        if self.cursor_x >= self.num_columns:
            self.cursor_x = 0
            self.cursor_y = (self.cursor_y + 1) % self.num_lines
            self.move_to(self.cursor_x, self.cursor_y)

    def putstr(self, string):
        for char in string:
            self.putchar(char)

    def move_to(self, cursor_x, cursor_y):
        self.cursor_x = cursor_x % self.num_columns
        self.cursor_y = cursor_y % self.num_lines
        ddram_address = self.cursor_x + self._line_offsets[self.cursor_y]
        self.hal_write_command(self.LCD_DDRAM | ddram_address)

    def custom_char(self, location, charmap):
        location &= 0x7
        self.hal_write_command(self.LCD_CGRAM | (location << 3))
        for char in charmap:
            self.hal_write_data(char)

    def hal_write_command(self, cmd):
        raise NotImplementedError

    def hal_write_data(self, data):
        raise NotImplementedError