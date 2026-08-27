import utime


class MPU6050:
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    ACCEL_XOUT_H = 0x3B
    TEMP_OUT_H = 0x41
    GYRO_XOUT_H = 0x43
    FIFO_EN = 0x23
    USER_CTRL = 0x6A
    FIFO_COUNT_H = 0x72
    FIFO_COUNT_L = 0x73
    FIFO_R_W = 0x74

    ACCEL_SCALE = 16384.0
    GYRO_SCALE = 131.0

    # FIFO packet layout: accel_x/y/z + gyro_x/y/z, 2 bytes each (temp is not
    # enabled in FIFO_EN, so no TEMP_OUT bytes are interleaved between them).
    FIFO_PACKET_SIZE = 12

    def __init__(self, i2c, addr=0x68, sample_rate_hz=100, dlpf_cfg=3):
        self.i2c = i2c
        self.addr = addr
        self.sample_rate_hz = 0
        self.dlpf_cfg = 0

        self._write_u8(self.PWR_MGMT_1, 0x00)
        self._write_u8(self.GYRO_CONFIG, 0x00)
        self._write_u8(self.ACCEL_CONFIG, 0x00)
        self.set_dlpf(dlpf_cfg)
        self.set_sample_rate(sample_rate_hz)
        self.enable_fifo(accel=True, gyro=True)

    def _write_u8(self, register, value):
        self.i2c.writeto_mem(self.addr, register, bytes([value & 0xFF]))

    def _read_u8(self, register):
        data = self.i2c.readfrom_mem(self.addr, register, 1)
        if len(data) < 1:
            raise OSError("short read from MPU6050 register %d" % register)
        return data[0]

    def _read_s16(self, register):
        data = self.i2c.readfrom_mem(self.addr, register, 2)
        if len(data) < 2:
            raise OSError("short read from MPU6050 register %d" % register)
        value = (data[0] << 8) | data[1]
        if value & 0x8000:
            value -= 0x10000
        return value

    def set_dlpf(self, dlpf_cfg):
        if dlpf_cfg < 0 or dlpf_cfg > 6:
            raise ValueError("dlpf_cfg must be between 0 and 6")
        self._write_u8(self.CONFIG, dlpf_cfg)
        self.dlpf_cfg = dlpf_cfg

    def set_sample_rate(self, sample_rate_hz):
        if sample_rate_hz <= 0:
            raise ValueError("sample_rate_hz must be > 0")
        gyro_output_rate_hz = 8000 if self.dlpf_cfg in (0, 7) else 1000
        divider = int((gyro_output_rate_hz / sample_rate_hz) - 1)
        if divider < 0:
            divider = 0
        if divider > 255:
            divider = 255
        self._write_u8(self.SMPLRT_DIV, divider)
        self.sample_rate_hz = gyro_output_rate_hz / (divider + 1)

    def enable_fifo(self, accel=True, gyro=True, temp=False):
        value = 0
        if accel:
            value |= 0x08
        if gyro:
            value |= 0x70
        if temp:
            value |= 0x80
        # Reset first, then enable FIFO mode and its selected sources.
        self._write_u8(self.USER_CTRL, 0x04)
        utime.sleep_ms(50)
        self._write_u8(self.FIFO_EN, value)
        self._write_u8(self.USER_CTRL, 0x40)

    def fifo_count(self):
        high = self._read_u8(self.FIFO_COUNT_H)
        low = self._read_u8(self.FIFO_COUNT_L)
        return (high << 8) | low

    @staticmethod
    def _to_signed16(value):
        return value - 0x10000 if value & 0x8000 else value

    def read_fifo(self):
        count = self.fifo_count()
        if count == 0:
            return []
        # Only read whole packets; the FIFO can be mid-write on the next byte.
        usable = count - (count % self.FIFO_PACKET_SIZE)
        if usable == 0:
            return []
        data = self.i2c.readfrom_mem(self.addr, self.FIFO_R_W, usable)
        if len(data) < usable:
            return []
        samples = []
        for i in range(0, usable, self.FIFO_PACKET_SIZE):
            accel_x = self._to_signed16((data[i] << 8) | data[i+1])
            accel_y = self._to_signed16((data[i+2] << 8) | data[i+3])
            accel_z = self._to_signed16((data[i+4] << 8) | data[i+5])
            gyro_x = self._to_signed16((data[i+6] << 8) | data[i+7])
            gyro_y = self._to_signed16((data[i+8] << 8) | data[i+9])
            gyro_z = self._to_signed16((data[i+10] << 8) | data[i+11])
            samples.append({
                'accel': {
                    'x': accel_x / self.ACCEL_SCALE,
                    'y': accel_y / self.ACCEL_SCALE,
                    'z': accel_z / self.ACCEL_SCALE
                },
                'gyro': {
                    'x': gyro_x / self.GYRO_SCALE,
                    'y': gyro_y / self.GYRO_SCALE,
                    'z': gyro_z / self.GYRO_SCALE
                }
            })
        return samples

    def get_accel_data(self, g=False):
        try:
            x = self._read_s16(self.ACCEL_XOUT_H) / self.ACCEL_SCALE
            y = self._read_s16(self.ACCEL_XOUT_H + 2) / self.ACCEL_SCALE
            z = self._read_s16(self.ACCEL_XOUT_H + 4) / self.ACCEL_SCALE
            if g:
                return {"x": x, "y": y, "z": z}
            gravity = 9.80665
            return {"x": x * gravity, "y": y * gravity, "z": z * gravity}
        except OSError:
            return None

    def get_gyro_data(self):
        try:
            return {
                "x": self._read_s16(self.GYRO_XOUT_H) / self.GYRO_SCALE,
                "y": self._read_s16(self.GYRO_XOUT_H + 2) / self.GYRO_SCALE,
                "z": self._read_s16(self.GYRO_XOUT_H + 4) / self.GYRO_SCALE,
            }
        except OSError:
            return None

    def get_temp(self):
        raw = self._read_s16(self.TEMP_OUT_H)
        return (raw / 340.0) + 36.53
