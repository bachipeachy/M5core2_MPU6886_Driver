"""
The MIT License (MIT)

Copyright (c) 2022 bachipeachy@gmail.com

Inspired by work done by Mika Tuupola at https://github.com/tuupola/micropython-mpu6886
based on MPU6886 user guide ..
https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/core/MPU-6886-000193%2Bv1.1_GHIC_en.pdf

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import sys
import ustruct
import utime
from micropython import const


class MPU6886:
    """ MPU-6886 is a 6-axis motion tracking device that combines a 3-axis gyroscope and a 3-axis accelerometer """

    # Device specified MPU6886 registers
    SELF_TEST_X_ACCEL = const(13)
    SELF_TEST_Y_ACCEL = const(14)
    SELF_TEST_Z_ACCEL = const(15)
    GYRO_CONFIG = const(27)
    ACCEL_CONFIG = const(28)
    ACCEL_XOUT_H = const(59)
    TEMP_OUT_H = const(65)
    GYRO_XOUT_H = const(67)
    SELF_TEST_X_GYRO = const(80)
    SELF_TEST_Y_GYRO = const(81)
    SELF_TEST_Z_GYRO = const(82)
    PWR_MGMT_1 = const(107)
    WHO_AM_I = const(117)
    
    # in use register mask
    GYRO_STANDBY = b'\x10'
    CLKSEL = b'\x01'
    
    # GYRO_CONFIG register masks
    FS_250DPS = b'\x00'
    FS_500DPS = b'\x08'
    FS_1000DPS = b'\x10'
    FS_2000DPS = b'\x18'

    # ACCEL_CONFIG register masks
    FS_2G = b'\x00'
    FS_4G = b'\x08'
    FS_8G = b'\x10'
    FS_16G = b'\x18'

    # Self Test masks for X, Y and Z at highest sensor resolution
    ST_X = b'\x80'
    ST_Y = b'\x40'
    ST_Z = b'\x20'

    # temperature constants
    TEMP_OFFSET = 25
    TEMP_SO = 326.8

    def __init__(self, i2c, **kwargs):
        """ initialize and save avg stationary 6-axis sensor readings and tolerance """

        self.i2c = i2c
        self._imuparms = {'address': 0x68, 'accel_fs': MPU6886.FS_2G, 'accel_dial': None, 'gyro_fs': MPU6886.FS_250DPS,
                          'gyro_dial': None, 'SG': 9.800665, 'accel_ft': None, 'gyro_ft': None, 'debug': False}
        [print("* IGNORING ERROR invalid parm '{}'..".format(k)) for k in kwargs.keys() if
         k not in self.imuparms.keys()]

        [self.imuparms.update({k: v}) for k, v in kwargs.items()]

        if self.imuparms['accel_fs'] == MPU6886.FS_2G:
            self.imuparms['accel_dial'] = 2000
        elif self.imuparms['accel_fs'] == MPU6886.FS_4G:
            self.imuparms['accel_dial'] = 4000
        elif self.imuparms['accel_fs'] == MPU6886.FS_8G:
            self.imuparms['accel_dial'] = 8000
        elif self.imuparms['accel_fs'] == MPU6886.FS_16G:
            self.imuparms['accel_dial'] = 16000

        if self.imuparms['gyro_fs'] == MPU6886.FS_250DPS:
            self.imuparms['gyro_dial'] = 250
        elif self.imuparms['gyro_fs'] == MPU6886.FS_500DPS:
            self.imuparms['gyro_dial'] = 500
        elif self.imuparms['gyro_fs'] == MPU6886.FS_1000DPS:
            self.imuparms['gyro_dial'] = 10000
        elif self.imuparms['gyro_fs'] == MPU6886.FS_2000DPS:
            self.imuparms['gyro_dial'] = 20000

        # validate existence of IMU
        if self.reg(MPU6886.WHO_AM_I) != b'\x19':
            raise RuntimeError("MPU6886 not found in I2C bus.")
        else:
            if self.imuparms['debug']:
                print("* IMU id verified")

        # Gyro low power mode standby
        self.reg(MPU6886.PWR_MGMT_1, MPU6886.GYRO_STANDBY)
        utime.sleep_ms(100)
        if self.imuparms['debug']:
            print("* Set gyro in lowpower standby mode..")

        # auto select clock
        self.reg(MPU6886.PWR_MGMT_1, MPU6886.CLKSEL)
        if self.imuparms['debug']:
            print("* set autoselect clock..")

            # set accel full scale 2000 mG
        self.reg(MPU6886.ACCEL_CONFIG, self.imuparms['accel_fs'])
        if self.imuparms['debug']:
            print("* set acceleration dial@ {} mG".format(self.imuparms['accel_dial']))
        
        # set gyr0 full scale 250 dps/s
        utime.sleep_ms(10)
        self.reg(MPU6886.GYRO_CONFIG, self.imuparms['gyro_fs'])
        if self.imuparms['debug']:
            print("* set gyro dial@ {} dps/s".format(self.imuparms['gyro_dial']))
        
        # save factoy trim for self test
        self.imuparms['accel_ft'] = self._ft(sensor='accel')
        self.imuparms['gyro_ft'] = self._ft(sensor='gyro')

        if self.imuparms['debug']:
            print("* Initialization complete")

    @property
    def imuparms(self):
        return self._imuparms

    @imuparms.setter
    def imuparms(self, value):
        self._imuparms = value

    def reg(self, r, val=None, nbytes=1):
        """ read and write 'val if not None' into register for specified num of bytes """

        if val is not None:
            self.i2c.writeto_mem(self.imuparms['address'], r, val)
        utime.sleep_ms(1)
        byt = self.i2c.readfrom_mem(self.imuparms['address'], r, nbytes)
        if nbytes == 6:
            byt = ustruct.unpack(">hhh", byt)
        elif nbytes == 2:
            byt = ustruct.unpack(">h", byt)
        if self.imuparms['debug']:
            print("* reg#{} {} bytes -> {}".format(r, nbytes, byt))
        return byt

    @property
    def temperature(self):
        """ Die temperature in deg F  """

        t = self.reg(MPU6886.TEMP_OUT_H, nbytes=2)[0]
        t = (t / MPU6886.TEMP_SO) + MPU6886.TEMP_OFFSET
        t = round(((1.8 * t) + 32), 1)
        if self.imuparms['debug']:
            print("* imu temperature deg F -> ", t)
        return t

    @property
    def accel(self):
        """ returns tuple of X, Y, Z axis acceleration values mg (milli SG) as int """

        xyz = self.reg(MPU6886.ACCEL_XOUT_H, nbytes=6)
        result = tuple([int(self.imuparms['accel_dial'] * val / 32768) for val in xyz])
        if self.imuparms['debug']:
            print("  accl -> {} @fs = {} mG".format(result, self.imuparms['accel_dial']))
        return result

    @property
    def gyro(self):
        """ returns tuple of X, Y, Z axis gyro values in deg/sec as int. """

        xyz = self.reg(MPU6886.GYRO_XOUT_H, nbytes=6)
        gyro = tuple([int(self.imuparms['gyro_dial'] * val / 32768) for val in xyz])
        if self.imuparms['debug']:
            print("  gyro -> {} @fs = {} dps".format(gyro, self.imuparms['gyro_dial']))
        return gyro

    def _ft(self, sensor):
        """ returns factory trim values as a 3-int tuple for self test in UOM og mg or dps """
        dial = None
        if sensor == 'accel':
            dial = 2000
        elif sensor == 'gyro':
            dial = 250

        trim = []
        xyz = [getattr(MPU6886, 'SELF_TEST_' + axis + sensor.upper()) for axis in ('X_', 'Y_', 'Z_')]

        tuple([trim.append(int(dial * int.from_bytes(v, sys.byteorder) / 32768))
               for v in [self.reg(r) for r in xyz]])

        print("* IMU {} factory trims x, y, z -> {} {}".format(sensor, trim, 'mG' if sensor == 'accel' else 'dps'))
        return trim

    def _st(self, sensor):
        """ return self test response 'res' -> difference in readings with self test enabled and disabled """

        r = getattr(MPU6886, sensor.upper() + '_CONFIG')

        enabled = []
        for i, mask in enumerate((MPU6886.ST_X, MPU6886.ST_Y, MPU6886.ST_Z)):
            self.reg(r, mask)
            utime.sleep_ms(10)
            enabled.append(getattr(self, sensor)[i])

        fs = None
        if sensor == 'accel':
            fs = MPU6886.FS_2G
        elif sensor == 'gyro':
            fs = MPU6886.FS_250DPS
        self.reg(r, fs)
        utime.sleep_ms(10)
        disabled = getattr(self, sensor)

        st_r = tuple(x - y for x, y in zip(enabled, disabled))
        self.reg(r, self.imuparms[sensor + '_fs'])

        print("* {} self test response x, y, z -> {} {}\n  should be less than factory trim values -> {}".format(
            sensor, st_r, 'mG' if sensor == 'accel' else 'dps', self.imuparms[sensor + '_ft']))

        return st_r

    def selftest_experimental(self, sensor='accel', tolerance=None):
        """ compares self test response with factory trim for equality within specified allowable tolerance """

        if sensor not in ('accel', 'gyro'):
            print("* no implementation for sensor = {}".format(sensor))
            return
        else:
            if tolerance is None:
                if sensor == 'accel':
                    tolerance = 40
                elif sensor == 'gyro':
                    tolerance = 1

        st = [abs(v) for v in getattr(self, '_st')(sensor=sensor)]

        if max(st) > 2 * tolerance:
            result = {0: "failed", 1: "failed", 2: "failed"}

            ft = self.imuparms[sensor + '_ft']
            [result.update({i: "passed"}) for i, (x, y) in enumerate(zip(st, ft)) if x <= y]
            result = tuple(result.values())
            print("* {} selftest x, y, z -> {}, allowable tolerance of 2*{} {}".format(
                sensor, result, tolerance, 'mG' if sensor == 'accel' else 'dps'))
            return result
        else:
            print("* {} test passed\n"
                  "  the max self test response of {} is within allowable tolerance of 2*{}"
                  .format(sensor, max(st), tolerance))

"""
if __name__ == "__main__":
    """ test MPU6886 class """

    from machine import SoftI2C, Pin

    i2c = SoftI2C(scl=Pin(22), sda=Pin(21))

    print("main> Initializing  IMU ..")
    imu = MPU6886(i2c, debug=False)

    print("main> running imu.accel fs: 2000 mG ..")
    for i in range(5):
        print("{} > {}".format(i + 1, imu.accel))

    print("main> running imu.gyro fs: 250 dps/s ..")
    for i in range(5):
        print("{} > {}".format(i + 1, imu.gyro))

    print("main> running accel self test ..")
    imu.selftest_experimental(sensor='accel')

    print("main> running gyro self test ..")
    imu.selftest_experimental(sensor='gyro')
"""