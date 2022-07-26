"""
The MIT License (MIT)

Copyright (c) 2022 bachipeachy@gmail.com
inspired by work done by Mika Tuupola at https://github.com/tuupola/micropython-mpu6886

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

import ustruct
import utime
from micropython import const


class MPU6886:
    """ MPU-6886 is a 6-axis motion tracking device that combines a 3-axis gyroscope and a 3-axis accelerometer """

    # Device specified MPU6886 registers
    GYRO_CONFIG = const(27)
    ACCEL_CONFIG = const(28)
    ACCEL_XOUT_H = const(59)
    TEMP_OUT_H = const(65)
    GYRO_XOUT_H = const(67)
    PWR_MGMT_1 = const(107)
    WHO_AM_I = const(117)

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
        self._imuparms = {'address': 0x68, 'accel_fs': MPU6886.FS_2G, 'accel_div': 16384, 'gyro_fs': MPU6886.FS_250DPS,
                          'gyro_div': 131, 'SG': 9.800665, 'RAD': 0.017453292519943, 'debug': False}

        [self.imuparms.update({k: v}) for k, v in kwargs.items()]

        if self.imuparms['accel_fs'] == MPU6886.FS_4G:
            self.imuparms['accel_div'] = 8192
        elif self.imuparms['accel_fs'] == MPU6886.FS_8G:
            self.imuparms['accel_div'] = 4096
        elif self.imuparms['accel_fs'] == MPU6886.FS_16G:
            self.imuparms['accel_div'] = 2048

        if self.imuparms['gyro_fs'] == MPU6886.FS_500DPS:
            self.imuparms['gyro_div'] = 65.5
        elif self.imuparms['gyro_fs'] == MPU6886.FS_1000DPS:
            self.imuparms['gyro_div'] = 32.8
        elif self.imuparms['gyro_fs'] == MPU6886.FS_2000DPS:
            self.imuparms['gyro_div'] = 16.4

        # validate existence of IMU
        if self.reg(MPU6886.WHO_AM_I) != b'\x19':
            raise RuntimeError("MPU6886 not found in I2C bus.")
        else:
            if self.imuparms['debug']:
                print("*  IMU id verified")

        # reset MPU6886 -- write b'\x10' to PWR_MGMT_1
        self.reg(MPU6886.PWR_MGMT_1, b'\x10')
        if self.imuparms['debug']:
            print("*  IMU reset")

        # autoselect MPU6886 clock -- write b'\x01' to PWR_MGMT_1
        utime.sleep_ms(100)
        self.reg(MPU6886.PWR_MGMT_1, b'\x01')
        if self.imuparms['debug']:
            print("*  IMU autoselect clock")

        # set acceleration full scale select, choose MPU6886.ACCEL_FS_SEL mask for 'no self test'
        self.reg(MPU6886.ACCEL_CONFIG, MPU6886.FS_2G)

        # set gyro full scale select, choose MPU6886.GYRO_FS_SEL mask for 'no self test'
        self.reg(MPU6886.GYRO_CONFIG, MPU6886.FS_250DPS)

        self.imuparms['accel_avg_tolerance'] = self.avg_tolerance('acceleration')
        self.imuparms['gyro_avg_tolerance'] = self.avg_tolerance('gyro')

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
        byt = self.i2c.readfrom_mem(self.imuparms['address'], r, nbytes)
        if nbytes == 6:
            byt = ustruct.unpack(">hhh", byt)
        elif nbytes == 2:
            byt = ustruct.unpack(">h", byt)
        return byt

    @property
    def temperature(self):
        """ Die temperature in deg F  """

        temp = self.reg(MPU6886.TEMP_OUT_H, nbytes=2)[0]
        temp = (temp / MPU6886.TEMP_SO) + MPU6886.TEMP_OFFSET
        temp = (1.8 * temp) + 32
        print("* imu temperature deg F -> ", temp)
        return temp

    @property
    def acceleration(self):
        """ returns tuple of X, Y, Z axis acceleration values in m/s^2 as floats """

        xyz = self.reg(MPU6886.ACCEL_XOUT_H, nbytes=6)
        accl = tuple([(value / self.imuparms['accel_div']) * self.imuparms['SG'] for value in xyz])
        if self.imuparms['debug']:
            print("    accl: ", accl)
        return accl

    @property
    def gyro(self):
        """ returns tuple of X, Y, Z axis gyro values in radians per second as floats. """

        xyz = self.reg(MPU6886.GYRO_XOUT_H, nbytes=6)

        gyro = [(value / self.imuparms['gyro_div']) * self.imuparms['RAD'] for value in xyz]
        if self.imuparms['debug']:
            print("    gyro: ", tuple(gyro))
        return tuple(gyro)

    def avg(self, sensor, delay=10, count=10):
        """ return average value for specified count of scans """

        xt, yt, zt = (0.0, 0.0, 0.0)
        n = float(count)
        while count:
            x, y, z = getattr(self, sensor)
            xt += x
            yt += y
            zt += z
            count -= 1
            utime.sleep_ms(delay)
        val = (xt / n, yt / n, zt / n)
        if self.imuparms['debug']:
            print("* avg_{} -> {}".format(sensor, val))
        return val

    def avg_tolerance(self, sensor, pause=1000, delay=10, count=10):
        """ static/stationary sensor readings and measured avg pct tolerance or variation in readings """

        avg = []
        avg1 = self.avg(sensor, delay=delay, count=count)
        utime.sleep_ms(pause)
        avg2 = self.avg(sensor, delay=delay, count=count)
        tolerance = (tuple(abs(x - y) for x, y in zip(avg1, avg2)))
        [avg.append((x + y) / 2) for x, y in zip(avg1, avg2)]
        pct = [100 * t / abs(x) for t, x in zip(tolerance, tuple(avg))]
        print(
            "* {} readings every {} msec interval, with {} msec pause between two '{} sample' averages  ..\n  avg -> {}\n  tolerance_pct -> {}"
            .format(sensor, delay, pause, count, tuple(avg), tuple(pct)))
        return tuple(avg), tuple(pct)

    def selftest(self, sensor='acceleration', axis='z'):
        """ return self test response as avg difference in sensor readings with self test enabled and disabled """

        mask = [m for m in ('ST_X', 'ST_Y', 'ST_Z') if m == 'ST_' + axis.upper()][0]
        mask = getattr(MPU6886, mask)

        retval = None
        if sensor == 'acceleration':
            self.reg(MPU6886.ACCEL_CONFIG, mask)
            val = self.avg_tolerance('acceleration')
            retval = (tuple(abs(x - y) for x, y in zip(val[0], self.imuparms['accel_avg_tolerance'][0])))
            self.reg(MPU6886.ACCEL_CONFIG, self.imuparms['accel_fs'])
        elif sensor == 'gyro':
            self.reg(MPU6886.GYRO_CONFIG, mask)
            val = self.avg_tolerance('gyro')
            retval = (tuple(abs(x - y) for x, y in zip(val[0], self.imuparms['gyro_avg_tolerance'][0])))
            self.reg(MPU6886.GYRO_CONFIG, self.imuparms['gyro_fs'])
        else:
            print("* error got {} but expecting 'acceleration' or 'gyro'".format(sensor))
        print("* {} selftest response val {}".format(sensor, retval))
        return retval
