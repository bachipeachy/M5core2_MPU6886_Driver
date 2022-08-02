from mpu6886 import MPU6886


if __name__ == "__main__":

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

        print("main> IMU temperature ..")
        temp = imu.temperature

        print("main> imuparms ..")
        [print('  ', k, '->', v) for k, v in imu.imuparms.items()]

