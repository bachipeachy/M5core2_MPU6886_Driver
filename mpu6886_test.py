from mpu6886 import MPU6886

if __name__ == "__main__":
    """ run self test on MPU6886 """
    
    from machine import SoftI2C, Pin

    i2c = SoftI2C(scl=Pin(22), sda=Pin(21))

    print("Initializing  IMU ..")
    imu = MPU6886(i2c, debug=True)
    print("imuparms ..")
    [print('  ', k, '->', v) for k, v in imu.imuparms.items()]
    print(" running self test ..")
    imu.selftest(sensor='acceleration', axis='z')
    imu.temperature
