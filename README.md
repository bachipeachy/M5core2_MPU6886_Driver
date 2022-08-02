# MPU6886_Driver_M5Stack_Core2
6-axis Inertial Measuring Unit MPU6886 Driver in MicroPython for M5Stack Core2 hardware

## Introduction
This repository contains micropython driver (mpu6886.py) tested on M5Stack Core2 hardware

mpu6886_test.py script exercises all the methods

## Installation
The driver may be installed in the M5Core2 flash memory under /lib folder without the need to reflash.

If there is a name conflict with the existing driver in the firmware, you may rename this driver.

## Features
The driver supports following methods and properties.

### Getter/Setter imuparms
imuparms is a dictionary of contants or parameters that stores program constants and default/user overidden parameters for MPU6886 class.

All these values can changed by the user.

### Method reg()
This method writes nbytes (usually 1, 2 or 6) of data to regsiter location passed to it and returns a value after reading back the register.

If value is None, it performs read only.

The return value for nbytes=6 is a tuple of 3, 16bit numbers with a max value of 32768 (typ)

For nbytes=2 it is a tuple of one 16-bit number.

For nbytes=1, it returns a binary number

The default is nbytes=1.

### Property temperature
Returns IMU instantaneous raw temperature in deg F as a type float

### Property acceleration
Returns a tuple of linear acceleration type float for x, y and z axes in m/sec/sec.

When the device is stationary, the sensor output will have residual values close to zeros in x and y direction, and 9.8 m/sec/sec in z direction.

### Property gyro
Returns a tuple of angular velocity type float for x, y and z axes in DPS (deg/sec)
When the device is stationary, the sensor output will have residual values close to zeros in all three directions.

### Method _ft()
Returns the results of factory run self test permanently stored in the registers. This reference data is compared against on-demand user perfomed self test to ensure integrity of the sensors.

### Method _st()
Returns self test response difference in readings with self test enabled and disabled

### Method selftest_experimental()
The IMU self test simulates synthetic linear and angular motion during which measurements are made. This is experimental, under testing for proper interpretation of user guide info.
The purpose of this test is to ensure the sensor is healthy.

#### Note
The gyro selftest yields correct results. However, in the accel selftest, the self test respose value is exhibiting a fairly fixed bias of roughly 450 mG in all three axes. Somemore testing is needed to validate the results. This is still in work.
The article (link below) for a comparable hardware has nice explanation on self test of Inertial Measuring Units.

https://www.edn.com/the-embedded-self-test-feature-in-mems-inertial-sensors/


