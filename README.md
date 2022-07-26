# MPU6886_Driver_M5Stack_Core2
6-axis Inertial Measuring Unit MPU6886 Driver in MicroPython for M5Stack Core2 hardware

## Introduction
This repository contains micropython driver (mpu6886.py) tested on M5Stack Core2 hardware

## Installation
The focaltouch.py driver may be installed in the M5Core2 flash memory under /lib folder without the need reflash.

If there is name conflict with existing driver in the firmware, you may rename this driver unless reflashed.

## Features
The driver supports following methods and properties

### Getter/Setter imuparms
imuparms is a dictionary of contants or parameters that stores program constants and default/user overidden parameters for MPU6886 class.

All these values can changed by the user.

### Method MPU6886.reg()
This method writes nbytes (usually 1, 2 or 6) of data to regsiter location passed to it and returns a value after reading back the register.

If value is None, it performs read only.

The return value for nbytes=6 is a tuple of 3, 16bit numbers.

For nbytes=2 it is a tuple of one 16-bit number.

For nbytes=1, it returns a single number.

The default nbytes=1.

### Property MPU6886.temperature
Returns IMU instantaneous raw temperature in deg F as a type float

### Property MPU6886.temperature
Return die temperature in deg F

### Property MPU6886.acceleration
Returns a tuple of linear acceleration type float in x, y and z axis in m/sec/sec.

When the device is stationary, the sensor output will have residual values close to zeros in x and y direction, and 9.8 m/sec/sec.

### Property MPU6886.gyro
Returns a tuple of angular velocity type float in x, y and z axis in DPS (degrees/sec)
When the device is stationary, the sensor output will have residual values close to zeros in all three directions.

### Method MPU6886.avg()
Returns averaged readings of the specified sensor='acceleration or 'gyro' for specified count of scans with specified number of msec delay between each successive reads.

### Method MPU6886.avg_tolerance()
Returns a two 3-element (x, y & z) tuple. The first value being 'avg' reading and the second one being 'tolerance'.

The method performs two sets of scans (many readings) delayed by default pause=1000 msec.

Each scan involves a default count=10 readings, sucecssive reads delayed by default delay=10 msec.
The two averages are averaged again as final avg in first return value.

The 'tolerance' i.e., the second return value is average difference between corresponding elements of the tuple (e.g. x val) obtained from the two sets of averages.
In ideal situation the tolerance will be (0, 0, 0). This computed measurement is used to judge if there is a significant change in two sucecssive readings. You may call 'tolerance' as insigificant change that can be ignored as statistical error, drift or change.

### Method MPU6886.selftest()
The IMU allows running a self test which simulates synthetic linear and angular motion during which measurements are made. The method allows you to choose sensor type, axis of interest and other params described in avg_tolerance() method.

Returns self test response as avg difference in sensor readings with self test enabled and disabled. This should be within tolerance -- hopefully!

The purpose of this test is to ensure the sensor is not damaged over its life span.


