# Eachine H8 mini level firmware

This is a dual mode version of the firmware for Eachine H8 mini / JJRC H8 mini.

**Do not flash the H8 firmware to the H101**

**Do not flash the H8 firmware to the H8S**

The firmware uses cascaded pids in level mode.

For accelerometer calibration move the pitch stick down 3 times within about 1- 2 seconds. Wait a couple of seconds after a failed attempt. Throttle has to be low, and roll centered. Flashing lights indicate the calibration process. This is saved so it has to be done only once.

To set / reset AUX1 channel use left - left - down (off) or right - right - down (on). The leds flash if a gesture is detected.

Do not switch into level mode while upside down.

#####Stock tx:
On the stock tx only the rate (expert) button works, and level/ acro mode is changed by gestures (default) as described above.
To start in acro mode by default with CH_AUX1 enabled comment out "#define AUX1_START_ON" in config.h.

Trims are currently not functional on the stock tx.

#####Devo tx:
Channels work as intended except the rate/expert channel which is always on. Dynamic trims are not used. If gestures are to be used make sure the tx can reach at least 80% of max rates ( high rates on).

Assign the extra channels to desired functions in config.h, mainly the level / acro mode change function should be set to a switch rather than gestures which is default.

#####Gyro calibration
Gyro calibration runs automatically after power up, and usually completes within 2-4 seconds. If for some reason the calibration fails to complete, such as if there is movement, it will eventually time out in 15 seconds.

During calibration the leds flash in an "X" like pattern. If movement is detected the flashing stops and only 2 leds illuminate. The flashing resumes when movement stops.

#####Accelerometer calibration
For accelerometer calibration move the pitch stick down 3 times within about 1- 2 seconds. Wait a couple of seconds after a failed attempt. Throttle has to be low, and roll centered. Flashing lights indicate the calibration process. This is saved so it has to be done only once.

Note the acc calibration also saves gyro biases which are used in some cases. The flash pattern is similar to the gyro calibration pattern.

#####Led error codes
In some cases the leds are used to indicate error conditions, and as such they flash a number of times, then a brake occurs, then the pattern repeats. In all such cases the quadcopter will not respond to commands, a power cycle will be required.

The most common of this is 2 flashes = low battery, usually caused by an in-flight reset due to low battery. All other flashes are non user serviceable. The description is in main.c.

#####Led flash patterns
At startup the leds should flash a gyro calibration pattern for 2 - 15 seconds, in a cross like pattern. Movement stops the flashing while it occurs.

Following should be a fast (20 times/sec) flash indicating that the quad is waiting for bind. 

If binding is completed the leds should light up continuously, while if tx connection is lost they will flash a short time a couple of times / second.

Overriding all this patterns except gyro calibration, is the low battery flash which is a slow, equally spaced on and off flash. 


### 23.1.16
* bias fix for larger gyro biases that can occur sometimes 

### 13.1.16
* clock setting fix
* pid's are now compatible with the acro only version 
* only rate d term is affected, new kd = old kd * 2.5
* 8Khz pwm used instead of 490Hz
* 1ms loop time, fixed
* level mode switch set to gestures by default now

### 28.12.15
* a battery low warning fix
* some level mode settings changed ( acc filter time, acc lpf )

### 23.12.15
* added gestures
* fix battery low hysteresys bug

