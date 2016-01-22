# Eachine H8 mini level firmware

This is a dual mode version of the firmware for Eachine H8 mini / JJRC H8 mini.

**Do not flash the H8 firmware to the H101**

**Do not flash the H8 firmware to the H8S**

The firmware uses cascaded pids in level mode.


For accelerometer calibration move the pitch stick down 3 times within about 1- 2 seconds. Wait a couple of seconds after a failed attempt. Throttle has to be low, and roll centered. Flashing lights indicate the calibration process. This is saved so it has to be done only once.

To set / reset AUX1 channel use left - left - down (off) or right - right - down (on). The leds flash if a gesture is detected.

Do not switch into level mode while upside down.

#####Stock tx:
On the stock tx only the rate (expert) button works, and level/ acro mode is changed as described above.
To start in acro mode by default with CH_AUX1 enabled comment out "#define AUX1_START_ON" in config.h.

Trims are currently not functional on the stock tx.

#####Devo tx:
Channels work as intended except the rate/expert channel which is always on. Dynamic trims are not used. If gestures are to be used make sure the tx can reach at least 80% of max rates ( high rates on).

Assign the extra channels to desired functions in config.h, mainly the level / acro mode change function should be set to a switch rather than gestures which is default.

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

