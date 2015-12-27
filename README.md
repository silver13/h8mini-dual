# Eachine H8 mini level firmware

This is a dual mode version of the firmware for Eachine H8 mini / JJRC H8 mini.

The firmware uses cascaded pids in level mode.


For accelerometer calibration move the pitch stick down 3 times within about 1- 2 seconds. Wait a couple of seconds between attempts. Throttle has to be low, and roll centered. Flashing lights indicate the calibration process. This is saved so it has to be done only once.

To set / reset AUX1 channel use left - left - down (off) or right - right - down (on). The leds flash if a gesture is detected.

Do not switch level mode on while upside down.

#####Stock tx:
On the stock tx only the rate button works, so you should set level mode to CH_AUX1 and use the gestures for on/off
To start in acro mode by default with CH_AUX1 enabled comment out "#define AUX1_START_ON"
Trims are currently not functional on the stock tx.

#####Devo tx:
Channels work as intended except the expert channel which is always on. Dynamic trims are not used. If gestures are to be used make sure the tx can reach at least 80% of max rates ( high rates on).

### 28.12.15
* a battery low warning fix
* some level mode settings changed ( acc filter time, acc lpf )

### Update 23.12.15
* added gestures
* fix battery low hysteresys bug

