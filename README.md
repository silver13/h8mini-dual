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
To start in acro mode by default with CH_AUX1 enabled comment out *#define AUX1_START_ON* in config.h.

Trim buttons can be used as extra channels, with trimup = on, trimdown = off, if set appropriately in config.h. *#define USE_STOCK_TX* should be uncommented also, to enable trims. Please note, enabling trims is not compatible with devo tx.

Some stock tx have bad centering, and can be 5% or more offset in one direction. They also have deadzone ( it does not remove the bad centering). For this , *STOCK_TX_AUTOCENTER* should be enabled, and will remove the center biases when the quad throttle is zero for more then 1 second. (after the lights go solid)

Quick settings: 
* enable *STOCK_TX_AUTOCENTER*
* enable *USE_STOCK_TX* if use of trim buttons is required
* enable expo if you wish by commenting out *DISABLE_EXPO*



#####Devo tx:
Channels work as intended except the rate/expert channel which is always on. Dynamic trims are not used. If gestures are to be used make sure the tx can reach at least 80% of max rates ( high rates on).

In config.h channel names DEVO_CHAN_5 - DEVO_CHAN_10 can also be used , and correspond to internal devo numbers.

Assign the extra channels to desired functions in config.h, mainly the level / acro mode change function should be set to a switch rather than gestures which is default.

Please note, using trims as a channel is only intended for the stock tx, and will produce erroneous results if used with Devo tx. For this reason, *#define USE_STOCK_TX* is commented out, and disables incompatible options.
 
Devo tx support is ok by default. 

#####Gyro calibration
Gyro calibration runs automatically after power up, and usually completes within 2-4 seconds. If for some reason the calibration fails to complete, such as if there is movement, it will eventually time out in 15 seconds.

During calibration the leds flash in an "X" like pattern. If movement is detected the flashing stops and only 2 leds illuminate. The flashing resumes when movement stops.

#####Accelerometer calibration
For accelerometer calibration move the pitch stick down 3 times within about 1- 2 seconds. Wait a couple of seconds after a failed attempt. Throttle has to be low, and roll centered. Flashing lights indicate the calibration process. This is saved so it has to be done only once.

Note, the acc calibration also saves gyro biases which are used in some cases. The flash pattern is similar to the gyro calibration pattern.

*Calibration has to be done on a horizontal surface*

The calibration does not get erased between firmware flashes unless "erase" is used in the relevant menu.

#####Led error codes
In some cases the leds are used to indicate error conditions, and as such they flash a number of times, then a brake occurs, then the pattern repeats. In all such cases the quadcopter will not respond to commands, a power cycle will be required.

The most common of this is 2 flashes = low battery, usually caused by an in-flight reset due to low battery. All other flashes are non user serviceable. The description is in main.c.

#####Led flash patterns
At startup the leds should flash a gyro calibration pattern for 2 - 15 seconds, in a cross like pattern. Movement stops the flashing while it occurs.

Following should be a fast (20 times/sec) flash indicating that the quad is waiting for bind. 

If binding is completed the leds should light up continuously, while if tx connection is lost they will flash a short time a couple of times / second.

Overriding all this patterns except gyro calibration, is the low battery flash which is a slow, equally spaced on and off flash. 


###Linux support
See post by :
http://www.rcgroups.com/forums/showpost.php?p=34293596&postcount=1248

Read [INSTALL.md](INSTALL.md) for more information.


###Wiki
http://sirdomsen.diskstation.me/dokuwiki/doku.php?id=start

### .10.16
* level mode drift bug fix
* buzzer functionality added, on programming pins
* board rotations in file sixaxis.c

###16.09.16
* High angle in level mode - up to 90 
* Yaw fix - level mode

###21.07.16
* updates from h101 fork
* added extra devo channels
* "prevent_reset" option which lowers throttle to kkep voltage up
* "motor_beeps"
* other changes to make fork differences smaller

###1.05.16
* ENABLESTIX feature now works correctly, wait 1 second on ground for it to deactivate
* gyro pll set to a different setting
* 8.5 mm motor curve redone ( old curve renamed _OLD)
* other changes


###16.04.16
* added auto flips
* headless mode now works in both level and acro mode.
* fixed an issue with powering up on a non horizontal surface in level mode (drift)
* manual trim values can now be entered in config.h for level mode
* fixed an bug in "auto lower throttle" which would have caused a corner dip

###19.02.16
* added "STOCK_TX_AUTOCENTER", recommended to enable for stock tx
* added use of trim buttons as extra channels ( by balrog-kun )

###09.02.16
* fixed "gyro not found" in debug mode / after flashing (hopefully)
* selectable motor curves, also with 8.5mm hubsan motors support
* pwm frequency in config.h (now set to 16k)
* "invert yaw option" for hubsan motors/other builds that have spinning yaw

###05.02.16
* added linux compilation support by balrog-kun
* (option) added throttle transient compensation 
* (option) added "anti-clipping" with feedforward 
* new options disabled by default

### 25.1.16
* (option) selectable software gyro filter
* (option) auto throttle lowering to prevent control issues when near full throttle
* (option) throttle angle compensation for level mode
* new options all disabled by default
* rearranged config.h

### 23.1.16
* bias fix for larger gyro biases that can occur sometimes 

### 13.1.16
* clock setting fix
* pid's are now compatible with the acro only version 
* only rate d term is affected, new kd = old kd * 2.5
* 8Khz pwm used instead of 490Hz
* 1ms loop time, fixed
* level mode switch set to gestures by default

### 28.12.15
* a battery low warning fix
* some level mode settings changed ( acc filter time, acc lpf )

### 23.12.15
* added gestures
* fix battery low hysteresys bug

