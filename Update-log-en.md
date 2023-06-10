[![zh](https://img.shields.io/badge/lang-zh-blue.svg)](Update-log.md)

# Update Log

[Firmware Update Instructions](README-en.md)

**Note**: Newest entries are on top.

## Firmware Version Compatibility

| Screen Version | Motherboard Version | Description         |  Update time  |
| ---------------|---------------------|---------------------|-----------|
| V1.4.2          | 1.x.5.1b            | Release version     | 2023-05-20 |
| 1.5-Beta       | 1.x.5.2 - 1.x.5.3a  | Beta                |           |
| 1.4 - V1.4.1   | 1.x.4 - 1.x.5.1a    | Historical Versions |           |

---

## Screen Firmware Changelog


### UI version: V1.4.2 - 20230520   

1. Optimized from V1.4.1.   
2. Compatible with motherboard firmware prior to 1.1.5.1b.    
3. Support multi-file display, in [Settings] - [Advanced Settings] can enable multi-file display. (Note: this feature is only compatible with motherboard firmware 1.x.5.1b)    
4. Supports folder printing. (Note: this feature is only compatible with motherboard firmware 1.x.5.1b)   
5. The fan speed in print is shown as: (0-100) %; The fan speed is set to 0-255.    
6. Added a prompt for M600 to trigger refuelling.   
7. Fixed the issue of no backlight on the screen when the minimum brightness of the screen was 10%.   




### UI version: 1.5-Beta  

1. New UI version, there is a compatibility relationship with motherboard firmware versions below 1.x.5.2 (Z direction is displayed as 0), but it does not affect the actual printing.
2. Add the first page to display XYZ coordinate system.
3. Z coordinate display accuracy to percentile.
4. This version is a beta version, continuous improvement will be made to add more functions.

### UI version: 1.4.1  

1. Optimize on the basis of version 1.4.2.
2. Compatible with motherboard firmware before firmware 1.1.5.1a.
3. Solve the problem of synchronizing the 'break material detection switch' with the 'ready screen move button' switch.

### UI version: 1.4  

1. Fix the problem that the display is dead and stuck.
2. Add blue screen prompt after detecting screen error. 3.
3. Add screen standby function: click [Settings] [Advanced Settings] in the main interface, then click "small sun" next to the brightness bar to switch to "moon" to enable screen standby, and the brightness will be reduced after 3 minutes.
4. Fix some known bugs.

---

## Motherboard Firmware Update Log  

### 1.x.5.1b - 20230520  

1. Improve on 1.x.5.1a, compatible with UI version V1.4.2 (not compatible with UI version 1.5-Beta).
2. Fix a broken material alarm that was incorrectly triggered once when reopening the broken material detection after closing it.
3. Support multi-file display, you can enable multi-file display in [Settings] - [Advanced Settings]. (Note: this function is only compatible with UI version V1.4.2)
4. Support folder printing. (Note: This feature is only compatible with UI version V1.4.2)
5. The fan speed displayed in printing is: (0-100)%; the fan speed setting is: 0-255.
6. Fix the fan speed displayed in the printing interface always shows 255, as shown in 5.
7. Repair the problem of some special symbols causing failure to start normally.
8. Fix the display of wrong name when power failure is renewed.
9. Fix some other known bugs.

### 1.x.5.1a

1. Make firmware compatible with screen firmware version 1.4 and V1.4.1 on the basis of 1.x.5.3 (not compatible with screen firmware version 1.5-Beta).
2. fix the error that the print flow can only be above 100%, and limit it to 50%-150%.
3. Repair the problem of setting the speed of incoming and outgoing materials in the preparation interface.
4. Repair the bug of material multi-extrusion when resuming printing after pause.
5. Optimize the moving steps when pausing and resuming printing.
6. Optimize the power-off renewal function.
7. Solve the situation that the screen can't be paused after re-powering after power off during printing.
8. Add:M355 command to control LED. <https://marlinfw.org/docs/gcode/M355.html>
9. Add:M300 command to control buzzer (buzzer frequency is fixed). <https://marlinfw.org/docs/gcode/M300.html>

### 1.x.5.3

1. Improve on the basis of 1.x.5.2. 2.
2. Optimize read/write cards.

### 1.x.5.2

1. Improve on the basis of 1.x.5.1. 2.
2. Fix the bug:Platform moves backward after removing TF card.
3. Fix the bug: the screen keeps showing "Update firmware..." when booting up the machine. interface. 4.
4. Fix the bug: The files in the file selection list are lost after the screen is powered off and powered on again.

### 1.x.5.1

1. Improve on the basis of 1.x.5.
2. Enable: Linear Pressure Control V1.5; default K0.0.

### 1.x.5

1. Optimize on the basis of 1.x.4.1.
2. Enable hot-side PID auto-tuning.
3. Support M600 code pause.
4. Optimize stop printing position.
5. Fix some known bugs.

### 1.x.4.1

1. Optimize on the basis of 1.x.4.
2. Turn off the hot bed PID.
3. Solve the light flashing situation of 1.1.4 firmware.
4. Fix some known bugs.

### 1.x.4

1. Add SD card read/write error warning. 2.
2. Default max acceleration: X1100 Y900 Z100 E1500. 3.
3. Enable: Host Action Commands.
4. Enable: Hot Bed PID,Default:M304 P97.10 I1.41 D1675.16.
5. Fix hot end thermal disconnect report error display error content.

---
