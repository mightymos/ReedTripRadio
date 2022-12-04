Alternative firmware for wireless 433MHz magnetic door/window reed sensors.

STC15W104 are 8051 based processors + SYN115 radio transmitter.
'104 model has 4KB flash space and board also has a tamper detect switch which is cool.

Some sensors have STC15W101 but with only 1KB flash. Also some sensors purchased did not have tamper switch.
However 1KB flash would not support hardware abstraction layer (unless emulated EEPROM area can be used for code space).

STC processors do not allow read/verify of written firmware so open source alternative is needed to confirm program behavior.

Boards contain a header that may be populated with pins labeled with G (ground), T (transmit), and R (receive) for flashing.

For receiving radio messages recommend:
Sonoff Bridge 433 MHz
https://tasmota.github.io/docs/devices/Sonoff-RF-Bridge-433/

Another alternative firmware for the Sonoff Bridge is:
https://github.com/xoseperez/espurna

ESPurna is nice because it treats wireless sensors as "virtual" sensors (show up as permanent switch entities in Home Assistant).
Also ESPurna can learn unique sensor codes.

Some Sonoff Bridge contain an onboard EFM8BB1 which can additionally be flashed to support more radio protocols:
https://github.com/Portisch/RF-Bridge-EFM8BB1

Flashing tool:
https://github.com/area-8051/stcgal-patched

Firmware uses hardware abstraction layer (HAL):
https://github.com/area-8051/uni-STC

Proposed features (some added as compared with original firmware):
[01] Transmit on reed switch open/close (interrupt)                                 (DONE)
[02] Transmit on tamper switch open/close (interrupt)                               (DONE)
[03] Manage power modes                                                             (DONE)
[04] Support inverted protocols                                                     (DONE)
[05] Ability to select any supported rc-switch transmission protocol                (DONE)
[06] "Heart beat" mode for periodic transmission                                    (DONE)
[07] Adjustable LED blink behavior                                                  (TODO)
[08] Adjustable sleep behavior                                                      (DONE)
[09] User configuration/input with tamper switch press(es) or serial bytes          (DONE)
[10] Add tamper closed key                                                          (DONE)
[11] Add tamper "trip" mode                                                         (TODO)
[11] Store settings in EEPROM                                                       (DONE)
[12] Send information over radio (e.g., settings?, battery?)                        (TODO)
[13] Compare power usage to original firmware                                       (TODO)
[14] Test transmission protocols 2-12                                               (TODO)


![Hookup Example](/photos/hookup_example.jpg?raw=true "Wireless 433 MHz Door Sensor)