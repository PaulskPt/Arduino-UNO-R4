# Arduino-UNO-R4
Exploring UNO R4 Minima &amp; WiFi features.

On the UNO R4, Serial is the USB port, Serial1 is on pins 0 & 1.

Put
```
#define NO_USB
#include <Arduino.h>
```
at the top of the sketch to route Serial to pins 0 and 1.

On the WiFi Serial2 is connected to the Wi-Fi module.

Fork additions by @PaulskPt:
- Added folder R4WiFi_led_matrix_with_accents;
- Added in this folder the sketch: R4WiFi_led_matrix_with_accents.ino.
This is just a try. To use accents I think it should be better to 
define characters 5-column wide (especially for use of the accent-circumflex).
I am thinking about definining a character set with 5-column characters.

- Added folder R4WiFi_RTC_NTPsync_to_led_matrix;
- In this folder added:
  - the sketch: R4WiFi_RTC_NTPsync_to_led_matrix.ino;
  - the file adafruit_secrets.h.

You have to fill-in your WiFi credentials in this file.


This example ia a try to display the date and time synchronized from an NTP server
onto the led matrix. I made some changes to the definition of the led matrix
characters '-' and ':'.


