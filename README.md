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


- Added folder R4WiFi_AHT20_to_led_matrix.
- In this folder added:
  - the sketch: R4WiFi_AHT20_to_led_matrix.ino.

This example reads the temperature and humidity values from an Adafruit AHT20 sensor
and displays the values to the led matrix.

# Updates

2025-05-10 Added folder R4WiFi_RTC_NTP_joy-it_LCD1.28R
 In this folder added:
  - the sketch: SBC_LCD1_28R_Arduino_NTP_Clock_Example.ino.
  - the file adafruit_secrets.h.

This example, like the example ```R4WiFi_RTC_NTPsync_to_led_matrix``` gets a NTP datetime stamp,
instead of printing the datetime to the led matrix of the Uno R4, the datetime is used to drive
an analog clock onto the Joy-it round LCD of 1.28inches.
In file ```adafruit_secrets.h``` you have to enter your ```WiFi SSID```, ```PASSWORD``` and ```TIMEZONE_OFFSET```. The latter represents the value in hours, for example Europe/Lisbon will have a TIMEZONE_OFFSET = "1" (1 hour).
- Added also a folder ```images``` with some images of this project version.
- Added a folder ```docs```. In it a monitor output text files for this project version.

