/*
 * Arduino_UNO_WiFi_R4__RTC_NTP_V5.ino (See forked repo: Arduino-UNO-R4, example: R4WiFi_RTC_NTP_V5)
 * This example shows how to set the RTC (Real Time Clock) on the Portenta C33 
 * to the current date and time retrieved from an NTP server on the Internet (pool.ntp.org).
 * Then the current time from the RTC is printed to the Serial port.
 * 
 * Instructions:
 * 1. Change the WiFi credentials in the arduino_secrets.h file to match your WiFi network.
 * 2. Upload this sketch to Portenta C33.
 * 3. Open the Serial Monitor.
 * 
 * Initial author: Sebastian Romero @sebromero
 * 
 * Modifications by @PaulskPt (Github)
 * Mods to display the received NTP datetime stamp do the 8x12 led matrix
 * The interval for NTP sync is defined by I_NTP_SYNC.
 * The RTC datetime readout interval is defined by I_TM
 * It takes about 19 seconds for the RTC datetime string to be displayed
 *
 * In V3 added save currentTime to and read it from EEPROM
 *
 * Update 2025-10-17:
 * I added a serialPrintf() function created by Microsoft Copilot.
 * See https://forum.arduino.cc/t/uno-r4-wifi-adding-eeprom-procedure-gives-flash-overrun/1396203/5,
 * response from user @david_2018:
 * The UNO R4 WiFi does not have actual EEPROM, it is emulated in flash memory, and is 8K bytes in size.
 * Added functionality to read data from a Qwiic connected sensor Pimoroni Breakout Garden BME280 and display the data on the matrix.
 * Update 2025-10-24:
 * Added board unique ID as BANNER4 (using the 107-Arduino-UniqueId library repo)
 * https://github.com/107-systems/107-Arduino-UniqueId
 * Update 2025-10-27. Added dst awarenesss for years 2025-2028 (for Portugal timezone). See DstPeriod map.
 */

#include <RTC.h>
//#include "C:\Users\<User>\AppData\Local\Arduino15\packages\arduino\hardware\renesas_uno\1.5.1\libraries\RTC\src\RTC.h"
#include <Arduino.h>
#include <107-Arduino-UniqueId.h>
#include <Time.h>
#include <TimeLib.h>

#include <NTPClient.h>
#include <string.h>
#include <stdio.h>  // for snprintf() and ...

// Added in version v4
// include <stdio.h>   See: https://forum.arduino.cc/t/printf-on-arduino/888528
// Write to & read from EEPROM
#include <EEPROM.h>
//#include <Utils.h>  // Note @PaulskPt 2025-10-17: Utils.h not found. I don't know where I got it from
//                       Utils.h seems not to be needed for this sketch with the Uno R4 WiFi

//#if defined(ARDUINO_PORTENTA_C33)
//#include <WiFiC3.h>
//#elif defined(ARDUINO_UNOWIFIR4)
//#include <WiFiS3.h>
//#endif

#if defined(ARDUINO_UNOWIFIR4)
#include <WiFiS3.h>
#endif

#include <WiFiUdp.h>
#include "arduino_secrets.h" 

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <map>
#include <string>
#include <ctime>


bool lStart = true;

const char fwVersion[] = "v4.3.1"; // 

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;

/* Note @PaulskPt: using a Pimoroni Breakout Garden BME280 sensor with default I2C address 0x76 */
uint8_t sensor_address = 0x76;

#ifndef MY_DEBUG
#define MY_DEBUG
#endif
// #define MY_DEBUG  (1)

#define USE_OLD_LOOP

#ifdef USE_EEPROM
#undef USE_EEPROM
#endif

// Define print macros
#ifdef MY_DEBUG
  #define DEBUG_PRINT(x)       Serial.print(x)
  #define DEBUG_PRINTLN(x)     Serial.println(x)
  #define DEBUG_PRINTF(...)    serialPrintf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(...)
#endif

#define BANNER_LEN 52
#define BANNER0 0
#define BANNER1 1
#define BANNER2 2
#define BANNER3 3
#define BANNER4 4

bool matrix_active = true; // Global flag to indicate if matrix updates are active
bool bme_failed_recently = false;

float t, p, a, h; // temperature, pressure, altitude, humidity

// Leading spaces ensure starting at the right side of the matrix
char ntp_sync_txt[BANNER_LEN]; // set in setup()
char rtc_dt_txt[BANNER_LEN];   // same
char bme_data_txt[BANNER_LEN]; // same
char id_str[33];  // 16 bytes * 2 chars + 1 null terminator
char banner_txt[BANNER_LEN]    = "";
size_t init_banner_txt_len; // set in setup()
uint8_t tot_width = 0;
uint8_t txt_idx = BANNER0; // (idx will be increased in loop(), thus: Start with showing BME280 data
size_t tot_char_width = 0; // total width of characters in banner_txt
uint32_t t_prev = 0;
//float   avg_char_width = 3.44f; // average character width

const uint16_t scroll_speed = 100; // milliseconds
const uint16_t ontime = 50; // was: 521; // microseconds. 521 (us) * 96 (pixels) = 50 ms frame rate if all the pixels are on.
static uint8_t scroll = 0; // scroll position.

// The following average column widths were calculated from actual measurements
uint8_t stby_avg_cols = 58; //           15 * 5
uint8_t ntp_avg_cols = 160; // was 175
uint8_t rtc_avg_cols = 156; // was 175   35 * 5 
uint8_t bme_avg_cols = 180; // was 220   43 * 5 = 215
uint8_t id_avg_cols  = 110; // was 125   25 * 5
uint8_t avg_cols = 0;

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

int tzOffset;
int tzDST_Offset = atoi(SECRET_TIMEZONE_DST_OFFSET); // can be negative or positive (hours)
int tzSTD_Offset = atoi(SECRET_TIMEZONE_STD_OFFSET); // can be negative or positive (hours)
const char SECRET_TIMEZONE_DST_ID[] = SECRET_TZ_DST_ID;
const char SECRET_TIMEZONE_STD_ID[] = SECRET_TZ_STD_ID;
signed long utc_offset; //  = tzOffset * 3600;    // utc_offset in seconds. Attention: signed long! Can be negative or positive

// Retrieve the date and time from the RTC and print them
RTCTime currentTime;

constexpr unsigned int LOCAL_PORT = 2390;      // local port to listen for UDP packets
//constexpr int NTP_PACKET_SIZE = 48; // NTP timestamp is in the first 48 bytes of the message

// Used by isDST()
struct DstPeriod {
  time_t start;
  time_t end;
};

// Used by isDST()
#ifdef REGION_EUROPE
std::map<std::string, DstPeriod> dst_start_end = {
  { "2025", {1743296400, 1761444000 } },
  { "2026", {1774746000, 1792893600 } },
  { "2027", {1806195600, 1824948000 } },
  { "2028", {1837645200, 1856397600 } }
};
#endif

int wifiStatus = WL_IDLE_STATUS;

WiFiUDP Udp; // A UDP instance to let us send and receive packets over UDP
NTPClient timeClient(Udp);

void initBannerText(char* buf, const char* literal) {
  memset(buf, 0, BANNER_LEN);
  strncpy(buf, literal, BANNER_LEN - 1);
}

int serialPrintf(const char *format, ...) {
  char buffer[128];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  return Serial.print(buffer);
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void connectToWiFi() {
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION)
    Serial.println("Please upgrade the firmware");
  
  // attempt to connect to WiFi network:
  while (wifiStatus != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    wifiStatus = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  Serial.println("Connected to WiFi");
  printWifiStatus();
}

void unblockI2CBus(uint8_t sclPin) {
  pinMode(sclPin, OUTPUT);
  DEBUG_PRINTLN(F("Unblocking I2C1 bus..."));
  for (int i = 0; i < 9; i++) {
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sclPin, LOW);
    delayMicroseconds(10);
  }
  DEBUG_PRINTLN(F("I2C1 bus unblocked"));
}

bool resetBME280() {
  uint8_t sclPin = 26; // (Qwiic which uses I2C1)
  Wire1.end();
  delay(100);
  unblockI2CBus(sclPin);
  Wire1.begin();
  Wire1.setClock(100000);
  delay(100);
  unsigned status = bme.begin(sensor_address, &Wire1);
  delay(1000); // Give the sensor time to settle !
  return status;
}

void sensor_type(unsigned status) {
  if (!status) {
    if (!resetBME280()) {
      Serial.println(F("Could not find a valid BME280 sensor, check wiring, address, sensor ID!"));
      Serial.print(F("SensorID was: 0x")); Serial.println(bme.sensorID(),16);
      Serial.print(F("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n"));
      Serial.print(F("   ID of 0x56-0x58 represents a BMP 280,\n"));
      Serial.print(F("        ID of 0x60 represents a BME 280.\n"));
      Serial.print(F("        ID of 0x61 represents a BME 680.\n"));
      Serial.println(F("BME280 reset failed."));
      while (1) delay(10);
    } else {
      Serial.println(F("BME280 reset succeeded."));
    }
  }
  else {
    const char *txts[] PROGMEM = {"BME280", "BME680", "bad addres, a BMP180 or BMP085"};
    Serial.print(F("SensorID is: 0x"));
    uint8_t sID = bme.sensorID();
    Serial.print(sID,16);
    Serial.print(", type is: ");
    if (sID >= 0x56 && sID <= 0x58)
      Serial.println(txts[0]);
    else if (sID == 0x60)
      Serial.println(txts[0]);
    else if (sID == 0x61)
      Serial.println(txts[1]);
    else if (sID == 0xFF)
      Serial.println(txts[2]);
    else
      Serial.println("Unknown");
    delay(3000);
  }
}

bool safe_take_forced_measurement(uint8_t retries = 3, uint16_t timeout_ms = 1000) {
  for (uint8_t attempt = 0; attempt < retries; attempt++) {
    unsigned long start = millis();
    while (!bme.takeForcedMeasurement()) {
      if (millis() - start > timeout_ms) {
        Serial.print(F("Attempt "));
        Serial.print(attempt + 1);
        Serial.println(F(": takeForcedMeasurement() timed out."));
        break;
      }
      delay(10);
    }
    if (bme.readTemperature() != NAN)
      return true; // Success
    
    delay(50); // Let the sensor settle before retry
  }
  return false;
}

/* Retrieve BME280 sensor data: temperature, pressure, humidity, altitude */
void get_bme_data() {
  const char txt0[] = "get_bme_data(): "; // No PROGMEM !

  if (bme_failed_recently) {
    Serial.print(txt0);
    Serial.println(F("Skipping BME read due to recent failure."));
    return;
  }

  DEBUG_PRINT(txt0);
  DEBUG_PRINTLN(F("Reading BME280 data..."));

  delay(500); // Let sensor stabilize

  //DEBUG_PRINTLN(F("Reading temperature..."));

#ifdef USE_FORCED_MEASUREMENT
  if (!lStart)
    reset_bme280();
#endif

  t = bme.readTemperature();

  //DEBUG_PRINTLN(F("Reading pressure..."));
  p = bme.readPressure() / 100.0F;

  //DEBUG_PRINTLN(F("Reading humidity..."));
  h = bme.readHumidity();

  //DEBUG_PRINTLN(F("Reading altitude..."));
  a = bme.readAltitude(SEALEVELPRESSURE_HPA);

  DEBUG_PRINT(txt0);
  DEBUG_PRINTLN(F("All reads done."));

  if (isnan(t) || isnan(p) || isnan(h) || isnan(a)) {
    Serial.println(F("Warning: BME280 returned NaN values. Trying to reset the sensor."));
    resetBME280();
  } else {
    //DEBUG_PRINTLN("BME280 data read successfully.");
    //DEBUG_PRINT("Temperature: "); DEBUG_PRINTLN(t);
    //DEBUG_PRINT("Pressure: "); DEBUG_PRINTLN(p);
    //DEBUG_PRINT("Humidity: "); DEBUG_PRINTLN(h);
    //DEBUG_PRINT("Altitude: "); DEBUG_PRINTLN(a);
    pr_bme_data();
  }
  delay(100);
  bme_failed_recently = false;
}

/* uint8_t bme_data_txt[BANNER_LEN]    = "BME280 data:  "; */
void bme_matrix_data() {
  /*
     txt0 was stored in flash memory (PROGMEM)
     serialPrintf() expects a RAM-resident string for %s
     Passing a PROGMEM pointer to %s causes undefined behavior — often a crash or hang
  */
  const char txt0[] = "bme_matrix_data(): ";  // No PROGMEM !!! See explanation above.

  //DEBUG_PRINT(txt0);
  //DEBUG_PRINT("init_banner_txt_len = ");
  //DEBUG_PRINTLN(init_banner_txt_len);

  DEBUG_PRINT(txt0);
  DEBUG_PRINTLN("BME280 data...");
  // Clear previous content after initial text
  memset(banner_txt, ' ', BANNER_LEN - 1);  // Fill with spaces
  // Ensure banner_txt is null-terminated before any strlen() or snprintf():
  banner_txt[BANNER_LEN - 1] = '\0';
  //DEBUG_PRINT(txt0);
  //DEBUG_PRINT("memset(banner_txt...) = \"");
  //DEBUG_PRINT(banner_txt);
  //DEBUG_PRINTLN("\"");

  char bme_data_txt2[64];

  int written = snprintf(bme_data_txt2, sizeof(bme_data_txt2),
    "  BME280 data: T:%4.1f°C P:%6.1fmB H:%4.1f%%", t, p, h);  // 48 characters (?)
  
  
  DEBUG_PRINT(txt0);
  DEBUG_PRINT("written = ");
  DEBUG_PRINTLN(written);
  /*
  bme_data_txt2[written] = '\0';
  DEBUG_PRINT("bme_data_txt2 = \"");
  DEBUG_PRINT(bme_data_txt2);
  DEBUG_PRINTLN("\"");
  */
  if (written < 0 || written >= sizeof(bme_data_txt2)) {
    DEBUG_PRINT(txt0);
    DEBUG_PRINTF("snprintf failed or overflowed\n");
    return;
  }

  if (written > 0 && written < BANNER_LEN) {
    memcpy(banner_txt, bme_data_txt2, written);
    banner_txt[written] = '\0';
  } else {
    DEBUG_PRINT(txt0);
    DEBUG_PRINT(F("Warning: banner_txt overflow prevented. Written = "));
    DEBUG_PRINTLN(written);
  }
  //DEBUG_PRINT(txt0);
  //DEBUG_PRINTF("banner_txt= \"%s\"\n", banner_txt);
  //DEBUG_PRINT(F("Approx. Altitude = "));
  //DEBUG_PRINTLN(bme.readAltitude(SEALEVELPRESSURE_HPA));
}

void pr_bme_data() {
  DEBUG_PRINTF("Temperature: %5.2f °C\n", t);
  DEBUG_PRINTF("Pressure: %7.2f Mb\n", p);
  DEBUG_PRINTF("Humidity: %5.2f %%\n", h);
  DEBUG_PRINTF("Altitude: %7.2f m\n", a);
}    



bool accent_flag = false;

const uint8_t font_5x8[] = 
{
  2, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, //  0  space  32   offset 0
  1, 0b01011111, 0b00000000, 0b00000000, 0b00000000, 0b00000000, //  1  !             6
  3, 0b00000011, 0b00000000, 0b00000011, 0b00000000, 0b00000000, //  2  "            12
  5, 0b00010100, 0b00111110, 0b00010100, 0b00111110, 0b00010100, //  3  #            18
  4, 0b00100100, 0b01101010, 0b00101011, 0b00010010, 0b00000000, //  4  $            24
  5, 0b01100011, 0b00010011, 0b00001000, 0b01100100, 0b01100011, //  5  %            30
  5, 0b00110110, 0b01001001, 0b01010110, 0b00100000, 0b01010000, //  6  &            36
  1, 0b00000011, 0b00000000, 0b00000000, 0b00000000, 0b00000000, //  7  '            42
  3, 0b00011100, 0b00100010, 0b01000001, 0b00000000, 0b00000000, //  8  ( 40         48
  3, 0b01000001, 0b00100010, 0b00011100, 0b00000000, 0b00000000, //  9  )            54
  5, 0b00101000, 0b00011000, 0b00001110, 0b00011000, 0b00101000, // 10  *            60
  5, 0b00001000, 0b00001000, 0b00111110, 0b00001000, 0b00001000, // 11  +            66
  2, 0b10110000, 0b01110000, 0b00000000, 0b00000000, 0b00000000, // 12  ,            72
  2, 0b00010000, 0b00010000, 0b00000000, 0b00000000, 0b00000000, // 13  -            78   was: 0b00001000, 0b00001000,...
  2, 0b01100000, 0b01100000, 0b00000000, 0b00000000, 0b00000000, // 14  .            84
  4, 0b01100000, 0b00011000, 0b00000110, 0b00000001, 0b00000000, // 15  /            90
  4, 0b00111110, 0b01000001, 0b01000001, 0b00111110, 0b00000000, // 16  0            96
  3, 0b01000010, 0b01111111, 0b01000000, 0b00000000, 0b00000000, // 17  1           102
  4, 0b01100010, 0b01010001, 0b01001001, 0b01000110, 0b00000000, // 18  2 50        108
  4, 0b00100010, 0b01000001, 0b01001001, 0b00110110, 0b00000000, // 19  3           114
  4, 0b00011000, 0b00010100, 0b00010010, 0b01111111, 0b00000000, // 20  4           120
  4, 0b00100111, 0b01000101, 0b01000101, 0b00111001, 0b00000000, // 21  5           126
  4, 0b00111110, 0b01001001, 0b01001001, 0b00110000, 0b00000000, // 22  6           132
  4, 0b01100001, 0b00010001, 0b00001001, 0b00000111, 0b00000000, // 23  7           138
  4, 0b00110110, 0b01001001, 0b01001001, 0b00110110, 0b00000000, // 24  8           144
  4, 0b00000110, 0b01001001, 0b01001001, 0b00111110, 0b00000000, // 25  9           150
  1, 0b00101000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, // 26  :           156   was: 0b00010100, ...
  2, 0b10000000, 0b01010000, 0b00000000, 0b00000000, 0b00000000, // 27  ;           162
  3, 0b00010000, 0b00101000, 0b01000100, 0b00000000, 0b00000000, // 28  < 60        168
  3, 0b00010100, 0b00010100, 0b00010100, 0b00000000, 0b00000000, // 29  =           174
  3, 0b01000100, 0b00101000, 0b00010000, 0b00000000, 0b00000000, // 30  >           180
  4, 0b00000010, 0b01011001, 0b00001001, 0b00000110, 0b00000000, // 31  ?           186
  5, 0b00111110, 0b01001001, 0b01010101, 0b01011101, 0b00001110, // 32  @           192
  4, 0b01111110, 0b00010001, 0b00010001, 0b01111110, 0b00000000, // 33  A           198
  4, 0b01111111, 0b01001001, 0b01001001, 0b00110110, 0b00000000, // 34  B           204
  4, 0b00111110, 0b01000001, 0b01000001, 0b00100010, 0b00000000, // 35  C           210
  4, 0b01111111, 0b01000001, 0b01000001, 0b00111110, 0b00000000, // 36  D           216
  4, 0b01111111, 0b01001001, 0b01001001, 0b01000001, 0b00000000, // 37  E           222
  4, 0b01111111, 0b00001001, 0b00001001, 0b00000001, 0b00000000, // 38  F 70        228
  4, 0b00111110, 0b01000001, 0b01001001, 0b01111010, 0b00000000, // 39  G           234
  4, 0b01111111, 0b00001000, 0b00001000, 0b01111111, 0b00000000, // 40  H           240
  3, 0b01000001, 0b01111111, 0b01000001, 0b00000000, 0b00000000, // 41  I           246
  4, 0b00110000, 0b01000000, 0b01000001, 0b00111111, 0b00000000, // 42  J           252
  4, 0b01111111, 0b00001000, 0b00010100, 0b01100011, 0b00000000, // 43  K           258
  4, 0b01111111, 0b01000000, 0b01000000, 0b01000000, 0b00000000, // 44  L           264
  5, 0b01111111, 0b00000010, 0b00001100, 0b00000010, 0b01111111, // 45  M           270
  5, 0b01111111, 0b00000100, 0b00001000, 0b00010000, 0b01111111, // 46  N           276
  4, 0b00111110, 0b01000001, 0b01000001, 0b00111110, 0b00000000, // 47  O           282
  4, 0b01111111, 0b00001001, 0b00001001, 0b00000110, 0b00000000, // 48  P 80        288
  4, 0b00111110, 0b01000001, 0b01000001, 0b10111110, 0b00000000, // 49  Q           294
  4, 0b01111111, 0b00001001, 0b00001001, 0b01110110, 0b00000000, // 50  R           300
  4, 0b01000110, 0b01001001, 0b01001001, 0b00110010, 0b00000000, // 51  S           306
  5, 0b00000001, 0b00000001, 0b01111111, 0b00000001, 0b00000001, // 52  T           312
  4, 0b00111111, 0b01000000, 0b01000000, 0b00111111, 0b00000000, // 53  U           318
  5, 0b00001111, 0b00110000, 0b01000000, 0b00110000, 0b00001111, // 54  V           324
  5, 0b00111111, 0b01000000, 0b00111000, 0b01000000, 0b00111111, // 55  W           330
  5, 0b01100011, 0b00010100, 0b00001000, 0b00010100, 0b01100011, // 56  X           336
  5, 0b00000111, 0b00001000, 0b01110000, 0b00001000, 0b00000111, // 57  Y           342
  4, 0b01100001, 0b01010001, 0b01001001, 0b01000111, 0b00000000, // 58  Z 90        348
  2, 0b01111111, 0b01000001, 0b00000000, 0b00000000, 0b00000000, // 59  [           354
  4, 0b00000001, 0b00000110, 0b00011000, 0b01100000, 0b00000000, // 60  '\'         360
  2, 0b01000001, 0b01111111, 0b00000000, 0b00000000, 0b00000000, // 61  ]           366
  3, 0b00000010, 0b00000001, 0b00000010, 0b00000000, 0b00000000, // 62  hat         372
  4, 0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b00000000, // 63  _           378
  2, 0b00000001, 0b00000010, 0b00000000, 0b00000000, 0b00000000, // 64  `           384
  4, 0b00100000, 0b01010100, 0b01010100, 0b01111000, 0b00000000, // 65  a           390
  4, 0b01111111, 0b01000100, 0b01000100, 0b00111000, 0b00000000, // 66  b           396
  4, 0b00111000, 0b01000100, 0b01000100, 0b00000000, 0b00000000, // 67  c           402
  4, 0b00111000, 0b01000100, 0b01000100, 0b01111111, 0b00000000, // 68  d 100       408
  4, 0b00111000, 0b01010100, 0b01010100, 0b00011000, 0b00000000, // 69  e           414
  3, 0b00000100, 0b01111110, 0b00000101, 0b00000000, 0b00000000, // 70  f           420
  4, 0b10011000, 0b10100100, 0b10100100, 0b01111000, 0b00000000, // 71  g           426
  4, 0b01111111, 0b00000100, 0b00000100, 0b01111000, 0b00000000, // 72  h           432
  3, 0b01000100, 0b01111101, 0b01000000, 0b00000000, 0b00000000, // 73  i           438
  4, 0b01000000, 0b10000000, 0b10000100, 0b01111101, 0b00000000, // 74  j           444
  4, 0b01111111, 0b00010000, 0b00101000, 0b01000100, 0b00000000, // 75  k           450
  3, 0b01000001, 0b01111111, 0b01000000, 0b00000000, 0b00000000, // 76  l           456
  5, 0b01111100, 0b00000100, 0b01111100, 0b00000100, 0b01111000, // 77  m           462
  4, 0b01111100, 0b00000100, 0b00000100, 0b01111000, 0b00000000, // 78  n 110       468
  4, 0b00111000, 0b01000100, 0b01000100, 0b00111000, 0b00000000, // 79  o           474
  4, 0b11111100, 0b00100100, 0b00100100, 0b00011000, 0b00000000, // 80  p           480
  4, 0b00011000, 0b00100100, 0b00100100, 0b11111100, 0b00000000, // 81  q           486
  4, 0b01111100, 0b00001000, 0b00000100, 0b00000100, 0b00000000, // 82  r           492
  4, 0b01001000, 0b01010100, 0b01010100, 0b00100100, 0b00000000, // 83  s           498
  3, 0b00000100, 0b00111111, 0b01000100, 0b00000000, 0b00000000, // 84  t           504
  4, 0b00111100, 0b01000000, 0b01000000, 0b01111100, 0b00000000, // 85  u           510
  5, 0b00011100, 0b00100000, 0b01000000, 0b00100000, 0b00011100, // 86  v           516
  5, 0b00111100, 0b01000000, 0b00111100, 0b01000000, 0b00111100, // 87  w           522
  5, 0b01000100, 0b00101000, 0b00010000, 0b00101000, 0b01000100, // 88  x 120       528
  4, 0b10011100, 0b10100000, 0b10100000, 0b01111100, 0b00000000, // 89  y           534
  3, 0b01100100, 0b01010100, 0b01001100, 0b00000000, 0b00000000, // 90  z           540
  3, 0b00001000, 0b00110110, 0b01000001, 0b00000000, 0b00000000, // 91  {           546
  1, 0b01111111, 0b00000000, 0b00000000, 0b00000000, 0b00000000, // 92  |           552
  3, 0b01000001, 0b00110110, 0b00001000, 0b00000000, 0b00000000, // 93  }           558      
  4, 0b00001000, 0b00000100, 0b00001000, 0b00000100, 0b00000000, // 94  ~           564
  4, 0b00100000, 0b01010110, 0b01010101, 0b01111000, 0b00000000, // 95  á 127       570      =  95 * 6       Original: 4, 0b00100000, 0b01010100, 0b01010100, 0b01111000, 0b00000000, // a           390
  4, 0b00111000, 0b01000100, 0b11000100, 0b00000000, 0b00000000, // 96  ç 128       576      =  96 * 6       Original: 4, 0b00111000, 0b01000100, 0b01000100, 0b00000000, 0b00000000, // c           402
  3, 0b01000100, 0b01111110, 0b01000001, 0b00000000, 0b00000000, // 97  í 129       582      =  97 * 6       Original: 3, 0b01000100, 0b01111101, 0b01000000, 0b00000000, 0b00000000, // i           438
  4, 0b00111000, 0b01000100, 0b01000110, 0b00111001, 0b00000000, // 98  ó 130       588      =  98 * 6       Original: 4, 0b00111000, 0b01000100, 0b01000100, 0b00111000, 0b00000000, // o           474
  4, 0b00110010, 0b01001001, 0b01001001, 0b00110010, 0b00000000, // 99  ô 131       594      =  99 * 6       Original: 4, idem
  3, 0b00000111, 0b00000101, 0b00000111, 0b00000000, 0b00000000, //100  ° 132       600      = 100 * 6
};

// LED matrix info.
const uint8_t led_matrix_pin_first = 28;
const uint8_t led_matrix_pin_last = 38;
const uint8_t led_matrix_pin_count = led_matrix_pin_last - led_matrix_pin_first + 1;
const uint8_t led_matrix_rows = 8;
const uint8_t led_matrix_cols = 12;

// Pixel-to-pin translation table.
// A HEX value encodes two pin numbers. The MSB is to be driven LOW,
// the LSB is to be driven HIGH.
// Example: pixel (4,2) contains the value 0x60, meaning that pin 6 must
// be driven low and pin 0 must be driven high to activate the pixel.
// The pin number is an offset to the constant led_matrix_pin_first
// Note that they all appear in pairs, so you could make the table 50%
// smaller at the cost of doing some swapping for odd or even columns.
// (0,0) is upper left corner when the board's USB connector points to the left.
const uint8_t led_matrix_pins[led_matrix_rows][led_matrix_cols] =
{
  //  0     1     2     3     4     5     6     7     8     9    10     11
  { 0x37, 0x73, 0x47, 0x74, 0x43, 0x34, 0x87, 0x78, 0x83, 0x38, 0x84, 0x48 }, // 0
  { 0x07, 0x70, 0x03, 0x30, 0x04, 0x40, 0x08, 0x80, 0x67, 0x76, 0x63, 0x36 }, // 1
  { 0x64, 0x46, 0x68, 0x86, 0x60, 0x06, 0x57, 0x75, 0x53, 0x35, 0x54, 0x45 }, // 2
  { 0x58, 0x85, 0x50, 0x05, 0x56, 0x65, 0x17, 0x71, 0x13, 0x31, 0x14, 0x41 }, // 3
  { 0x18, 0x81, 0x10, 0x01, 0x16, 0x61, 0x15, 0x51, 0x27, 0x72, 0x23, 0x32 }, // 4
  { 0x24, 0x42, 0x28, 0x82, 0x20, 0x02, 0x26, 0x62, 0x25, 0x52, 0x21, 0x12 }, // 5
  { 0xa7, 0x7a, 0xa3, 0x3a, 0xa4, 0x4a, 0xa8, 0x8a, 0xa0, 0x0a, 0xa6, 0x6a }, // 6
  { 0xa5, 0x5a, 0xa1, 0x1a, 0xa2, 0x2a, 0x97, 0x79, 0x93, 0x39, 0x94, 0x49 }, // 7
};

// Every byte represents a column of the LED matrix.
// Can hold BANNER_LEN (was: 32) 5x8-font characters.
// Buffer can be smaller at the price of more code.
uint8_t led_matrix_buffer[5*BANNER_LEN];

// Suggested by MS Copilot to avoid floating pins.
// before calling get_bme_data().
void reset_matrix_pins() {
  for (uint8_t i = 0; i < led_matrix_pin_count; i++) {
    pinMode(led_matrix_pin_first + i, INPUT_PULLUP); // or just INPUT
  }
}

// Activate the pixel at (x,y) for ontime microseconds.
void put_pixel(uint8_t x, uint8_t y, uint32_t ontime)
{
  
  uint8_t pins = led_matrix_pins[y][x];
  uint8_t l = (pins>>4) + led_matrix_pin_first;
  uint8_t h = (pins&0xf) + led_matrix_pin_first;
  
  if (l == 20 || l == 21 || h == 20 || h == 21) {
    DEBUG_PRINTF("put_pixel(): l = %d, h = %d\n", l, h);
    return;
  }

  pinMode(l,OUTPUT);
  digitalWrite(l,LOW);
  pinMode(h,OUTPUT);
  digitalWrite(h,HIGH);
  // If ontime = 0, pixel remains active until it is deactivated
  // by another put_pixel that happens to use the same pin(s).
  if (ontime!=0)
  {
    delayMicroseconds(ontime);
    pinMode(l,INPUT);
    pinMode(h,INPUT);
  }
}

// Call periodically at desired fps rate.
// ontime specifies how long a pixel remains on.
void led_matrix_buffer_show(uint32_t x_offset, uint32_t ontime)
{
  static unsigned long last_refresh = 0;
  if (millis() - last_refresh < 10) return;
  last_refresh = millis();

  for (uint8_t i=0; i<led_matrix_cols; i++)
  {
    if (i+x_offset>=sizeof(led_matrix_buffer)) return;
    uint8_t col = led_matrix_buffer[i+x_offset];
    for (uint8_t row=0; row<led_matrix_rows; row++)
    {
      if ((col&0x01)!=0)
      {
        put_pixel(i,row,ontime);
      }
      col >>= 1;
    }
  }
}

uint8_t col_shown_cnt = 0; // counts how many columns have been shown

// Write a character to the buffer.
uint8_t led_matrix_putch(uint8_t *p_buffer, uint16_t buffer_size, uint8_t ch)
{
  uint8_t i;
  uint8_t ch_cpy = ch;
  
  if (ch < ' ') return 0;
  ch -= ' '; // make space the first character (index 0)
  /*
  DEBUG_PRINTLN(F("\n-------------------------"));
  DEBUG_PRINT(F("led_matrix_putch(): Original ch= "));
  DEBUG_PRINTF("0x%02x", ch_cpy);
  DEBUG_PRINT(F(" + 0x20 = "));
  DEBUG_PRINTF("0x%02x\n", ch_cpy+' ');
  DEBUG_PRINTF("ch= 0x%02x ", ch);
  DEBUG_PRINT(F(" + 0x20 = "));
  DEBUG_PRINTF("0x%02x\n", ch+' ');
  */
  uint16_t offset = 0;
  if (ch == 162 || ch == 163 || ch == 194)  // Check for accent value indicator (Pound sign £ or Latin capital letter A with circumflex Â)
  {
    accent_flag = true; // set flag because an accent char comes with 2 bytes

    //DEBUG_PRINT(F("accent_flag= "));
    //DEBUG_PRINTLN(accent_flag ? "true" : "false");

    if (accent_flag == true) {
      //DEBUG_PRINT(F("Accent detected for char: "));
      //DEBUG_PRINTLN((char)ch_cpy);
      //DEBUG_PRINTLN(F(" -- exiting for next char to determine which accented char to use."));
      return 0;  // it's a ch with an accent: get the next ch
    }
  }
  if (accent_flag == true) {
    accent_flag = false; // reset flag
    //DEBUG_PRINT(F("accent_flag= "));
    //DEBUG_PRINTLN(accent_flag ? "true" : "false");

    //DEBUG_PRINT(F("Processing accented char for code: "));
    //DEBUG_PRINTF("0x%02x\n", ch);
    switch (ch) {
      case 0x81: { // 129 dec
        ch = 127;  // index for á
        break;
      }
      case 0x87: { // 135 dec
        ch = 128;  // index for ç
        break;
      }
      case 0x89: { // 137 dec
        ch = 129;  // index for í
        break;
      }
      case 0x90: { // 176 dec
        //Serial.println(F("Detected degree symbol °"));
        ch = 132;  // index for °
        break;
      }
      case 0x93: { // 147 dec
        ch = 130;  // index for ó
        break;
      }
      case 0x94: { // 148 dec
        ch = 131;  // index for ô
        break;
      }
     
      default: {
        ch = 32; // set index for a space
        break;
      }
    }
    offset = 6*(ch-' ');  // e.g. if ch is 127 then offset is 6*(127-32) = 6*95 = 570
    //serialPrintf("after accent correction, offset = 6*(%d - 32) = %d\n", ch, offset);
  } else {
    offset = 6*ch;
  }

/*
#ifdef MY_DEBUG
  Serial.print("ch after correction= ");
  //Serial.println(ch, HEX);
  serialPrintf("0x%02x\n", ch);
  Serial.print("offset = ");
  Serial.println(offset);
#endif
*/
  uint8_t width = font_5x8[offset];
  tot_width += width; // calculate to total width of characters displayed
  for (i=0; i<width; i++) { 
    offset += 1;
    // This is supposed to prevent buffer overflow.
    if (i>=buffer_size) break;
    p_buffer[i] = font_5x8[offset];
  }
  return i+1;
}

// Write a string to the buffer.
uint16_t led_matrix_puts(uint8_t *p_buffer, uint16_t buffer_size, uint8_t *p_str) {
  uint8_t *p = p_buffer;
  while (*p_str!=0) {
    p += led_matrix_putch(p,buffer_size-(p-p_buffer),*p_str);
    p_str++;
  }
  return p - p_buffer;
}

void clr_banner_txt() {
  for (uint8_t i = init_banner_txt_len; i < BANNER_LEN; i++) {  // Clear from after "  NTP Sync at: "
    banner_txt[i] = '\0';
  }
}

void clr_led_matrix_bfr() {
  const char txt0[] = "clr_led_matrix_bfr(): ";

  //DEBUG_PRINT(txt0);
  //DEBUG_PRINTLN("Starting clear loop...");
  for (uint16_t i = 0; i < 5 * BANNER_LEN; i++) {
    led_matrix_buffer[i] = 0;
    //DEBUG_PRINTF("led_matrix_buffer[%d] = 0x%02x\n", i, led_matrix_buffer[i]);
  }
  DEBUG_PRINT(txt0);
  DEBUG_PRINTLN("Buffer cleared.");
}

void swap_banner(uint8_t b_cnt) {
  const char txt0[] = "swap_banner(): ";  // No PROGMEM !
  size_t le;
  char stby_txt[16] = {0};  // All bytes set to '\0'
  strncpy(stby_txt, "  ...standby...", 15);

  if (b_cnt == BANNER0) {
     snprintf(banner_txt, BANNER_LEN, "%s", stby_txt);
  }
  else if (b_cnt == BANNER1) {
    le = strlen(ntp_sync_txt);
    for(uint8_t i=0; i < le; i++) {
      banner_txt[i] = ntp_sync_txt[i];
    }
    banner_txt[le] = '\0';  // Ensure null termination
  }
  else if (b_cnt == BANNER2) {
    //DEBUG_PRINT(txt0);
    //DEBUG_PRINTF("copying rtc_dt_txt: \"%s\"\n", rtc_dt_txt);
    le = strlen(rtc_dt_txt);
    for(uint8_t i=0; i < le; i++) {
      banner_txt[i] = rtc_dt_txt[i];
    }
    banner_txt[le] = '\0';  // Ensure null termination
  }
  else if (b_cnt == BANNER3) {
    DEBUG_PRINT(txt0);
    DEBUG_PRINTLN(F("updating BME data before displaying banner"));
    get_bme_data(); // update t, p, h, a
    bme_matrix_data();  // updates banner_txt directly
  }
  else if (b_cnt == BANNER4) {
    // "Board ID: 13AB373033315832337C2982D574B" // Length 39
    //formatBoardID();
    //snprintf(banner_txt, BANNER_LEN, "Board ID: %s", id_str);
    //snprintf(banner_txt, BANNER_LEN, "Board ID: %s", OpenCyphalUniqueId.c_str());
    char id_tail[7];  // 6 chars + null terminator
    // Why id_str + 26?
    // id_str is 32 characters long (16 bytes × 2 hex digits)
    // To get the last 6 characters: start at index 32 - 6 = 26
    strncpy(id_tail, id_str + 26, 6);  // Copy from position 26 to 31
    id_tail[6] = '\0';  // Ensure null termination
    snprintf(banner_txt, BANNER_LEN, "  BOARD ID suffix: %s", id_tail);
    // or: snprintf(banner_txt, BANNER_LEN, "BOARD ID suffix %-6s", id_tail);  // to have the suffix appear at a fixed matrix column
  }

  // Don't print for NTP or RTC datetime banners
  if (b_cnt != BANNER1 && b_cnt != BANNER2) {
    DEBUG_PRINT(txt0);
    DEBUG_PRINT(F("Final banner_txt:  \""));
    DEBUG_PRINT(banner_txt);
    DEBUG_PRINTLN("\"");
  }
}

void standby_banner() {
  clr_led_matrix_bfr();
  avg_cols = stby_avg_cols;
  matrix_active = false;
  //reset_matrix_pins();
  delay(10);
  swap_banner(BANNER0); // updates banner_txt directly
  led_off();  // make sure the led is off
  matrix_active = true;
  if (matrix_active == true) {
    led_matrix_puts(led_matrix_buffer, sizeof(led_matrix_buffer), (uint8_t*)banner_txt);
  }
  do_scroll();
}

size_t calc_banner_txt_len() {
  return strlen(banner_txt);
}

void fill_banner_w_curtime(uint8_t scroll, uint16_t ontime) {
  const char txt0[] = "fill_banner_w_curtime(): ";  // No PROGMEM !!!
  clr_banner_txt();

  //DEBUG_PRINTF("%sinit_banner_txt_len = %u\n", txt0, init_banner_txt_len);

  RTC.getTime(currentTime); // is a global variable

  // Option 1: Explicit call to toString()
  arduino::String isoString1 = currentTime.toString();
  Serial.print("ISO 8601 (explicit): ");
  Serial.println(isoString1);

  // Option 2: Implicit conversion to arduino::String
  //arduino::String isoString2 = now;
  //Serial.print("ISO 8601 (implicit): ");
  //Serial.println(isoString2);

  //for (uint8_t i=0; i < sizeof(currentTime); i++) {
  //  banner_txt[init_banner_txt_len + i] = String(currentTime)[i];
  //}
  for (uint8_t i = 0; i < isoString1.length(); i++) {
    banner_txt[init_banner_txt_len + i] = isoString1[i];
  }
  banner_txt[BANNER_LEN - 1] = '\0';  // Make sure banner_txt is null-terminated!

  DEBUG_PRINTF("%sbanner_txt: \"%s\"\n", txt0, banner_txt);
  //DEBUG_PRINTF("%sbanner_text length = %u\n", txt0, calc_banner_txt_len());

  // Load text message.
  led_matrix_puts(led_matrix_buffer, sizeof(led_matrix_buffer), (uint8_t*)banner_txt);

}

void pr_banner() {
  tot_char_width = calc_banner_txt_len(); // corrective bias
  // do not use here the macro DEBUG_PRINTF !
  serialPrintf("banner_txt = \"%s\"\n", banner_txt);
  serialPrintf("banner_txt length = %u characters.\n", tot_char_width); // %u for unsigned int
}

bool led_is_on = false;

#define led_sw_cnt 20  // Defines limit of time count led stays on

void led_on() {
  //blinks the built-in LED every second
  digitalWrite(LED_BUILTIN, HIGH);
  led_is_on = true;
  //delay(1000);
}

void led_off() {
  digitalWrite(LED_BUILTIN, LOW);
  led_is_on = false;
  //delay(1000);
}

time_t getCurrentUnixTimeFromRTC() {
  RTCTime t;
  RTC.getTime(t);
  return t.getUnixTime();
}

void syncSystemTimeFromRTC() {
  const char txt0[] = "syncSystemTimeFromRTC(): ";
  time_t rtcTime = getCurrentUnixTimeFromRTC();  // RTCTime → time_t
  DEBUG_PRINT(txt0);
  DEBUG_PRINT(F("rtcTime = "));
  DEBUG_PRINTLN(rtcTime);

  setTime(rtcTime);                            // ✅ uses the 1-argument version
  
  // Verification
  DEBUG_PRINT(txt0);
  DEBUG_PRINT(F("Verification: RTC time = "));
  DEBUG_PRINTLN(ctime(&rtcTime));
}


// Get the current date and time from an NTP server and convert
// it to UTC +1 by passing the time zone offset in hours.
// You may change the time zone offset to your local one.
void sync_ntp() {
  const char txt0[] = "sync_ntp(): ";
  auto unixTime = timeClient.getEpochTime() + utc_offset;
  DEBUG_PRINT(txt0);
  DEBUG_PRINT(F("Unix time = "));
  DEBUG_PRINTLN(unixTime);
  DEBUG_PRINT(txt0);

  RTCTime timeToSet = RTCTime(unixTime);
  DEBUG_PRINT(txt0);
  DEBUG_PRINT(F("Setting RTC to: "));
  DEBUG_PRINTLN(timeToSet);
  RTC.setTime(timeToSet);

  syncSystemTimeFromRTC(); // update system time

  // Get current time from RTC
  RTC.getTime(currentTime); // currentTime is a global variable
  DEBUG_PRINT(txt0);
  DEBUG_PRINT(F("The RTC was just set to: "));
  DEBUG_PRINTLN(String(currentTime));
}


void save_tm_to_EEPROM() {
  const char txt0[] = "save_tm_to_EEPROM(): ";
  // Set EEPROM start address
  int addr = 1;
  int last_ptr_addr = 0;
  Serial.print(txt0);
  Serial.print(F("the currentTime written to EEPROM = \""));
  for (uint8_t i = 0; i < sizeof(currentTime); i++) {
    /*
    unsigned char val = String(currentTime)[i];
    *ptr = val;
    ptr++;
    */
    EEPROM.write(addr, String(currentTime)[i]);
    serialPrintf("%c", String(currentTime)[i]);
    addr++;
  }
  Serial.println("\"");
  Serial.print(txt0);
  Serial.print(F("value of pointer after save: "));
  serialPrintf("0x%02x\n", addr);
  /*
  last_ptr_addr = ptr
  ptr = 0;
  *ptr = last_ptr_addr; // save last pointer address at location 0
  */
  EEPROM.write(0,addr); // Save the last address

}

void read_tm_fm_EEPROM() {
  const char txt0[] = "read_tm_fm_EEPROM(): ";
  // Set EEPROM start address
  int addr = EEPROM.read(0); // read the value of the last address
  byte value;
  Serial.print(txt0);
  Serial.print(F("the pointer address read from EEPROM address 0 = 0x"));
  Serial.println(addr, HEX);
  Serial.print(txt0);
  Serial.print(F("the currentTime read from EEPROM = \""));
  for (int i = 1; i < addr; i++) {
    value = EEPROM.read(i);
    if (value == 0) break;
    serialPrintf("%c", value);
  }
  Serial.println("\"");
}

void clear_EEPROM() {
  int addr = EEPROM.read(0); // read the value of the last address
  if (addr >= EEPROM.length()-1) return;
#ifdef MY_DEBUG
  Serial.println(F("clear_EEPROM(): setting address: "));
#endif
  for (int i = 1; i < addr; i++) {
    //serialPrintf("0x%02x", i);
    //Serial.print(" to value 0xFF\n");
    EEPROM.write(i, 0xFF);
  }
#ifdef MY_DEBUG
  Serial.println();
#else
  Serial.println("Clear_EEPROM(): EEPROM cleared");
#endif
}

void do_scroll() {
  const char txt0[] = "do_scroll(): ";
  while (true) {
    if ((scroll >= 0 && scroll < avg_cols) && (led_is_on == false)) {
      //DEBUG_PRINT(txt0);
      //DEBUG_PRINTLN(F("Switching led ON"));
      led_on();
    }
    //DEBUG_PRINT(txt0);
    //DEBUG_PRINTF("scroll = %d, avg_cols = %d, led = %s\n", scroll, avg_cols, (led_is_on == 1) ? "true" : "false");
    // Refresh display
    if (matrix_active == true) {
      led_matrix_buffer_show(scroll,ontime);
    }

    // Update scroll position.
    if (millis()>=t_prev+scroll_speed) {
      t_prev = millis();
      scroll += 1; // Scroll to the left.
      //DEBUG_PRINT(txt0);
      //DEBUG_PRINTF("scroll = %d, avg_cols = %d\n", scroll, avg_cols);
      if ((scroll >= avg_cols) && (led_is_on == true)) {
        //DEBUG_PRINT(txt0);
        //DEBUG_PRINTLN(F("Switching led OFF"));
        led_off();
        //DEBUG_PRINTF("scroll = %d, avg_cols = %d, led = %s\n", scroll, avg_cols, (led_is_on == 1) ? "true" : "false");
      }
      size_t le = strlen(banner_txt);
      if (scroll > (5*le)) {
        scroll = 0;
        break;
      }
    }
  }
}

void pr_line() {
  for (uint8_t i=0; i<82; i++)
    Serial.print("-");
  Serial.println();
}

/* currentTime is a global variable */
void pr_currtime() {
  Serial.print("currentTime = ");
  for (uint8_t i=0; i < sizeof(currentTime); i++) {
    char c = String(currentTime)[i];
    if (c == 0) 
      break;
    Serial.print(c);
  }
  Serial.println();
}

#if defined(ARDUINO_UNOR4_WIFI)
const char boardName[] = "Arduino Uno R4 WiFi";
#else
const char boardName[] = "Unknown";
#endif

void formatBoardID() {
  for (size_t i = 0; i < OpenCyphalUniqueId.ID_SIZE; i++) { 
    sprintf(&id_str[i * 2], "%02X", OpenCyphalUniqueId[i]);
  }
  id_str[32] = '\0';  // Null-terminate
  //DEBUG_PRINT(F("Formatted ID string: "));
  //DEBUG_PRINTLN(id_str);
  //DEBUG_PRINT(F("Length: "));
  //DEBUG_PRINTLN(strlen(id_str));
}

const char* getBoardID() {
  return id_str;
}

void pr_Id() {
  Serial.print(F("\nThis sketch is running on a"));
  if (strstr(boardName, "Arduino") != NULL)
    Serial.print("n ");
  else
    Serial.print(" ");
  Serial.print(boardName);
  Serial.print(F(" board,\nwith the Unique ID: \""));
  Serial.print(getBoardID());
  Serial.println("\"");
}

// From MS Copilot
int getYearFromUnix(time_t uxTime) {
    struct tm *timeinfo = gmtime(&uxTime);
    if (timeinfo == nullptr) return -1;  // Or some sentinel value
    return timeinfo->tm_year + 1900;
}

// From MS Copilot
bool isDST() {
  static constexpr const char txt0[] PROGMEM = "isDST(): ";
  if (lStart) {
    Serial.println(F("DST years in map:"));
    for (const auto& entry : dst_start_end) {
      Serial.print(" - ");
      Serial.println(entry.first.c_str());
    }
  }
  DEBUG_PRINT(txt0);
  RTCTime curTime;
  RTC.getTime(curTime);
  time_t currentUnix = curTime.getUnixTime();
  DEBUG_PRINT(F("RTC.getTime() = "));
  DEBUG_PRINTLN(curTime);
  int yy = curTime.getYear();  // Your existing helper function
  DEBUG_PRINT(txt0);
  DEBUG_PRINT(F("Year from RTC.getTime = "));
  DEBUG_PRINTF("%d\n", yy);
  std::string yearStr = std::to_string(yy);
  //DEBUG_PRINT(txt0);
  //DEBUG_PRINTF("yy = %d, yearStr = %s\n", yy, yearStr.c_str());
  auto it = dst_start_end.find(yearStr);
  if (it != dst_start_end.end()) {
    time_t dstStart = it->second.start;
    time_t dstEnd = it->second.end;
    if (currentUnix >= dstStart && currentUnix < dstEnd) {
      DEBUG_PRINT(txt0);
      DEBUG_PRINT(F("We're in ")); // WEST time"));
      DEBUG_PRINTLN(SECRET_TIMEZONE_DST_ID);
      return true;
    } else {
      DEBUG_PRINT(txt0);
      DEBUG_PRINT(F("We're in ")); //WE_IDe"));
      DEBUG_PRINTLN(SECRET_TIMEZONE_STD_ID);
      return false;
    }
  } else {
    DEBUG_PRINT(txt0);
    DEBUG_PRINT(F("No DST data for this year ("));
    DEBUG_PRINT(yearStr.c_str());
    DEBUG_PRINTLN(F(")"));
    return false;
  }
}

void setup(void) {
  const char txt0[] = "setup(): ";
  Serial.begin(115200);
  //while (!Serial);  // Prefere not to use a blocking loop
  delay(1000);
  
  formatBoardID(); // call this function just once to set id_str and before the call to pr_Id()
  pr_Id();

  Serial.print(F("Firmware version: "));
  Serial.println(fwVersion);

  initBannerText(ntp_sync_txt, "  NTP synctime: ");
  initBannerText(rtc_dt_txt, "  RTC datetime: ");
  initBannerText(bme_data_txt, "  BME280 data:  ");
  init_banner_txt_len = strlen(rtc_dt_txt);

  standby_banner();
  txt_idx += 1;

  /*
  Wire1.end();
  delay(10);
  Wire1.begin();
  Wire1.setClock(100000);
  delay(10);
  unsigned status = bme.begin(sensor_address, &Wire1);
  */
  unsigned status = resetBME280();
  sensor_type(status);

#ifdef USE_FORCED_MEASUREMENT
  bme.setSampling(
    Adafruit_BME280::MODE_FORCED,
    Adafruit_BME280::SAMPLING_X1,
    Adafruit_BME280::SAMPLING_X1,
    Adafruit_BME280::SAMPLING_X1,
    Adafruit_BME280::FILTER_OFF);
#else
  bme.setSampling(
    Adafruit_BME280::MODE_NORMAL,
    Adafruit_BME280::SAMPLING_X1,   // temperature
    Adafruit_BME280::SAMPLING_X1,   // pressure
    Adafruit_BME280::SAMPLING_X1,   // humidity
    Adafruit_BME280::FILTER_OFF
  );
  DEBUG_PRINTLN(F("BME280 Sampling mode: MODE_NORMAL"));
#endif
  get_bme_data();  // initial read to set t, p, h, a
  //pr_bme_data(); // print initial BME data to Serial

  //define LED_BUILTIN as an output
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize LED matrix pins.
  for (uint8_t i=0; i<led_matrix_pin_count; i++)
  {
    pinMode(led_matrix_pin_first+i,INPUT); // all off
  }

  // Ready...
  t_prev = millis();
  // Go!

  connectToWiFi();
  //RTC.begin();
  if (!RTC.begin()) {
    DEBUG_PRINT(txt0);
    DEBUG_PRINTLN("RTC.begin() failed!");
  }
    
  setSyncProvider([]() {
    RTCTime rtcNow;
    RTC.getTime(rtcNow);
    return rtcNow.getUnixTime();
  });

  Serial.println(F("\nStarting connection to server..."));
  //Udp.begin(LOCAL_PORT);
  timeClient.begin();
  timeClient.update();

  sync_ntp();  // Get NTP datetime stamp and update internal RTC
  
  setSyncInterval(1);  // Sync every second
  time_t current = now();  // ✅ This calls the TimeLib function
  DEBUG_PRINT(txt0);
  DEBUG_PRINT(F("System time now = "));
  DEBUG_PRINTLN(ctime(&current));

  // Retrieve the date and time from the RTC and print them

  if (isDST())
    tzOffset = tzDST_Offset;
  else
    tzOffset = tzSTD_Offset;
  utc_offset = tzOffset * 3600;
  Serial.print(txt0);
  Serial.print(F("Timezone utc offset = "));
  if (tzOffset < 0)
    Serial.print("-");
  Serial.print(abs(utc_offset)/3600);
  Serial.println(F(" hour(s)"));
 
  RTC.getTime(currentTime); 

#ifdef USE_EEPROM
  save_tm_to_EEPROM();  // Save currentTime to EEPROM 
  read_tm_fm_EEPROM();  // For check: read saved currentTime from EEPROM
//#ifdef MY_DEBUG
//  clear_EEPROM();  // Eventually, clear the EEPROM memory where the currentTima has been saved
//  read_tm_fm_EEPROM();  // double check after clearing EEPROM
//#endif
#endif

  Serial.print(txt0);
  Serial.print(F("The RTC was just set to: "));
  Serial.println(String(currentTime));
#ifdef USE_OLD_LOOP
  pr_line();
  DEBUG_PRINTF("Showing BANNER: %d\n", txt_idx);
  clr_led_matrix_bfr();
  swap_banner(BANNER1);
  fill_banner_w_curtime(scroll, ontime); // clears & fills the banner_txt
  // Refresh display.
  if (matrix_active == true) {
    led_on();
    led_matrix_buffer_show(scroll, ontime);
    led_off();
  }
  //pr_banner();

  scroll = 0;
  avg_cols = ntp_avg_cols;

  do_scroll();
#endif
  txt_idx = BANNER2; 
}

unsigned long s_time = millis(); // start time
unsigned long sntp_time = s_time; // start time (for ntp sync)
unsigned long c_time = 0;  // current time
unsigned long d_time = 0;  // difference time
unsigned long n_time = 0;  // ntp sync time
unsigned long b2_s_time = s_time; // banner2 start time
unsigned long b2_c_time = 0; // banner2 current time
unsigned long b2_d_time = 0; // banner2 difference time
#define I_NTP_SYNC 900000 // interval time NTP sync about 15 minutes
#define I_TM  30000 // interval time, about 1/2 minute (+ text scroll time (usually 16 secs))

uint8_t txt_idx_old = 0;
uint8_t scroll_old = 0;

void loop() {
  const char txt0[] = "loop(): "; // No PROGMEM !
  // Scroll speed is determined by both scroll_speed and ontime.
  /*
  const uint16_t scroll_speed = 100; // milliseconds
  const uint16_t ontime = 521; // microseconds. 521 (us) * 96 (pixels) = 50 ms frame rate if all the pixels are on.
  static uint8_t scroll = 0; // scroll position.
  */
  c_time = millis();
  d_time = c_time - s_time; 
  n_time = c_time - sntp_time;

  if ((n_time  >= I_NTP_SYNC) && (scroll == 0)) { 
    sntp_time = c_time;  // update 
    sync_ntp();  // Get NTP datetime stamp and update internal RTC
    if (txt_idx == BANNER1) {
      clr_led_matrix_bfr();
      swap_banner(BANNER1);
      fill_banner_w_curtime(scroll, ontime); // clears & fills the banner_txt
      if (matrix_active == true)
        led_matrix_buffer_show(scroll, ontime);
      //pr_banner();
    }
  }

  if (txt_idx_old != txt_idx) { // scroll_old != scroll) 
    txt_idx_old = txt_idx;
    scroll_old = scroll;
  }

  // At lStart or when the current text has been scrolled
  if ( lStart || scroll == 0 ) { 
    pr_line();
    DEBUG_PRINTF("Showing BANNER: %d\n", txt_idx);
    
    s_time = c_time;

    if (txt_idx == BANNER2) {
      if (!lStart) {
        b2_c_time = millis();
        b2_d_time = b2_c_time - b2_s_time;   
        DEBUG_PRINT(txt0);
        DEBUG_PRINT(F("b2 loop duration: ")); // One b2 loop takes 53 seconds (status: 2025-10-25 at 22h25 utc +1) 53498
        DEBUG_PRINT(b2_d_time);
        DEBUG_PRINTLN(F(" mSeconds"));
        b2_s_time = b2_c_time;
      }
      clr_led_matrix_bfr();
      // Retrieve the date and time from the RTC and print them
      RTC.getTime(currentTime);

      //Serial.println();
      Serial.print(F("The RTC datetime: "));
      Serial.println(String(currentTime));

      avg_cols = rtc_avg_cols;
      matrix_active = true;
      // Load text message.
      swap_banner(BANNER2);
      fill_banner_w_curtime(scroll, ontime); // clears & fills the banner_txt
      if (matrix_active == true) {
        led_on();
        led_matrix_buffer_show(scroll, ontime);
        led_off();
        scroll = 0;
      }
//#ifdef MY_DEBUG
      //pr_currtime();
//#endif
      //pr_banner();
      do_scroll(); // scrolls the current banner_txt
    }
    else if (txt_idx == BANNER3) {
      clr_led_matrix_bfr();
      avg_cols = bme_avg_cols;
      matrix_active = false;
      //reset_matrix_pins();
      delay(10);
      swap_banner(BANNER3); // updates banner_txt directly
      matrix_active = true;
      if (matrix_active == true) {
        led_matrix_puts(led_matrix_buffer, sizeof(led_matrix_buffer), (uint8_t*)banner_txt);
        //led_matrix_buffer_show(scroll, ontime);
      }
      //pr_banner();
      do_scroll(); // scrolls the current banner_txt
    }
    else if (txt_idx == BANNER4) {
      clr_led_matrix_bfr();
      avg_cols = id_avg_cols;
      matrix_active = false;
      //reset_matrix_pins();
      delay(10);
      swap_banner(BANNER4); // updates banner_txt directly
      matrix_active = true;
      if (matrix_active == true) {
        led_matrix_puts(led_matrix_buffer, sizeof(led_matrix_buffer), (uint8_t*)banner_txt);
        //led_matrix_buffer_show(scroll, ontime);
      }
      //pr_banner();
      do_scroll(); // scrolls the current banner_txt
    }
    DEBUG_PRINT(txt0);
    DEBUG_PRINTF("BANNER%d done!\n",txt_idx);
    txt_idx += 1;
    if (txt_idx > BANNER4)
      txt_idx = BANNER2; 
    DEBUG_PRINT(txt0);
    DEBUG_PRINTF("changing to BANNER%d\n", txt_idx);
    if (lStart) 
      lStart = false;
  }
}
