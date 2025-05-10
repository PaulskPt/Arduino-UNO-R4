/*
   Arduino Watch Lite Version
   You may find full version at: https://github.com/moononournation/ArduinoWatch

   2025-05-10 many changes by Paulus Schulinck (Github handle: @PaulskPt)
   Added functionality to get a intervals a NTP datetime stamp (unixTime)
   Next sync the RTC built-in the used Arduino Uno R4 WiFi with the received unixTime
   Next get the unixTime of the RTC instead of millis() to calculate the next position of
   the three hands of the clock (hour, minute and second).
   Added functionality to add SECRET_TIMEZONE_OFFSET in file arduino_secrets.h, e.g.: "1"
   for Europe/Lisbon, or "-4" for America/New_York.
*/

/*******************************************************************************
 * Start of Arduino_GFX setting
 * 
 * Arduino_GFX try to find the settings depends on selected board in Arduino IDE
 * Or you can define the display dev kit not in the board list
 * Defalult pin list for non display dev kit:
 * Arduino Nano, Micro and more: TFT_CS:  9, TFT_DC:  8, TFT_RST:  7, TFT_BL:  6
 * ESP32 various dev board     : TFT_CS:  5, TFT_DC: 27, TFT_RST: 33, TFT_BL: 22
 * ESP32-C3 various dev board  : TFT_CS:  7, TFT_DC:  2, TFT_RST:  1, TFT_BL:  3
 * ESP32-S2 various dev board  : TFT_CS: 34, TFT_DC: 26, TFT_RST: 33, TFT_BL: 21
 * ESP8266 various dev board   : TFT_CS: 15, TFT_DC:  4, TFT_RST:  2, TFT_BL:  5
 * Raspberry Pi Pico dev board : TFT_CS: 17, TFT_DC: 27, TFT_RST: 26, TFT_BL: 28
 * RTL872x various dev board   : TFT_CS: 18, TFT_DC: 17, TFT_RST:  2, TFT_BL: 23
 * Seeeduino XIAO dev board    : TFT_CS:  3, TFT_DC:  2, TFT_RST:  1, TFT_BL:  0
 * Teensy 4.1 dev board        : TFT_CS: 39, TFT_DC: 41, TFT_RST: 40, TFT_BL: 22
 ******************************************************************************/
#include <Arduino_GFX_Library.h>

#include <Time.h>
#include <TimeLib.h>
#include <stdio.h>
#include "RTC.h"
#include <NTPClient.h>
#include <string.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>
#include "arduino_secrets.h" 

#define TFT_CS  9
#define TFT_DC  8
#define TFT_RST 7
#define TFT_BL  6

bool my_debug = false;

/* More dev device declaration: https://github.com/moononournation/Arduino_GFX/wiki/Dev-Device-Declaration */
#if defined(DISPLAY_DEV_KIT)
Arduino_GFX *gfx = create_default_Arduino_GFX();
#else /* !defined(DISPLAY_DEV_KIT) */

/* More data bus class: https://github.com/moononournation/Arduino_GFX/wiki/Data-Bus-Class */
Arduino_DataBus *bus = create_default_Arduino_DataBus();

/* More display class: https://github.com/moononournation/Arduino_GFX/wiki/Display-Class */
Arduino_GFX *gfx = new Arduino_GC9A01(bus, 7 /* RST */, 0 /* rotation */, true /* IPS */);

#endif /* !defined(DISPLAY_DEV_KIT) */
/*******************************************************************************
 * End of Arduino_GFX setting
 ******************************************************************************/

#define BACKGROUND BLACK
#define MARK_COLOR WHITE
#define SUBMARK_COLOR DARKGREY // LIGHTGREY
#define HOUR_COLOR WHITE
#define MINUTE_COLOR BLUE // LIGHTGREY
#define SECOND_COLOR RED

#define SIXTIETH 0.016666667
#define TWELFTH 0.08333333
#define SIXTIETH_RADIAN 0.10471976
#define TWELFTH_RADIAN 0.52359878
#define RIGHT_ANGLE_RADIAN 1.5707963


///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
char timezoneOffsetStr[] = SECRET_TIMEZONE_OFFSET; // 1 For Portugal; -4 for NY, USA etc...
int TZ_OFFSET; // your timezone offset

constexpr unsigned int LOCAL_PORT = 2390;      // local port to listen for UDP packets
//constexpr int NTP_PACKET_SIZE = 48; // NTP timestamp is in the first 48 bytes of the message

// start now time variable in seconds
int time_now = now();

int wifiStatus = WL_IDLE_STATUS;
IPAddress timeServer(162, 159, 200, 123); // pool.ntp.org NTP server
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
WiFiUDP Udp; // A UDP instance to let us send and receive packets over UDP

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
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

void connectToWiFi(){
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  Serial.print(F("\nWiFi firmware version installed: "));
  Serial.println(fv);
  Serial.print(F("firmware latest version: "));
  Serial.println(WIFI_FIRMWARE_LATEST_VERSION);
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

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

/**
 * Calculates the current unix time, that is the time in seconds since Jan 1 1970.
 * It will try to get the time from the NTP server up to `maxTries` times,
 * then convert it to Unix time and return it.
 * You can optionally specify a time zone offset in hours that can be positive or negative.
*/
unsigned long getUnixTime(int8_t timeZoneOffsetHours = 0, uint8_t maxTries = 5){
  // Try up to `maxTries` times to get a timestamp from the NTP server, then give up.
  for (size_t i = 0; i < maxTries; i++){
    sendNTPpacket(timeServer); // send an NTP packet to a time server
    // wait to see if a reply is available
    delay(1000);

    if (Udp.parsePacket()) {
      Serial.println(F("\ngetUnixTime(): packet with unixtime received"));
      Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

      //the timestamp starts at byte 40 of the received packet and is four bytes,
      //or two words, long. First, extract the two words:
      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
      
      // Combine the four bytes (two words) into a long integer
      // this is NTP time (seconds since Jan 1 1900):
      unsigned long secsSince1900 = highWord << 16 | lowWord;

      // Now convert NTP time into everyday time:
      // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
      const unsigned long seventyYears = 2208988800UL;
      unsigned long secondsSince1970 = secsSince1900 - seventyYears + (timeZoneOffsetHours * 3600);
      return secondsSince1970;
    }
  }

  return 0;
}

// Retrieve the date and time from the RTC and print them
RTCTime currentTime;

bool led_is_on = false;

#define led_sw_cnt 20  // Defines limit of time count led stays on

void led_on() {
  //blinks the built-in LED every second
  digitalWrite(LED_BUILTIN, HIGH);
  led_is_on = true;
  //delay(1000);
}

void led_off()
{
  digitalWrite(LED_BUILTIN, LOW);
  led_is_on = false;
  //delay(1000);
}

unsigned long unixTime;
unsigned long long  unixTime2; // Used for calculations in milliseconds

void sync_ntp()
{
  // Get the current date and time from an NTP server and convert
  // it to UTC +1 by passing the time zone offset in hours.
  // You may change the time zone offset to your local one.
  unixTime = getUnixTime(TZ_OFFSET);  // See TZ_OFFSET definition in line 41
  Serial.print(F("sync_ntp(): Unix time = "));
  Serial.println(unixTime);
  RTCTime timeToSet = RTCTime(unixTime);
  Serial.print(F("sync_ntp(): RTC set to: "));
  Serial.println(timeToSet);
  RTC.setTime(timeToSet);
}

static uint8_t conv2d(const char *p)
{
    uint8_t v = 0;
    return (10 * (*p - '0')) + (*++p - '0');
}

static int16_t w, h, center;
static int16_t hHandLen, mHandLen, sHandLen, markLen;
static float sdeg, mdeg, hdeg;
static int16_t osx = 0, osy = 0, omx = 0, omy = 0, ohx = 0, ohy = 0; // Saved H, M, S x & y coords
static int16_t nsx, nsy, nmx, nmy, nhx, nhy;                         // H, M, S x & y coords
static int16_t xMin, yMin, xMax, yMax;                               // redraw range
static int16_t hh, mm, ss;
static unsigned long targetTime; // next action time

static int16_t *cached_points;
static uint16_t cached_points_idx = 0;
static int16_t *last_cached_point;

uint32_t t_prev = 0;

void draw_round_clock_mark(int16_t innerR1, int16_t outerR1, int16_t innerR2, int16_t outerR2, int16_t innerR3, int16_t outerR3)
{
  float x, y;
  int16_t x0, x1, y0, y1, innerR, outerR;
  uint16_t c;

  for (uint8_t i = 0; i < 60; i++)
  {
    if ((i % 15) == 0)
    {
      innerR = innerR1;
      outerR = outerR1;
      c = MARK_COLOR;
    }
    else if ((i % 5) == 0)
    {
      innerR = innerR2;
      outerR = outerR2;
      c = MARK_COLOR;
    }
    else
    {
      innerR = innerR3;
      outerR = outerR3;
      c = SUBMARK_COLOR;
    }

    mdeg = (SIXTIETH_RADIAN * i) - RIGHT_ANGLE_RADIAN;
    x = cos(mdeg);
    y = sin(mdeg);
    x0 = x * outerR + center;
    y0 = y * outerR + center;
    x1 = x * innerR + center;
    y1 = y * innerR + center;

    gfx->drawLine(x0, y0, x1, y1, c);
  }
}

void draw_square_clock_mark(int16_t innerR1, int16_t outerR1, int16_t innerR2, int16_t outerR2, int16_t innerR3, int16_t outerR3)
{
    float x, y;
    int16_t x0, x1, y0, y1, innerR, outerR;
    uint16_t c;

    for (uint8_t i = 0; i < 60; i++)
    {
        if ((i % 15) == 0)
        {
            innerR = innerR1;
            outerR = outerR1;
            c = MARK_COLOR;
        }
        else if ((i % 5) == 0)
        {
            innerR = innerR2;
            outerR = outerR2;
            c = MARK_COLOR;
        }
        else
        {
            innerR = innerR3;
            outerR = outerR3;
            c = SUBMARK_COLOR;
        }

        if ((i >= 53) || (i < 8))
        {
            x = tan(SIXTIETH_RADIAN * i);
            x0 = center + (x * outerR);
            y0 = center + (1 - outerR);
            x1 = center + (x * innerR);
            y1 = center + (1 - innerR);
        }
        else if (i < 23)
        {
            y = tan((SIXTIETH_RADIAN * i) - RIGHT_ANGLE_RADIAN);
            x0 = center + (outerR);
            y0 = center + (y * outerR);
            x1 = center + (innerR);
            y1 = center + (y * innerR);
        }
        else if (i < 38)
        {
            x = tan(SIXTIETH_RADIAN * i);
            x0 = center - (x * outerR);
            y0 = center + (outerR);
            x1 = center - (x * innerR);
            y1 = center + (innerR);
        }
        else if (i < 53)
        {
            y = tan((SIXTIETH_RADIAN * i) - RIGHT_ANGLE_RADIAN);
            x0 = center + (1 - outerR);
            y0 = center - (y * outerR);
            x1 = center + (1 - innerR);
            y1 = center - (y * innerR);
        }
        gfx->drawLine(x0, y0, x1, y1, c);
    }
}

void redraw_hands_cached_draw_and_erase()
{
    gfx->startWrite();
    draw_and_erase_cached_line(center, center, nsx, nsy, SECOND_COLOR, cached_points, sHandLen + 1, false, false);
    draw_and_erase_cached_line(center, center, nhx, nhy, HOUR_COLOR, cached_points + ((sHandLen + 1) * 2), hHandLen + 1, true, false);
    draw_and_erase_cached_line(center, center, nmx, nmy, MINUTE_COLOR, cached_points + ((sHandLen + 1 + hHandLen + 1) * 2), mHandLen + 1, true, true);
    gfx->endWrite();
}

void draw_and_erase_cached_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t color, int16_t *cache, int16_t cache_len, bool cross_check_second, bool cross_check_hour)
{
#if defined(ESP8266)
    yield();
#endif
    bool steep = _diff(y1, y0) > _diff(x1, x0);
    if (steep)
    {
        _swap_int16_t(x0, y0);
        _swap_int16_t(x1, y1);
    }

    int16_t dx, dy;
    dx = _diff(x1, x0);
    dy = _diff(y1, y0);

    int16_t err = dx / 2;
    int8_t xstep = (x0 < x1) ? 1 : -1;
    int8_t ystep = (y0 < y1) ? 1 : -1;
    x1 += xstep;
    int16_t x, y, ox, oy;
    for (uint16_t i = 0; i <= dx; i++)
    {
        if (steep)
        {
            x = y0;
            y = x0;
        }
        else
        {
            x = x0;
            y = y0;
        }
        ox = *(cache + (i * 2));
        oy = *(cache + (i * 2) + 1);
        if ((x == ox) && (y == oy))
        {
            if (cross_check_second || cross_check_hour)
            {
                write_cache_pixel(x, y, color, cross_check_second, cross_check_hour);
            }
        }
        else
        {
            write_cache_pixel(x, y, color, cross_check_second, cross_check_hour);
            if ((ox > 0) || (oy > 0))
            {
                write_cache_pixel(ox, oy, BACKGROUND, cross_check_second, cross_check_hour);
            }
            *(cache + (i * 2)) = x;
            *(cache + (i * 2) + 1) = y;
        }
        if (err < dy)
        {
            y0 += ystep;
            err += dx;
        }
        err -= dy;
        x0 += xstep;
    }
    for (uint16_t i = dx + 1; i < cache_len; i++)
    {
        ox = *(cache + (i * 2));
        oy = *(cache + (i * 2) + 1);
        if ((ox > 0) || (oy > 0))
        {
            write_cache_pixel(ox, oy, BACKGROUND, cross_check_second, cross_check_hour);
        }
        *(cache + (i * 2)) = 0;
        *(cache + (i * 2) + 1) = 0;
    }
}

void write_cache_pixel(int16_t x, int16_t y, int16_t color, bool cross_check_second, bool cross_check_hour)
{
    int16_t *cache = cached_points;
    if (cross_check_second)
    {
        for (uint16_t i = 0; i <= sHandLen; i++)
        {
            if ((x == *(cache++)) && (y == *(cache)))
            {
                return;
            }
            cache++;
        }
    }
    if (cross_check_hour)
    {
        cache = cached_points + ((sHandLen + 1) * 2);
        for (uint16_t i = 0; i <= hHandLen; i++)
        {
            if ((x == *(cache++)) && (y == *(cache)))
            {
                return;
            }
            cache++;
        }
    }
    gfx->writePixel(x, y, color);
}

void do_line() {
    for (uint8_t i=0; i<37; i++)
            Serial.print("-");
        Serial.println();
}

void setup(void) {

    Serial.begin(115200);
    while (!Serial);
  
    //define LED_BUILTIN as an output
    pinMode(LED_BUILTIN, OUTPUT);

    // Ready...
    t_prev = millis();
    // Go!
    Serial.println(F("\nArduino Uno R4 WiFi com Joy-it SBC LCD 1.28R display clock test"));
    connectToWiFi();
    Serial.println("\nStarting connection to NTP server...");
    Udp.begin(LOCAL_PORT);
    RTC.begin();

    if (sscanf(timezoneOffsetStr, "%d", &TZ_OFFSET) == 1) {
        Serial.print(F("Timezone offset in hours: "));
        Serial.println(TZ_OFFSET);
    } else {
        Serial.print(F("Invalid input!\n"));
    }
    sync_ntp();  // Get NTP datetime stamp and update internal RTC
 
    RTC.getTime(currentTime); 

    gfx->begin();
    gfx->fillScreen(BACKGROUND);

#ifdef TFT_BL
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
#endif

    // init LCD constant
    w = gfx->width();
    h = gfx->height();
    if (w < h)
    {
        center = w / 2;
    }
    else
    {
        center = h / 2;
    }
    hHandLen = center * 3 / 8;
    mHandLen = center * 2 / 3;
    sHandLen = center * 5 / 6;
    markLen = sHandLen / 6;
    cached_points = (int16_t *)malloc((hHandLen + 1 + mHandLen + 1 + sHandLen + 1) * 2 * 2);

    // Draw 60 clock marks
    draw_round_clock_mark(
    // draw_square_clock_mark(
        center - markLen, center,
        center - (markLen * 2 / 3), center,
        center - (markLen / 2), center);

    targetTime = currentTime.getUnixTime(); // + 1;  // Note PaulskPt: targetTime = unixTime2 + 1 second

    hh = currentTime.getHour();
    mm = currentTime.getMinutes(); // + 3;
    ss = currentTime.getSeconds(); // + 6;
}

unsigned long s_time = millis();  // start time
unsigned long sntp_time = s_time; // start time (for ntp sync)
unsigned long c_time = 0;  // current time
unsigned long d_time = 0;  // difference time
unsigned long n_time = 0;  // ntp sync time
#define I_NTP_SYNC 60000  // interval time NTP sync about 1 minute (because the Uno R4 RTC is very inaccurate!)
#define I_TM  300000  // interval time, about 5 minutes
bool lStart = true;
bool lStart2 = true;

void loop() {
    if (lStart) {
        Serial.print(F("RTC sync from NTP interval: "));
        Serial.print(I_NTP_SYNC / 60000);
        Serial.println(F(" minutes"));
    }
    c_time = millis();
    d_time = c_time - s_time; 
    n_time = c_time - sntp_time;

    if (n_time  >= I_NTP_SYNC) { // every 15 minutes
      sntp_time = c_time;  // update 
      sync_ntp();  // Get NTP datetime stamp and update internal RTC  }
    }
    // Retrieve the date and time from the RTC
    RTC.getTime(currentTime);
    if ( (lStart) || (d_time  >= I_TM) )  // Every 1 minute
    {
        if (lStart) lStart = false;
        s_time = c_time;
        // Prrint the date and time from RTC
        do_line();
        Serial.print(F("The RTC datetime: "));
        Serial.println(String(currentTime));
        do_line();
    }
    unixTime2 = currentTime.getUnixTime(); // unixTime2 in seconds
    hh = currentTime.getHour();
    mm = currentTime.getMinutes(); // + 3;
    ss = currentTime.getSeconds(); // + 6;
    
    // Pre-compute hand degrees, x & y coords for a fast screen update
    sdeg = SIXTIETH_RADIAN * ((unixTime2 % 1) + ss); // 0-59 seconds
    nsx = cos(sdeg - RIGHT_ANGLE_RADIAN) * sHandLen + center;
    nsy = sin(sdeg - RIGHT_ANGLE_RADIAN) * sHandLen + center;
    if ((nsx != osx) || (nsy != osy))
    {
        mdeg = (SIXTIETH * sdeg) + (SIXTIETH_RADIAN * mm); // 0-59 (includes seconds)
        hdeg = (TWELFTH * mdeg) + (TWELFTH_RADIAN * hh);   // 0-11 (includes minutes)
        mdeg -= RIGHT_ANGLE_RADIAN;
        hdeg -= RIGHT_ANGLE_RADIAN;
        nmx = cos(mdeg) * mHandLen + center;
        nmy = sin(mdeg) * mHandLen + center;
        nhx = cos(hdeg) * hHandLen + center;
        nhy = sin(hdeg) * hHandLen + center;

        // redraw hands
        redraw_hands_cached_draw_and_erase();

        ohx = nhx;
        ohy = nhy;
        omx = nmx;
        omy = nmy;
        osx = nsx;
        osy = nsy;

        delay(1);
    }
}