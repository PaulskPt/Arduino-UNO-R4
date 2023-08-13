/**
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
 * The interval for NTP sync is defined by I_TM.
 */

#include <Time.h>
#include <TimeLib.h>
#include "RTC.h"
#include <NTPClient.h>

//#if defined(ARDUINO_PORTENTA_C33)
//#include <WiFiC3.h>
//#elif defined(ARDUINO_UNOWIFIR4)
#include <WiFiS3.h>
//#endif

#include <WiFiUdp.h>
#include "arduino_secrets.h" 

#ifdef my_debug
#undef my_debug
#endif
// #define my_debug  (1)

#define BANNER_LEN 50

// Leading spaces ensure starting at the right.
//uint8_t banner_text[] = "  Arduino UNO R4 WiFi á í ó ô ç";
uint8_t banner_text[BANNER_LEN] = "  NTP Sync at: ";  // Was: "\0";
int initial_banner_text_len = strlen((char*)banner_text);

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

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
      Serial.println("packet received");
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

bool accent_flag = false;

const uint8_t font_5x8[] = 
{
  3, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, // space  32   offset 0
  1, 0b01011111, 0b00000000, 0b00000000, 0b00000000, 0b00000000, // !             6
  3, 0b00000011, 0b00000000, 0b00000011, 0b00000000, 0b00000000, // "            12
  5, 0b00010100, 0b00111110, 0b00010100, 0b00111110, 0b00010100, // #            18
  4, 0b00100100, 0b01101010, 0b00101011, 0b00010010, 0b00000000, // $            24
  5, 0b01100011, 0b00010011, 0b00001000, 0b01100100, 0b01100011, // %            30
  5, 0b00110110, 0b01001001, 0b01010110, 0b00100000, 0b01010000, // &            36
  1, 0b00000011, 0b00000000, 0b00000000, 0b00000000, 0b00000000, // '            42
  3, 0b00011100, 0b00100010, 0b01000001, 0b00000000, 0b00000000, // ( 40         48
  3, 0b01000001, 0b00100010, 0b00011100, 0b00000000, 0b00000000, // )            54
  5, 0b00101000, 0b00011000, 0b00001110, 0b00011000, 0b00101000, // *            60
  5, 0b00001000, 0b00001000, 0b00111110, 0b00001000, 0b00001000, // +            66
  2, 0b10110000, 0b01110000, 0b00000000, 0b00000000, 0b00000000, // ,            72
  2, 0b00001000, 0b00001000, 0b00000000, 0b00000000, 0b00000000, // -            78
  2, 0b01100000, 0b01100000, 0b00000000, 0b00000000, 0b00000000, // .            84
  4, 0b01100000, 0b00011000, 0b00000110, 0b00000001, 0b00000000, // /            90
  4, 0b00111110, 0b01000001, 0b01000001, 0b00111110, 0b00000000, // 0            96
  3, 0b01000010, 0b01111111, 0b01000000, 0b00000000, 0b00000000, // 1           102
  4, 0b01100010, 0b01010001, 0b01001001, 0b01000110, 0b00000000, // 2 50        108
  4, 0b00100010, 0b01000001, 0b01001001, 0b00110110, 0b00000000, // 3           114
  4, 0b00011000, 0b00010100, 0b00010010, 0b01111111, 0b00000000, // 4           120
  4, 0b00100111, 0b01000101, 0b01000101, 0b00111001, 0b00000000, // 5           126
  4, 0b00111110, 0b01001001, 0b01001001, 0b00110000, 0b00000000, // 6           132
  4, 0b01100001, 0b00010001, 0b00001001, 0b00000111, 0b00000000, // 7           138
  4, 0b00110110, 0b01001001, 0b01001001, 0b00110110, 0b00000000, // 8           144
  4, 0b00000110, 0b01001001, 0b01001001, 0b00111110, 0b00000000, // 9           150
  1, 0b00010100, 0b00000000, 0b00000000, 0b00000000, 0b00000000, // :           156
  2, 0b10000000, 0b01010000, 0b00000000, 0b00000000, 0b00000000, // ;           162
  3, 0b00010000, 0b00101000, 0b01000100, 0b00000000, 0b00000000, // < 60        168
  3, 0b00010100, 0b00010100, 0b00010100, 0b00000000, 0b00000000, // =           174
  3, 0b01000100, 0b00101000, 0b00010000, 0b00000000, 0b00000000, // >           180
  4, 0b00000010, 0b01011001, 0b00001001, 0b00000110, 0b00000000, // ?           186
  5, 0b00111110, 0b01001001, 0b01010101, 0b01011101, 0b00001110, // @           192
  4, 0b01111110, 0b00010001, 0b00010001, 0b01111110, 0b00000000, // A           198
  4, 0b01111111, 0b01001001, 0b01001001, 0b00110110, 0b00000000, // B           204
  4, 0b00111110, 0b01000001, 0b01000001, 0b00100010, 0b00000000, // C           210
  4, 0b01111111, 0b01000001, 0b01000001, 0b00111110, 0b00000000, // D           216
  4, 0b01111111, 0b01001001, 0b01001001, 0b01000001, 0b00000000, // E           222
  4, 0b01111111, 0b00001001, 0b00001001, 0b00000001, 0b00000000, // F 70        228
  4, 0b00111110, 0b01000001, 0b01001001, 0b01111010, 0b00000000, // G           234
  4, 0b01111111, 0b00001000, 0b00001000, 0b01111111, 0b00000000, // H           240
  3, 0b01000001, 0b01111111, 0b01000001, 0b00000000, 0b00000000, // I           246
  4, 0b00110000, 0b01000000, 0b01000001, 0b00111111, 0b00000000, // J           252
  4, 0b01111111, 0b00001000, 0b00010100, 0b01100011, 0b00000000, // K           258
  4, 0b01111111, 0b01000000, 0b01000000, 0b01000000, 0b00000000, // L           264
  5, 0b01111111, 0b00000010, 0b00001100, 0b00000010, 0b01111111, // M           270
  5, 0b01111111, 0b00000100, 0b00001000, 0b00010000, 0b01111111, // N           276
  4, 0b00111110, 0b01000001, 0b01000001, 0b00111110, 0b00000000, // O           282
  4, 0b01111111, 0b00001001, 0b00001001, 0b00000110, 0b00000000, // P 80        288
  4, 0b00111110, 0b01000001, 0b01000001, 0b10111110, 0b00000000, // Q           294
  4, 0b01111111, 0b00001001, 0b00001001, 0b01110110, 0b00000000, // R           300
  4, 0b01000110, 0b01001001, 0b01001001, 0b00110010, 0b00000000, // S           306
  5, 0b00000001, 0b00000001, 0b01111111, 0b00000001, 0b00000001, // T           312
  4, 0b00111111, 0b01000000, 0b01000000, 0b00111111, 0b00000000, // U           318
  5, 0b00001111, 0b00110000, 0b01000000, 0b00110000, 0b00001111, // V           324
  5, 0b00111111, 0b01000000, 0b00111000, 0b01000000, 0b00111111, // W           330
  5, 0b01100011, 0b00010100, 0b00001000, 0b00010100, 0b01100011, // X           336
  5, 0b00000111, 0b00001000, 0b01110000, 0b00001000, 0b00000111, // Y           342
  4, 0b01100001, 0b01010001, 0b01001001, 0b01000111, 0b00000000, // Z 90        348
  2, 0b01111111, 0b01000001, 0b00000000, 0b00000000, 0b00000000, // [           354
  4, 0b00000001, 0b00000110, 0b00011000, 0b01100000, 0b00000000, // '\'         360
  2, 0b01000001, 0b01111111, 0b00000000, 0b00000000, 0b00000000, // ]           366
  3, 0b00000010, 0b00000001, 0b00000010, 0b00000000, 0b00000000, // hat         372
  4, 0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b00000000, // _           378
  2, 0b00000001, 0b00000010, 0b00000000, 0b00000000, 0b00000000, // `           384
  4, 0b00100000, 0b01010100, 0b01010100, 0b01111000, 0b00000000, // a           390
  4, 0b01111111, 0b01000100, 0b01000100, 0b00111000, 0b00000000, // b           396
  4, 0b00111000, 0b01000100, 0b01000100, 0b00000000, 0b00000000, // c           402
  4, 0b00111000, 0b01000100, 0b01000100, 0b01111111, 0b00000000, // d 100       408
  4, 0b00111000, 0b01010100, 0b01010100, 0b00011000, 0b00000000, // e           414
  3, 0b00000100, 0b01111110, 0b00000101, 0b00000000, 0b00000000, // f           420
  4, 0b10011000, 0b10100100, 0b10100100, 0b01111000, 0b00000000, // g           426
  4, 0b01111111, 0b00000100, 0b00000100, 0b01111000, 0b00000000, // h           432
  3, 0b01000100, 0b01111101, 0b01000000, 0b00000000, 0b00000000, // i           438
  4, 0b01000000, 0b10000000, 0b10000100, 0b01111101, 0b00000000, // j           444
  4, 0b01111111, 0b00010000, 0b00101000, 0b01000100, 0b00000000, // k           450
  3, 0b01000001, 0b01111111, 0b01000000, 0b00000000, 0b00000000, // l           456
  5, 0b01111100, 0b00000100, 0b01111100, 0b00000100, 0b01111000, // m           462
  4, 0b01111100, 0b00000100, 0b00000100, 0b01111000, 0b00000000, // n 110       468
  4, 0b00111000, 0b01000100, 0b01000100, 0b00111000, 0b00000000, // o           474
  4, 0b11111100, 0b00100100, 0b00100100, 0b00011000, 0b00000000, // p           480
  4, 0b00011000, 0b00100100, 0b00100100, 0b11111100, 0b00000000, // q           486
  4, 0b01111100, 0b00001000, 0b00000100, 0b00000100, 0b00000000, // r           492
  4, 0b01001000, 0b01010100, 0b01010100, 0b00100100, 0b00000000, // s           498
  3, 0b00000100, 0b00111111, 0b01000100, 0b00000000, 0b00000000, // t           504
  4, 0b00111100, 0b01000000, 0b01000000, 0b01111100, 0b00000000, // u           510
  5, 0b00011100, 0b00100000, 0b01000000, 0b00100000, 0b00011100, // v           516
  5, 0b00111100, 0b01000000, 0b00111100, 0b01000000, 0b00111100, // w           522
  5, 0b01000100, 0b00101000, 0b00010000, 0b00101000, 0b01000100, // x 120       528
  4, 0b10011100, 0b10100000, 0b10100000, 0b01111100, 0b00000000, // y           534
  3, 0b01100100, 0b01010100, 0b01001100, 0b00000000, 0b00000000, // z           540
  3, 0b00001000, 0b00110110, 0b01000001, 0b00000000, 0b00000000, // {           546
  1, 0b01111111, 0b00000000, 0b00000000, 0b00000000, 0b00000000, // |           552
  3, 0b01000001, 0b00110110, 0b00001000, 0b00000000, 0b00000000, // }           558      
  4, 0b00001000, 0b00000100, 0b00001000, 0b00000100, 0b00000000, //             564
  4, 0b00100000, 0b01010110, 0b01010101, 0b01111000, 0b00000000, // á 127       570      = 95 * 6       Original: 4, 0b00100000, 0b01010100, 0b01010100, 0b01111000, 0b00000000, // a           390
  4, 0b00111000, 0b01000100, 0b11000100, 0b00000000, 0b00000000, // ç 128       576      = 96 * 6       Original: 4, 0b00111000, 0b01000100, 0b01000100, 0b00000000, 0b00000000, // c           402
  3, 0b01000100, 0b01111110, 0b01000001, 0b00000000, 0b00000000, // í 129       582      = 96 * 6       Original: 3, 0b01000100, 0b01111101, 0b01000000, 0b00000000, 0b00000000, // i           438
  4, 0b00111000, 0b01000100, 0b01000110, 0b00111001, 0b00000000, // ó 130       588      = 97 * 6       Original: 4, 0b00111000, 0b01000100, 0b01000100, 0b00111000, 0b00000000, // o           474
  4, 0b00110010, 0b01001001, 0b01001001, 0b00110010, 0b00000000, // ô 131       594      = 98 * 6       Original: 4, idem
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

// Activate the pixel at (x,y) for ontime microseconds.
void put_pixel(uint8_t x, uint8_t y, uint32_t ontime)
{
  uint8_t pins = led_matrix_pins[y][x];
  uint8_t l = (pins>>4) + led_matrix_pin_first;
  uint8_t h = (pins&0xf) + led_matrix_pin_first;
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

// Write a character to the buffer.
uint8_t led_matrix_putch(uint8_t *p_buffer, uint16_t buffer_size, uint8_t ch)
{
  uint8_t i;
  
  if (ch<' ') return 0;
  ch -= ' ';
  #ifdef my_debug
  Serial.print("ch= ");
  Serial.print(ch);
  Serial.print("= ");
  Serial.println(ch+' ');
  #endif
  
  uint16_t offset = 0;
  if (ch == 163)  // Check for accent value indicator
  {
    accent_flag = true; // set flag because an accent char comes with 2 bytes
  #ifdef my_debug
    Serial.print("accent_flag= ");
    Serial.println(accent_flag);
  #endif
    if (accent_flag == true) return 0;  // it's a ch with an accent: get the next ch
  }
  if (accent_flag == true)
  {
    accent_flag = false; // reset flag
  #ifdef my_debug
    Serial.print("accent_flag= ");
    Serial.println(accent_flag);
  #endif
    if (ch == 129)
      ch = 127; // index for á
    else if (ch == 135)
      ch = 128; // index for ç
    else if (ch == 141)
      ch = 129; // index for í
    else if (ch == 147)
      ch = 130; // index for ó
    else if (ch == 148)
      ch = 131; // index for ô
    else
      ch= 32; // set index for a space
    offset = 6*(ch-32);  // e.g. if ch is 127 then offset is 6*(127-32) = 6*95 = 570
  }
  else 
  {
    offset = 6*ch;
  }

  #ifdef my_debug
  Serial.print("ch after correction= ");
  Serial.println(ch);
  Serial.print("offset = ");
  Serial.println(offset);
  #endif
  uint8_t width = font_5x8[offset];
  for (i=0; i<width; i++) 
  { 
    offset += 1;
    // This is supposed to prevent buffer overflow.
    if (i>=buffer_size) break;
    p_buffer[i] = font_5x8[offset];
  }
  return i+1;
}

// Write a string to the buffer.
uint16_t led_matrix_puts(uint8_t *p_buffer, uint16_t buffer_size, uint8_t *p_str)
{
  uint8_t *p = p_buffer;
  while (*p_str!=0)
  {
    p += led_matrix_putch(p,buffer_size-(p-p_buffer),*p_str);
    p_str++;
  }
  return p - p_buffer;
}

uint32_t t_prev = 0;

void setup(void)
{
  Serial.begin(9600);
  while (!Serial);
  // Initialize LED matrix pins.
  for (uint8_t i=0; i<led_matrix_pin_count; i++)
  {
    pinMode(led_matrix_pin_first+i,INPUT); // all off
  }
  // Load text message.
  led_matrix_puts(led_matrix_buffer,sizeof(led_matrix_buffer),banner_text);
  // Ready...
  t_prev = millis();
  // Go!

  connectToWiFi();
  Serial.println("\nStarting connection to server...");
  Udp.begin(LOCAL_PORT);
  RTC.begin();

  // Get the current date and time from an NTP server and convert
  // it to UTC +2 by passing the time zone offset in hours.
  // You may change the time zone offset to your local one.
  auto unixTime = getUnixTime(1);
  Serial.print("Unix time = ");
  Serial.println(unixTime);
  RTCTime timeToSet = RTCTime(unixTime);
  RTC.setTime(timeToSet);

  // Retrieve the date and time from the RTC and print them
 
  RTC.getTime(currentTime); 
  Serial.println("The RTC was just set to: " + String(currentTime));
  fill_banner_txt(0, 521); // 
  // Refresh display.
  led_matrix_buffer_show(0, 521);
  pr_banner();
}

void clr_banner_txt()
{
  for (int i=initial_banner_text_len; i < BANNER_LEN; i++)  // Clear from after "  NTP Sync at: "
  {
    banner_text[i] = '\0';
  }
}

void clr_led_matrix_bfr()
{
  // See definition above: uint8_t led_matrix_buffer[5*32];
  for (int i=0; i < (5*BANNER_LEN); i++)
  {
    led_matrix_buffer[i]  = 0;
  }
}

void fill_banner_txt(uint8_t scroll, uint16_t ontime)
{
  for (int i=0; i < sizeof(currentTime); i++)
  {
    //if ( String(currentTime)[i] == '\0') break;
    banner_text[initial_banner_text_len + i] = String(currentTime)[i];
  }
  // Load text message.
  led_matrix_puts(led_matrix_buffer,sizeof(led_matrix_buffer),banner_text);
}

void pr_banner()
{
  int len2 = strlen((char*)banner_text);
  Serial.print("banner_text = \"");
  Serial.print(String((char*)banner_text));
  Serial.println("\"");
  Serial.print("length banner_text= ");
  Serial.print(len2);
  Serial.println(" characters.");
}

unsigned long s_time = millis(); // start time
unsigned long c_time = 0;  // current time
unsigned long d_time = 0;  // difference time
#define I_TM  300000  // interval time, about 5 minutes

void loop(){
  // Scroll speed is determined by both scroll_speed and ontime.
  const uint16_t scroll_speed = 100; // milliseconds
  const uint16_t ontime = 521; // microseconds. 521 (us) * 96 (pixels) = 50 ms frame rate if all the pixels are on.
  static uint8_t scroll = 0; // scroll position.

  bool new_time = false;
  c_time = millis();
  d_time = c_time - s_time; 
  /*
  Serial.print("loop(): I_TM= ");
  Serial.print(I_TM);
  Serial.print(", d_time= ");
  Serial.print(d_time);
  Serial.print(", c_time= ");
  Serial.print(c_time);
  Serial.print(", s_time= ");
  Serial.println(s_time);
  */
  if ((d_time  >= I_TM) && (scroll == 0)) // Eveery 5 minutes and only if the current text has been scrolled
  {
    s_time = c_time;
    // Retrieve the date and time from the RTC and print them
    RTC.getTime(currentTime);
    Serial.println("------------------------------------------");
    Serial.println("The RTC datetime: " + String(currentTime));
    clr_led_matrix_bfr();
    clr_banner_txt();
    fill_banner_txt(scroll, ontime); // fill the banner_text
    new_time = true; // set flag

    Serial.print("String(currentTime)= ");
    for (int i=0; i < sizeof(currentTime); i++)
    {
      char c = String(currentTime)[i];
      if (c == 0) break;
      Serial.print(c);
      //banner_text[i] = c; // String(currentTime)[i];  // Was: = (char)c;
    }
    Serial.println();

    if (new_time)
    {
      pr_banner();
      new_time = false;
    }
  }

  // Refresh display
  led_matrix_buffer_show(scroll,ontime);

  // Update scroll position.
  if (millis()>=t_prev+scroll_speed)
  {
    t_prev = millis();
    scroll += 1; // Scroll to the left.
    if (scroll>5*strlen((char*)banner_text)) scroll = 0; // restart
  }
}
