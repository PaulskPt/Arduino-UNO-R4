#define SECRET_SSID "<Your SSID here>"
#define SECRET_PASS "<Your Password here>"

#define REGION_EUROPE
// #define REGION_USA

#ifdef REGION_EUROPE // Europe/Lisbon
  #define SECRET_TZ_DST_ID "WEST"
  #define SECRET_TIMEZONE_DST_OFFSET "1"
  #define SECRET_TZ_STD_ID "WET"
  #define SECRET_TIMEZONE_STD_OFFSET "0"
#endif