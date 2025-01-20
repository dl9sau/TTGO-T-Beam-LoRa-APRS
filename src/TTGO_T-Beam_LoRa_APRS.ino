// Tracker for LoRA APRS
// from OE1ACM and OE3CJB redesigned by SQ9MDD .
// KISS ans Bluetooth by SQ5RWU
// TTGO T-Beam v1.0 only
//
// licensed under CC BY-NC-SA
// Includes
//#include <TTGO_T-Beam_LoRa_APRS_config.h> // to config user parameters
#include <Arduino.h>
#include <SPI.h>
#ifdef HAS_SX126X
    #include <RadioLib.h>
#elif HAS_SX127X
  #include <BG_RF95.h>         // library from OE1ACM
#endif
#include <math.h>
#include <driver/adc.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#ifdef HAS_TFT
  #include <TFT_eSPI.h>
#else
  #include <Adafruit_SSD1306.h>
  #include <Adafruit_GFX.h>
  #include <splash.h>
  #include <Adafruit_SPITFT.h>
  #include <Adafruit_SPITFT_Macros.h>
  #include <gfxfont.h>
#endif
#ifdef T_BEAM_V1_2
  #define XPOWERS_CHIP_AXP2101
  #include <XPowersLib.h>
#elif T_BEAM_V1_0
  #include <axp20x.h>
#endif
#include <esp_task_wdt.h>
#include <sys/time.h>
#include "preference_storage.h"
#include "syslog_log.h"
#include "ArduinoJson.h"
// Access to SPIFFS for wifi.cfg
#include "esp_spiffs.h"
#include "SPIFFS.h"
#include "FS.h" // SPIFFS is declared
#define FORMAT_SPIFFS_IF_FAILED true

// "brown out" (bad usb cable or insufficient usb power capacity):
// https://iotespresso.com/how-to-disable-brownout-detector-in-esp32-in-arduino/
// If you power your ESP32, open your Serial Monitor, and see the ESP32 rebooting continuously, with the following message: 'Brownout detector was triggered' Then you are probably providing insufficient power to the ESP32. It is generally observed in applications requiring higher power, like WiFi or BLE. The dictionary meaning of the term ‘brownout’ is
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include "version.h"
#include "taskGPS.h"
#ifdef KISS_PROTOCOL
  #include "taskTNC.h"
#endif
#ifdef ENABLE_WIFI
  #include "taskWebServer.h"
#endif

// Enable verbose debug output, level 2 on compile (-D DEVELOPMENT_DEBUG via plaformio.ini) or by finetuning it here.
// debug_verbose 1 currently affects syslog level LOG_DEBUG. 0 disables verbose output.
#ifdef DEVELOPMENT_DEBUG
int debug_verbose = 1;
#else
int debug_verbose = 0;
#endif

String wifi_info;                // saving wifi info (CLI|AP|dis) for Oled. If WIFI not compiled in, we still need this variable

String RemoteDebug;
uint32_t RemoteDebugNr = 0L;


// oled address
#define SSD1306_ADDRESS 0x3C

// SPI config
#ifdef HELTEC_WIRELESS_TRACKER
  #define SPI_sck 9
  #define SPI_miso 11
  #define SPI_mosi 10
  #define SPI_ss 8
  #define SPI_irq 14
  #define SPI_rst 12
  #define SPI_busy 13
#else
  #define SPI_sck 5
  #define SPI_miso 19
  #define SPI_mosi 27
  #define SPI_ss 18
  #if defined(T_BEAM_V1_2) && defined(HAS_SX126X)
    #define SPI_irq 33
    #define SPI_rst 23
    #define SPI_busy 32
  #else
    #define SPI_irq 26
  #endif
#endif

// IO config
#if defined(T_BEAM_V1_0) || defined(T_BEAM_V1_2)
  #define I2C_SDA 21
  #define I2C_SCL 22
  #define BUTTON  38                //pin number for Button on TTGO T-Beam
  #define BUZZER 15                 // enter your buzzer pin gpio
  const byte TXLED  = 4;            //pin number for LED on TX Tracker
#elif T_BEAM_V0_7
  #define I2C_SDA 21
  #define I2C_SCL 22
  #define BUTTON  39                //pin number for Button on TTGO T-Beam
  #define BATTERY_PIN 35
  #define BUZZER 15                 // enter your buzzer pin gpio
  const byte TXLED  = 4;            //pin number for LED on TX Tracker
/* Original LORA32 V2.1 Setup
#elif LORA32_21
  #define I2C_SDA 4
  #define I2C_SCL 15
  #define BUTTON 2                  //pin number for BUTTO
  #define BUZZER 13                 // enter your buzzer pin gpio
  const byte TXLED  = 4;            //pin number for LED on TX Tracker
*/
#elif LORA32_21                     // Modified as in #47
  #define I2C_SDA 21
  #define I2C_SCL 22
  #define BUTTON 2                  //pin number for BUTTO
  #define BATTERY_PIN 35
  #define BUZZER 13                 // enter your buzzer pin gpio
  const byte TXLED  = 4;            //pin number for LED on TX Tracker
#elif LORA32_2
  #define I2C_SDA 21
  #define I2C_SCL 22
  #define BUTTON 2                  //pin number for BUTTO
  #define BATTERY_PIN 35
  #define BUZZER 13                 // enter your buzzer pin gpio
  const byte TXLED  = 4;            //pin number for LED on TX Tracker
#elif LORA32_1
  #define I2C_SDA 21
  #define I2C_SCL 22
  #define BUTTON 2                  //pin number for BUTTO
  #define BATTERY_PIN 35
  #define BUZZER 13                 // enter your buzzer pin gpio
  const byte TXLED  = 4;            //pin number for LED on TX Tracker
#elif HELTEC_V1
  #define I2C_SDA 4
  #define I2C_SCL 15
  #define BUTTON 2                  //pin number for BUTTO
  #define BATTERY_PIN 35
  #define BUZZER 13                 // enter your buzzer pin gpio
  const byte TXLED  = 4;            //pin number for LED on TX Tracker
#elif HELTEC_V2
  #define I2C_SDA 4
  #define I2C_SCL 15
  #define BUTTON 2                  //pin number for BUTTO
  #define BATTERY_PIN 35
  #define BUZZER 13                 // enter your buzzer pin gpio
  const byte TXLED  = 4;            //pin number for LED on TX Tracker
#elif HELTEC_WIRELESS_TRACKER
  #define I2C_SDA 7                 // Not used by OLED. It has TFT
  #define I2C_SCL 6                 // Not used by OLED. It has TFT
  #define BUTTON 0                  //pin number for BUTTO
  #define BATTERY_PIN 1
  #define ADC_CTRL    2             // HELTEC Wireless Tracker ADC_CTRL = HIGH powers the voltage divider to read BatteryPin. Only on V05 = V1.1
  #define VEXT_CTRL   3             // this is for GPS and TFT screen on Wireless_Tracker and only for
  #define BUZZER 5                  // enter your buzzer pin gpio
  const byte TXLED  = 18;           //pin number for LED on TX Tracker. White LED
  #ifdef HAS_TFT
    #define TFT_BACKLIGHT  21           //pin number for TFT Backlight
  #endif
#else
  #define BATTERY_PIN 35
#endif

#ifdef HELTEC_WIRELESS_TRACKER
  #define TXLED_HIGH HIGH
  #define TXLED_LOW LOW
#else
  #define TXLED_HIGH LOW
  #define TXLED_LOW HIGH
#endif

// Variables for LoRa settings
#ifdef LORA_SPEED_1200
  ulong lora_speed = 1200;
#else
  ulong lora_speed = 300;
#endif

#ifdef TXFREQ
  double lora_freq = TXFREQ;
#else
  double lora_freq = 433.775;
#endif

ulong lora_speed_cross_digi = 1200;
double lora_freq_cross_digi = 433.900;

double lora_freq_rx_curr = lora_freq;
ulong lora_speed_rx_curr = lora_speed;

volatile boolean flag_lora_packet_available = false;

// Variables for WIFI APRS-IS connection. Requires ENABLE_WIFI
#ifdef ENABLE_WIFI
boolean aprsis_enabled = false;
String aprsis_host = "euro.aprs2.net";
uint16_t aprsis_port = 14580;
String aprsis_filter = "";
String aprsis_own_filters_in = "";
boolean aprsis_own_filter_in_is_whitelist = true;
String aprsis_own_filters_words_in = "";
String aprsis_callsign = "";
String aprsis_password = "-1";
uint8_t aprsis_data_allow_inet_to_rf = 0;  // 0: disable (default). 1: gate to main qrg. 2: gate to secondary qrg. 3: gate to both frequencies
extern void do_send_status_message_about_reboot_to_aprsis();
extern void do_send_status_message_about_shutdown_to_aprsis();
#else
boolean aprsis_enabled = false;
#endif

// Variables for APRS packaging
String Tcall;                       //your Call Sign for normal position reports
#if !defined(CALLSIGN)
#define CALLSIGN "N0CALL"
#endif
String aprsSymbolTable = APRS_SYMBOL_TABLE;
String aprsSymbol = APRS_SYMBOL;
String relay_path;
#ifdef APRS_OBJECT_NAME
  String aprsObjectName = APRS_OBJECT_NAME;
#else
  String aprsObjectName = "";
#endif
String aprsComment = MY_COMMENT;
double aprsLatPresetDouble = 0.0;
double aprsLonPresetDouble = 0.0;
String aprsLatPreset = LATITUDE_PRESET;
String aprsLonPreset = LONGITUDE_PRESET;
String aprsLatPresetDAO = LATITUDE_PRESET;
String aprsLonPresetDAO = LONGITUDE_PRESET;
String aprsLatPresetFromPreferences = LATITUDE_PRESET;
String aprsLonPresetFromPreferences = LONGITUDE_PRESET;
String aprsLatPresetNiceNotation;
String aprsLonPresetNiceNotation;
String aprsLatLonAsMaidenheadGridLocator;
String aprsLatLonDAO = "";
String aprsLatLonPresetCOMP = "";
boolean aprsLatLonInvalidPosition = true;
RawDegrees bestRawLat;
RawDegrees bestRawLng;
double bestDoubleLat;
double bestDoubleLng;
double bestHdop = 99.9;
boolean no_gps_position_since_boot = true;

int position_ambiguity = 0; // 0: default, compressed. -1: uncompressed. -2: uncompressed, with DAO '!W..!'. -3: uncompressed, with DAO '!w..!'. 1: ambiguity 1/10'. 2: ambiguity 1'. 3: ambiguity 10'. 4: Ambuguity 1 deg (60').
//String LatShownP = aprsLonPreset;
//String LongShownP = aprsLonPreset;
// "P" means original Preset, "u": Preset has been updated to the last valid position (DL3EL). "p" means, invalid gps, last known position used a temporary preset.
String aprsPresetShown = "P";
//double lastTxdistance = 0;

#if defined(T_BEAM_V1_2) || defined(T_BEAM_V1_0) || defined(T_BEAM_V0_7) || defined(HELTEC_WIRELESS_TRACKER) || defined(FORCE_ENABLE_GPS)
									         // ^ may used as compile time define
  boolean gps_state = true;
#else
  boolean gps_state = false;
#endif
TaskHandle_t xHandle_GPS;	// needed for being able to suspend/resume task when GPS sleeps
volatile boolean gps_task_enabled = false;
uint8_t gps_may_sleep = 1;      // gps may go to sleep: 1 only if we are battery powered. 2 always
uint32_t t_gps_fix_lost = 1L;
uint32_t t_gps_got_first_fix_after_last_loss = 0L;

// as we now collect the gps_data at the beginning of loop(), also the speed ist from there, we should not query gps.speed.kmph() directly later on
// the same show be done with course and alti (later)
// after successful data retrieval, gps_isValid becomes true (or turn false, if retrieval fails)
boolean gps_isValid = false;
int gps_speed = 0;
//int gps_speed_kmph_oled = 0;
char gps_time_s[20];// Room for len(01:02:03 04.05.2022) + 1 /* \0 */  -> 20

boolean fixed_beacon_enabled = false;
boolean show_cmt = true;
// Telemetry sequence, current value
int tel_sequence;
// Telemetry path
String tel_path;
uint8_t tel_allow_tx_on_rf = 0; // 0 is best ;)

#ifdef SHOW_ALT
  boolean showAltitude = true;  /* obsolete. use altitude_ratio 0 .. 100 % */
  uint8_t altitude_ratio = 100; // Recommended: 10%. May be 0 % speed (-> 100% altitude), 100 % speed (-> no altitude), or something in between
#else
  boolean showAltitude = false;  /* obsolete. use altitude_ratio 0 .. 100 % */
  uint8_t altitude_ratio = 0; // Recommended 10%. May be 0 % speed (-> 100% altitude), 100 % speed (-> no altitude), or something in between
#endif
boolean always_send_cseSpd_AND_altitude = false;
#ifdef SHOW_BATT
  boolean showBattery = true;
#else
  boolean showBattery = false;
#endif
uint8_t add_winlink_notification = 0;		// Periodically add notification to the WINLINK System. 0: disable. 1: only if bluetooth client is connected. 2: enable
uint32_t winlink_notified = 0L;
#ifdef ENABLE_TNC_SELF_TELEMETRY
  //boolean enable_tel = true;
  boolean enable_tel = false;
#else
  boolean enable_tel = false;
#endif
// Telemetry interval, seconds
#ifdef TNC_SELF_TELEMETRY_INTERVAL
  int tel_interval = TNC_SELF_TELEMETRY_INTERVAL;
#else
  int tel_interval = 3600;
#endif
#ifdef TNC_SELF_TELEMETRY_MIC
  int tel_mic = 1; // telemetry as "T#MIC"
#else
  //int tel_mic = 0; // telemetry as "T#001"  // numeric 0-9. Needs to remember sequence
  //int tel_mic = -1; // telemetry as "T#AbC" // time based alphanumeric 0-9,A-Z,a-z. cool, but aprs.fi complains. Resolution 23s in two months.
  int tel_mic = -2; // telemetry as "T#001"  // time based numeric 0-9. Resolution 10min in one week.
#endif
#if defined(ENABLE_BLUETOOTH)
  boolean enable_bluetooth = true;
  volatile boolean serial_bt_client_is_connected = false;
  // For handling state change of new bluetooth clients
  boolean serial_bt_client_was_connected = false;
  boolean new_serial_bt_client_connected = false;
#else
  boolean enable_bluetooth = false;
#endif
#if defined(ENABLE_WIFI)
  uint8_t enable_webserver = 2;
  boolean webserverStarted = false;
  boolean tncServer_enabled = false;
  boolean gpsServer_enabled = false;
  // Mapping Table {Power, max_tx_power} = {{8, 2}, {20, 5}, {28, 7}, {34, 8}, {44, 11}, {52, 13}, {56, 14}, {60, 15}, {66, 16}, {72, 18}, {80,20}}.
  // We'll use "min", "low", "mid", "high", "max" -> 2dBm (1.5mW) -> 8, 11dBm (12mW) -> 44, 15dBm (32mW) -> 60, 18dBm (63mW) ->72, 20dBm (100mW) ->80
  int8_t wifi_txpwr_mode_AP = 8;
  int8_t wifi_txpwr_mode_STA = 80;
  extern void refill_preferences_as_jsonData();
  extern void fill_wifi_config_as_jsonData();
#else
  void refill_preferences_as_jsonData() { ; };
  void fill_wifi_config_as_jsonData() { ; };
#endif
#ifdef ENABLE_PREFERENCES
String preferences_as_jsonData;
String wifi_config_as_jsonData;
#endif
#ifdef ENABLE_OLED
  boolean enabled_oled = true;
#else
  boolean enabled_oled = false;
#endif

// Variables and Constants
String outString="";                      //The new Output String with GPS Conversion RAW
String buildnr = "";
char compile_flags[128];

//Oled Display (DL3EL)
// to implement the blinking ticker, we have to save the content of the OLED. Also the preparation of output to OLED is done via these variables
// Lines in "normal" display (no rx or tx or keypress)
String OledHdr = "";      // Callsign
String OledLine1 = "";    // localtime + uptime
String OledLine2 = "";    // WebServer Info (CLI|AP|dis), next beacon (SB|FB), GPS Age (if > 2s)
String OledLine3 = "";    // Position
String OledLine4 = "";    // speed, course, altitude
String OledLine5 = "";    // sat info, batt info
boolean display_do_full_refresh = true;
int oled_line3and4_format = 0; // 0: original format of line3 and line4; Lat/Lon in aprs format. Alternative format: 1: classic. 2: nautical. 3: classic lat/lon left 4: nautical lat/lon left
int oled_show_locator = 0;    // 0: show always locator (and never lat/lon). 1: show never locator (and always Lat/lon). 2: 10:50 ratio. 6: with 20:100 ratio. 5: 20:20 ratio
int oled_loc_amb = 0;     // display locator not more precise than 0: RR99XX. -1: RR99XX99. -2: RR99XX99XX
#if defined(ENABLE_TNC_SELF_TELEMETRY)
  uint32_t nextTelemetryFrame = 60*1000L;  // first possible start of telemetry 60s after boot.
#endif

// cycle through the menu with by pressing the middle button multible times
int button_down_count = 0;
boolean button_key_up = true;
boolean button_t_lock = false;
ulong button_time_delay = 0;

// structure for LastHeard Array. Used for displaying course and distance to them.
#define MAX_LH 5                  // max lastheard
struct LastHeard {
  String callsign;
  uint32_t time_received = 0L;
  double lat;
  double lng;
  boolean direct;
};
struct LastHeard LH[MAX_LH];

struct struct_rx_raw_packet {
  String packet;
  char QRG;
};
struct struct_rx_raw_packet RX_RAW_PACKET_LIST[3];

String LastRXMessage = "";
String LastRXMessageSender = "";
String LastRXMessageTimeStr = "";
uint8_t LastRXMessageChanged = 0L;
uint8_t LastRXMessageInfo = 0;  // bitmask: 1 personal aprs text message. 2 winlink message. 3 both. => Display maker: 1: "m", 2: "w", 3: "M"

//byte Variables
byte  lora_TXStart;          //start of packet data in TXbuff
byte  lora_TXEnd;            //end of packet data in TXbuff
byte  lora_FTXOK;            //flag, set to 1 if TX OK
byte  lora_TXPacketType;     //type number of packet to send
byte  lora_TXDestination;    //destination address of packet to send
byte  lora_TXSource;         //source address of packet received
byte  lora_FDeviceError;     //flag, set to 1 if RFM98 device error
byte  lora_TXPacketL;        //length of packet to send, includes source, destination and packet type.

unsigned long lastPositionTX = 0L;
float BattVolts;
float InpVolts;

// variables for smart beaconing
#ifdef SB_ALGO_KENWOOD
  // Kenwood scales better on lower speed.
  ulong sb_min_interval = 120000L;
  ulong sb_max_interval = 1800000L;
  float sb_min_speed = 5;
  float sb_max_speed = 70;
  float sb_angle = 28;
#else
  ulong sb_min_interval = 60000L;
  ulong sb_max_interval = 360000L;
  float sb_min_speed = 0;
  float sb_max_speed = 30;
  float sb_angle = 30;                      // angle to send packet at smart beaconing
#endif
int sb_turn_slope = 26;			// kenwood example: 26 in mph. Yaesu suggests 26 in high speed car, 11 car in low/mid speed, 7 on walking;
					// TS 7 if <= 20km/h, TS 11 if <= 50km/h, TS 26 else.
int sb_turn_time = 30;			// min. 30s between transmissions (kenwood example)

float average_speed[5] = {0,0,0,0,0};
float average_speed_final = 0.0;
float old_course = 0, new_course = 0;
int point_avg_speed = 0, point_avg_course = 0;

ulong nextTX = 60000L;                  // preset time period between TX = 60000ms = 60secs = 1min

ulong time_to_refresh = 1000;            // typical time display lines are shown, before overwritten with new info
ulong next_fixed_beacon = 75000L;    // first fixed beacon approx 125s after system start (DL3EL)
ulong fix_beacon_interval = FIX_BEACON_INTERVAL;
ulong showRXTime = SHOW_RX_TIME;
ulong shutdown_delay = 0;
ulong shutdown_delay_time = 10000;
ulong shutdown_countdown_timer = 0;

boolean shutdown_active = true;
boolean shutdown_countdown_timer_enable = false;
boolean usb_status_before = false;
uint32_t reboot_interval = 0L;

// Variables required to Power Save OLED
// With "Display dimmer enabled" it will turn OLED off after some time
// if the checkbox is disabled the display stays OFF
uint oled_timeout = SHOW_OLED_TIME; // OLED Timeout
bool display_is_on = true; // Turn ON OLED at first startup
ulong oled_timer;
boolean freeze_display = false;

// Variable to manually send beacon from html page
uint8_t manBeacon = 0;  // 1: triggered from web-interface, 2: triggered from CLI

// Variable to show AP settings on OLED
#ifdef ENABLE_WIFI
int8_t WIFI_DISABLED = 0;
int8_t WIFI_SEARCHING_FOR_AP = 1;
int8_t WIFI_CONNECTED_TO_AP = 2;
int8_t WIFI_NOT_CONNECTED_TO_AP = 4;
int8_t WIFI_RUNNING_AS_AP = 8;
int8_t wifi_connection_status = WIFI_DISABLED;
int8_t wifi_connection_status_prev = -1;
String oled_wifi_SSID_curr = "";
String oled_wifi_PASS_curr = "";
String oled_wifi_IP_curr = "";
// needed here for SPIFFS WLAN Credentials:
String wifi_ModeAP_SSID;
String wifi_ModeAP_PASS;
String wifi_ModeSTA_SSID;
String wifi_ModeSTA_PASS;
// AP Array, currently max 10 APs possible
#define MAX_AP_CNT 10                  // max number of possible APs
struct AccessPoint APs[MAX_AP_CNT];
int apcnt = 0;
#endif

#define JSON_MAX_FILE_SIZE 2560
static StaticJsonDocument<JSON_MAX_FILE_SIZE> JSONBuffer;                         //Memory pool

#define ANGLE_AVGS 3                  // angle averaging - x times
float average_course[ANGLE_AVGS];
float avg_c_y, avg_c_x;

#ifdef RXDISABLE		// define RXDISABLE if you don't like to receive packets. Saves power consumption
  boolean lora_rx_enabled = false;
#else
  boolean lora_rx_enabled = true;
#endif

#ifdef TXDISABLE		// define TXDISABLE if you like to ensure that we never TX (i.e. if we are behind an rx-amplifier)
  boolean lora_tx_enabled = false;
  uint8_t txPower = 0;
  uint8_t txPower_cross_digi = 0;
#else
  boolean lora_tx_enabled = true;
  #ifdef TXdbmW
    uint8_t txPower = TXdbmW;
    uint8_t txPower_cross_digi = TXdbmW;
    #else
    uint8_t txPower = 23;
    uint8_t txPower_cross_digi = 23;
  #endif
#endif

uint16_t preambleLen = 8; // default tx preamble len
constexpr uint16_t preambleLen_default = 8; // default tx preamble len
constexpr uint16_t rxTimeoutSymbols = 1024; // extended rx timout to avoid rejecting packets with long preamble

#define UNITS_SPEED_KMH 1
#define UNITS_SPEED_MS  2
#define UNITS_SPEED_MPH 4
#define UNITS_SPEED_KN  8
#define UNITS_DIST_M    64
#define UNITS_DIST_FT   128

uint8_t units_speed = UNITS_SPEED_KMH;
uint8_t units_dist = UNITS_DIST_M;		// used for height

// may be configured
boolean rate_limit_message_text = true;		// ratelimit adding messate text (-> saves airtime)
boolean lora_automatic_cr_adaption = false;	// automatic CR adaption
						// We'll count users (except own digipeated packets), and if we're alone, CR4/8 doesn't disturb anyone.
						// If we have a high load on the channel, we'll decrease up to CR4/5.
						// You may set this to off if you are a fixed station / repeater / gateway
						// This may become set to true by default, after it proves it behaves good to our network
uint8_t lora_digipeating_mode = 1;		// Digipeating: 0: disabled (recommended if the device should not do repeating decisions, and even more, if you have attached a normal aprs digipeating software via kiss). 1: if own call addressed (recommended for users) 2: act as WIDE1 fill-in digi (recommended for standalone fill-in-digi). 3: act as a simple stupid WIDE2 digi
String lora_digipeating_myalias = "";		// My alias (can be addressed at first path element for digipeating)
uint8_t lora_cross_digipeating_mode = 0;	// 0: disable cross freq digipeating. 1: send on both frequencies. 2: send only on cross frequency
#define FLAG_ADD_SNR_RSSI_FOR_RF     1
#define FLAG_ADD_SNR_RSSI_FOR_KISS   2
#define FLAG_ADD_SNR_RSSI_FOR_APRSIS 4
#define FLAG_ADD_SNR_RSSI_FOR_RF__ONLY_IF_HEARD_DIRECT     8
#define FLAG_ADD_SNR_RSSI_FOR_KISS__ONLY_IF_HEARD_DIRECT   16
#define FLAG_ADD_SNR_RSSI_FOR_APRSIS__ONLY_IF_HEARD_DIRECT 32
uint8_t lora_add_snr_rssi_to_path = (FLAG_ADD_SNR_RSSI_FOR_KISS | FLAG_ADD_SNR_RSSI_FOR_APRSIS__ONLY_IF_HEARD_DIRECT);	// Add snr+rssi to path. May become default, after it proves it behaves good to our network
boolean kiss_add_snr_rssi_to_path_at_position_without_digippeated_flag = 1; // Add snr+rssi at last digipeater, without digipeated flag, at last position in path. Set to 1, if you pass data to aprs-is. Set to 0 if you pass data to your favourite digipeater software. We need this hack because our rssi-encoded data should not be interpreted as "(last ==) direct heard station" in the aprs-is net.
int tx_own_beacon_from_this_device_or_fromKiss__to_frequencies = 1;	// TX own beacon generated from this device or our beacon from from-kiss on following frequencies. Only if lora_digipeating_mode > 1 (we are a WIDE1 or WIDE2 digi). 1: main freq. 2: cross_digi_freq. 3: both frequencies. 5: special case for SP (allow sending on both frequencies, even if wie are not a WIDE digi; not recommended)
boolean tx_own_beacon_from_this_device_or_fromKiss__to_aprsis = true;	// TX own beacon generated from this device or our beacon from from-kiss to aprs-is.
int rx_on_frequencies = 1;			// RX freq. Only if lora_digipeating_mode < 2 (we are a user) 1: main freq. 2: cross_digi_freq. 3: both frequencies

bool acceptOwnPositionReportsViaKiss = true;		// true: Switches off local beacons as long as a kiss device is sending positions with our local callsign. false: filters out position packets with own callsign coming from kiss (-> do not send to LoRa).
boolean gps_allow_sleep_while_kiss = true;		// user has a kiss device attached via kiss which sends positions with own call, we don't need our gps to be turned on -> We pause sending positions by ourself (neither fixed nor smart beaconing). Except: user has a display attached to this tracker, he'll will be able to see his position because our gps does not go to sleep (-> set this to false). Why sleep? Energy saving
boolean wifi_do_fallback_to_mode_AP = true;		// Allow fallback to mode AP after once connected successfully connected (after boot) to configured remote AP. Disable for igates, where you don't need your tracker to be a hotspot. You like to enable, if you use your tracker portable and it should automatically be wifi client to your home network, and be AP if you are outside.
boolean send_status_message_to_aprsis = true;		// Send reboot, wifi- or internet-loss as APPRS-status-message to APRS-IS
boolean send_status_message_about_shutdown_to_rf = true;	// If you configured the auto shutdown feature when usb-power is lost (i.e. you use this devince in your car switched off your motor, then send a status message with "qrt"
uint8_t usb_serial_data_type = 0;		// 0: KISS. 1: Display some debug messages on serial port. 2: Display lora-received packets in TNC trace format. 3: 1+2. 4: Send GPS NMEA sentences.
						// If >0  usb-serial KISS-send and KISS-receive are stoped.
						// Unfortunately, 0 was pre-set for kiss. -> There's no real-off.
						// Because we test on set bits, number 128 (binary 10000000) means "off".

#ifdef KISS_PROTOCOL
// do not configure
uint32_t time_last_own_position_via_kiss_received = 0L;	// kiss client sends position report with our call+ssid. Remember when.
uint32_t time_last_frame_via_kiss_received = 0L;	// kiss client sends aprs-text-messages with our call+ssid. Remember when.
boolean kiss_client_came_via_bluetooth = false;
#endif

uint16_t adjust_cpuFreq_to = 80;
uint8_t units = UNITS_SPEED_KMH | UNITS_DIST_M;

// do not configure
boolean dont_send_own_position_packets = false;		// dynamicaly set if kiss device sends position. Maybe there are other usecases (-> kiss-independent)
boolean gps_state_before_autochange = false;		// remember gps state before autochange
uint32_t time_last_lora_frame_received_on_main_freq = 0L;
uint32_t time_last_own_text_message_via_kiss_received = 0L;
uint32_t time_lora_automaic_cr_adoption_rx_measurement_window = 0L;
uint32_t time_last_status_packet_sent = 0L;
uint16_t lora_automaic_cr_adoption_rf_transmissions_heard_in_timeslot = 0;
uint16_t lora_packets_received_in_timeslot_on_main_freq = 0;
uint16_t lora_packets_received_in_timeslot_on_secondary_freq = 0;
uint32_t stats_rx_freq_main = 0;
uint32_t stats_rx_freq_secondary = 0;
uint32_t stats_tx_freq_main = 0;
uint32_t stats_tx_freq_secondary = 0;
uint32_t stats_tx_beacon = 0;
uint32_t stats_tx_status = 0;
uint32_t stats_tx_telemetry = 0;
uint32_t stats_digipeated_rf2rf = 0;
uint32_t stats_from_aprsis = 0;
uint32_t stats_to_aprsis = 0;
uint32_t stats_from_serial = 0;
uint32_t stats_to_serial = 0;
uint32_t stats_from_kiss = 0;
uint32_t stats_to_kiss = 0;
boolean sendpacket_was_called_twice = false;
// bits for sendpacket()
#define SP_POS_FIXED 1
#define SP_POS_GPS   2
#define SP_ENFORCE_COURSE 4
uint8_t latlon_precision = 0; // 0: 0.01' (uncompressed default), >= 18.52m. 1: 0.001' >= 1.852m. 2: 0.0001' >= 18.52cm. Depends on position_ambiguity and is dynamically computed while parsing settings. do not change here. This is the precision of the coordinates (instead of just truncation the location strings)

String MY_APRS_DEST_IDENTIFYER = "APLOX1";

#ifdef ENABLE_WIFI
  tWebServerCfg webServerCfg;
  String to_aprsis_data = "";
#endif

static const adc_atten_t atten = ADC_ATTEN_DB_6;
static const adc_unit_t unit = ADC_UNIT_1;
#ifdef T_BEAM_V1_0
  AXP20X_Class axp;
#elif T_BEAM_V1_2
  XPowersAXP2101 axp;
#endif


// Singleton instance of the radio driver
//BG_RF95 rf95(18, 26);        // TTGO T-Beam has NSS @ Pin 18 and Interrupt IO @ Pin26
#ifdef HAS_SX127X
  BG_RF95 rf95(SPI_ss, SPI_irq);
  #define LORA_MAX_MESSAGE_LEN BG_RF95_MAX_MESSAGE_LEN
#elif HAS_SX126X
  SX1262 radio = new Module(SPI_ss, SPI_irq, SPI_rst, SPI_busy);
  #define LORA_MAX_MESSAGE_LEN 255-4 // 255 byte chip buffer - header
#else
  #define LORA_MAX_MESSAGE_LEN 255-4
#endif
char lora_TXBUFF_for_digipeating[LORA_MAX_MESSAGE_LEN+1] = "";		// buffer for digipeating
uint32_t time_lora_TXBUFF_for_digipeating_was_filled = 0L;

char blacklist_calls[256] = "";

#ifdef HAS_TFT
  TFT_eSPI tft = TFT_eSPI();
  #ifdef HELTEC_WIRELESS_TRACKER
    #define bigSizeFont     2
    #define smallSizeFont   1
    #define lineSpacing     9
  #endif

  #if !defined(WHITE)
    #define WHITE TFT_WHITE
  #endif

  #define OLED_LINE_LEN_MAX 25

  class Emulated_SSD1306 {
    public:
      //void dim(bool do_dim) { if (do_dim) tft.fillScreen(TFT_BLACK); }
      void dim(bool do_dim) { digitalWrite(TFT_BACKLIGHT, do_dim ? TXLED_LOW : TXLED_HIGH); }
      void clearDisplay(void) { tft.fillScreen(TFT_BLACK); }
      void setTextColor(int textColor) { tft.setTextColor(textColor, TFT_BLACK); }
      void setTextSize(uint8_t textSize) { tft.setTextSize(textSize < 2 ? smallSizeFont : bigSizeFont); }
      void setCursor(uint8_t x, uint8_t y) { tft.setCursor(x, y ? (y+2 / lineSpacing) +8 : 0); }
      void println(String &s) { tft.print(s); }
      void display(void) { return; }
  };
  Emulated_SSD1306 display;

  void do_tft_init() {
    tft.init();
    tft.begin();
    tft.setRotation(1);
    tft.setTextFont(0);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE,TFT_BLACK);
    tft.setTextSize(bigSizeFont);
    tft.setCursor(0, 0);
    display_do_full_refresh = true;
    display.dim(false);
  }

#else
  #ifdef HELTEC_WIRELESS_TRACKER
    #define OLED_RESET 21         // not used
  #else
    #define OLED_RESET 16         // not used
   #endif
  // initialize OLED display
  Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

  #define OLED_LINE_LEN_MAX 21
#endif


#ifdef IF_SEMAS_WOULD_WORK
  xSemaphoreHandle sema_lora_chip;
  xSemaphoreHandle sema_handle_aprs_message_addressed_to_us;
  xSemaphoreHandle sema_is_call_blacklisted;
#else
  volatile boolean sema_lora_chip = false;
  volatile boolean sema_handle_aprs_message_addressed_to_us = false;
  volatile boolean sema_is_call_blacklisted = false;
#endif


// + FUNCTIONS-----------------------------------------------------------+//

// This is a reimplementation of the buggy getLocalTime() library call of esp32
// which caused 5s delay on each call of getLocalTime(), if no gps- or ntp-
// time was obtained since boot. getLocalTime() has an optional argument to
// reduce this delay, but even then each call has a delay of 10ms.
// Also, posix recommends to call time with nullpointer instead of storing
// the time to the address of the argument. On linux, see man (2) time.
bool getLocalTimeTheBetterWay(struct tm * info)
{
  time_t now = time(0);
  if (now && now != ~((time_t ) 0)) {
    localtime_r(&now, info);
    if (info->tm_year > 99)
      return true;
  }
  return false;
}

void do_serial_println(const String &msg)
{
  if (usb_serial_data_type & 1) {
    Serial.println(msg);
  }
}

#ifdef HAS_SX126X
void signal_new_packet_received() {
  flag_lora_packet_available = true;
}
#endif

char *ax25_base91enc(char *s, uint8_t n, uint32_t v){
  /* Creates a Base-91 representation of the value in v in the string */
  /* pointed to by s, n-characters long. String length should be n+1. */
  for(s += n, *s = '\0'; n; n--)
  {
    *(--s) = v % 91 + 33;
    v /= 91;
  }
  return(s);
}


void store_compressed_position(double Tlat, double Tlon) {
    uint32_t aprs_lat = 900000000 - Tlat * 10000000;
    uint32_t aprs_lon = 900000000 + Tlon * 10000000 / 2;
    char helper_base91[] = {"0000\0"};
    String s;
    int i;

    if (position_ambiguity > 0) {
      // strip off n decimals
      i = (position_ambiguity > 4 ? 4 : position_ambiguity) -1;
      aprs_lat = (uint32_t ) (aprs_lat / (10000 * pow(10, i)) * 1000 * pow(10, i));
      aprs_lon = (uint32_t ) (aprs_lon / (10000 * pow(10, i)) * 1000 * pow(10, i));
    }
    aprs_lat = aprs_lat / 26 - aprs_lat / 2710 + aprs_lat / 15384615;
    aprs_lon = aprs_lon / 26 - aprs_lon / 2710 + aprs_lon / 15384615;

    ax25_base91enc(helper_base91, 4, aprs_lat);
    for (i = 0; i < 4; i++) {
      s += helper_base91[i];
    }
    ax25_base91enc(helper_base91, 4, aprs_lon);
    for (i = 0; i < 4; i++) {
      s += helper_base91[i];
    }
    aprsLatLonPresetCOMP = String(s);
}


void prepareAPRSFrame(uint8_t sp_flags) {
  static uint8_t cnt = 0;
  double curr_hdop = (gps.hdop.isValid() ? gps.hdop.hdop() : 99.9);
  double curr_kmph = (gps.speed.isValid() ? gps.speed.kmph() : 0.0);
  int curr_sats = gps.satellites.value();
  boolean may_add_dao_extension = (position_ambiguity <= -2 && latlon_precision > 0 && aprsLatLonDAO != "");
  boolean time_to_add_alt = false;
  // altitude_ratio: 0%, 10%, 25%, 50%, 75%, 90%, 100%
  // course change on turn has a heigher priority
  boolean altitude_isValid = (gps_state && gps.altitude.isValid() && gps.altitude.age() < 10000L && ((curr_hdop < 1.0 && curr_sats >= 5) || (curr_kmph > 16.0 && curr_sats >= 4)));
  boolean cseSpd_isValid = (gps_state && gps.speed.isValid() && (sp_flags & SP_ENFORCE_COURSE || (gps.speed.age() < 10000L && gps.course.isValid() && gps.course.age() < 10000L && ((curr_hdop < 1.5 && curr_sats >= 5) || (curr_kmph > 16.0 && curr_sats >= 4)))));
  int Tspeed = (cseSpd_isValid ? gps.speed.knots() : 0);
  int Tcourse = (cseSpd_isValid ? gps.course.deg() : -1);
  long Talt = (altitude_isValid ? gps.altitude.feet() : 0);
  long Talt_for_compression = Talt;

  cnt++;

  if (Tspeed < 0)
    Tspeed = 0;
  // for cse/spd. 0 deg is 360 deg. 0 marks invalid
  if (Tcourse < 0 || Tcourse > 360)
    Tcourse = 0;
  else if (Tcourse == 0)
    Tcourse = 360;
  if (Talt > 999999) Talt=999999;
  else if (Talt < -99999) Talt=-99999;

  if (altitude_isValid && altitude_ratio > 0) {
    if (altitude_ratio <= 50)
      time_to_add_alt = (cnt % (100 / altitude_ratio) == 0);
    else if (altitude_ratio < 100)
      time_to_add_alt = (cnt % (100 / (100-altitude_ratio)) != 0);
    else
      time_to_add_alt = true;
  }

  if (may_add_dao_extension) {
    const char *p = aprsComment.c_str();
    const char *q;
    while ((q = strchr(p, '!'))) {
      if (q-p == 4) {
         may_add_dao_extension = false; // already found a DAO field
         break;
      }
      p = q+1;
    }
  }

  outString = String(Tcall);
  outString += ">";
  outString += MY_APRS_DEST_IDENTIFYER;
  if (!relay_path.isEmpty()) {
    if (relay_path.length() < 3) {
      int ssid = relay_path.toInt();
      if (ssid < 0 || ssid > /* 15 // no, max hop 3 */ 3 || relay_path == "0")
        goto out_relay_path;
      if (ssid > 0) {
        if (ssid <= /* 15 // no, max hop 3 */ 3) {
          char buf[4];
          sprintf(buf, "-%d", ssid);
          outString += buf;
          goto out_relay_path;
        }
      } // else: relay_path.toInt("Q") or relay_path.toInt("QQ") is 0 (not -1 - wwtf ;) . Q and QQ is valid; fall through
    }
  }
  outString = outString + "," + relay_path;
out_relay_path:
  outString += ":";

  if (aprsObjectName.isEmpty()) {
    if (
#if defined(ENABLE_BLUETOOTH) && defined(KISS_PROTOCOL)
        serial_bt_client_is_connected ||
#endif
        (time_last_own_text_message_via_kiss_received &&
          ((time_last_own_text_message_via_kiss_received + 24*60*60*1000L) > millis()) )
       )
      outString += "=";
    else
      outString += "!";
  } else {
    // We can send our position as APRS objects. For those who love to see their tracker as "DL1AAA-L4"
    // on the map (which whould not be a valid src-call on RF)
    char buf[19]; // room for ";NameLen09*123456z" == 1 + 9 + 1 + 7 + \0 == 19
    char buf_time[8]; //room for 123456z + \0 == 8
    struct tm timeinfo;
    char *p;

    // Objects always have a length of 9 and a timestamp
    if (getLocalTimeTheBetterWay(&timeinfo)) {
      strftime(buf_time, sizeof(buf_time), "%H%M%Sz", &timeinfo);
    } else {
      strcpy(buf_time, "000000");
    }
    snprintf(buf, sizeof(buf), ";%-9.9s*%-7.7s", aprsObjectName.c_str(), buf_time);
    buf[sizeof(buf)-1] = 0;
    for (p = buf; *p; p++) {
      if (!isprint(*p & 0xff))
        *p = '_';
    }
    outString += String(buf);
  }

  if (aprsLatLonInvalidPosition) {
    // aprs spec: default null position.
    // The null position should be include the \. symbol (unknown/indeterminate position).
    outString += "0000.00N\\00000.00W.";
    may_add_dao_extension = false;
  } else if (position_ambiguity == 0 && aprsLatLonPresetCOMP.length()) {
    // position_ambiguity is not defined for compressed positions.
    // But we test it, in order to see what happens.
    // Else we could send in non-compressed mode, but will loose altitude or course/speed info.
    // This is ok, because if you like to hide your position for privacy reasons,
    // course/speed and altituded are also values you like to protect.
    //if (!(sp_flags & SP_POS_FIXED) && gps_state && gps_isValid &
         //(position_ambiguity < -4 || position_ambiguity >= 0) &&
         //aprsLatLonPresetCOMP.length()) {
    //if (position_ambiguity >= 0) {
    char helper_base91[] = {"0000\0"};

    outString += aprsSymbolTable;
    outString += aprsLatLonPresetCOMP;
    outString += aprsSymbol;

    // Due to spec, /A...... in message text could be used with compressed and uncompressed positions.
    // But if you encode altitude in compressed position: course/speed (i.e. 090/012) may not be added to message text
    // -> if we need (or wish to have always) course/speed, compression is always for speed, and altitude is part of the message text

    boolean may_send_alt_compressed = true;
    if (always_send_cseSpd_AND_altitude || !altitude_ratio) {
                                           // ^altitude ratio == 0 means 'never altitude'
         // ^always_send_cseSpd_AND_altitude means, we have to go to the compressed-speed-section, regardless of the alt-ratio
         // because altitude could be added to compressed speed, but not: speed to compressed altitude.
         // We go there even for thes special case of altitude_ratio == 100.
      may_send_alt_compressed = false;
    } else if (!time_to_add_alt && altitude_ratio < 100) {
                                 // ^altitude ratio == 100 means 'always altitude', except if always_send_cseSpd_AND_altitude is configured (checked 3 lines above)
                // ^no time to add altitude -> set may_send_alt_compressed to false
      may_send_alt_compressed = false;
    } else {
      if (altitude_ratio < 100) {
        if (!cseSpd_isValid && !altitude_isValid)
          may_send_alt_compressed = false;
        else if (cseSpd_isValid && (sp_flags & SP_ENFORCE_COURSE))
          may_send_alt_compressed = false;
        // else: time_to_add_alt is true -> keep may_send_alt_compressed true
      } // else: altitude ratio == 100 means 'always altitude'
    }

    if (may_send_alt_compressed) {

      if (altitude_isValid) {
        if (Talt_for_compression < 0) Talt_for_compression = 0;
        else if (Talt_for_compression > 15270967) Talt_for_compression = 15270967; /* 1.002** (90*91+90-1) */
        ax25_base91enc(helper_base91, 2, (uint32_t ) (log1p(Talt_for_compression) / 0.001998));
                                                                /* ^ math.log1p(1.002-1) */
        outString += helper_base91[0];
        outString += helper_base91[1];
      } else {
        outString += "  ";
      }
      outString += "X";

    } else {

      if (cseSpd_isValid) {
        ax25_base91enc(helper_base91, 1, (uint32_t ) (Tcourse == 360 ? 0 : Tcourse) / 4);
        outString += helper_base91[0];
        ax25_base91enc(helper_base91, 1, (uint32_t ) (log1p((Tspeed > 1018 ? 1018 : Tspeed)) / 0.07696));
                                                                     /* 1.08**90 */        /* ^ math.log1p(1.08-1) */
        outString += helper_base91[0];
      } else {
        outString += "  ";
      }
      outString += "H";

      if (time_to_add_alt) {
        char buf[7];
        outString += "/A=";
        sprintf(buf, "%06ld", Talt);
        outString += buf;
      }
    }

    may_add_dao_extension = false;

  } else {  // not compressed, i.e. fixed position

    if (position_ambiguity > 0) {
      char buf[10]; // room for 00000.00E + \0 == 10
      int n;
      int pos;
      // Only change mm.hh in dd[d]mm.hh, due to spec. Not degrees.
      n = position_ambiguity > 4 ? 4 : position_ambiguity;
      sprintf(buf, aprsLatPreset.c_str());
      for (pos = 6; pos > 1; pos--) {
        if (pos == 4) {
          // don't overwrite '.'
          continue;
        }
        buf[pos] = ' ';
        if (!(--n))
          break;
      }
      outString += String(buf);
      outString += aprsSymbolTable;
      sprintf(buf, aprsLonPreset.c_str());
      n = position_ambiguity > 4 ? 4 : position_ambiguity;
      for (pos = 7; pos > 2; pos--) {
        if (pos == 5) {
          // don't overwrite '.'
          continue;
        }
        buf[pos] = ' ';
        if (!(--n))
          break;
      }
      outString += String(buf);
      outString += aprsSymbol;
      may_add_dao_extension = false;
    } else {
      outString += (may_add_dao_extension ? aprsLatPresetDAO : aprsLatPreset);
      outString += aprsSymbolTable;
      outString += (may_add_dao_extension ? aprsLonPresetDAO : aprsLonPreset);
      outString += aprsSymbol;

      // course/speed and altitude only if position ambiguity <= 0
      if (cseSpd_isValid &&
           ( always_send_cseSpd_AND_altitude ||
             (!time_to_add_alt && altitude_ratio < 100) ||
             (sp_flags & SP_ENFORCE_COURSE) ) )  {
        char buf[8];
        sprintf(buf, "%.3d/%.3d", Tcourse, (Tspeed > 999 ? 999 : Tspeed));
        outString += String(buf);
      }

      if (altitude_isValid && (always_send_cseSpd_AND_altitude || time_to_add_alt) ) {
        char buf[7];
        outString += "/A=";
        sprintf(buf, "%06ld", Talt);
        outString += String(buf);
      }
    }

  }

  bool advertise_winlink = (add_winlink_notification && (add_winlink_notification == 2
#ifdef ENABLE_BLUETOOTH
                              || serial_bt_client_is_connected
#endif
         ) );

  if (show_cmt || advertise_winlink) {
    static uint8_t comments_added = 0;
    static uint32_t time_comment_added = 0L;
    if (!rate_limit_message_text) {
      comments_added = 0;
    } else {
      uint32_t t_offset = (gps_state ? sb_max_interval : fix_beacon_interval);
      // send comment text not under 10min, and at least every hour
      if (t_offset < 600000L)
        t_offset = 600000L;
      else if (t_offset > 3600000L)
        t_offset = 3600000L;
      if ((time_comment_added + t_offset) < millis())
        comments_added = 0;
    }
    if ((comments_added++ % 10) == 0) {
      if (show_cmt) {
        if (!aprsComment.isEmpty())
          outString += aprsComment;
        time_comment_added = millis();
      }
      if (advertise_winlink) {
        static uint32_t time_last_winlink_notifification_added = 0L;
        // Unfortunately, ".../A=xxxxxx/WINLINK/Thomas..." does not work. "/ WINLINK/" would work, but this is a nice notation.
        // That's why we add the "WINLINK" word at the end of comment text.
        if (time_last_winlink_notifification_added == 0L || time_last_winlink_notifification_added + 3600000L < millis()) {
          if (outString.indexOf("winlink") == -1) {
            if (!outString.isEmpty()) {
              if (!outString.endsWith(" ")) {
                if (!outString.endsWith(".") && !outString.endsWith(";") && !outString.endsWith(",") && !outString.endsWith("-")) {
                  outString += ".";
                }
                outString += " ";
              }
            }
            outString += "@winlink";
          }
          time_last_winlink_notifification_added = millis();
        }
      }
    }
  }

  if (showBattery) {
    if (BattVolts > 1.0) {
       if (!outString.endsWith(" "))
         outString += " ";
      outString += "Batt=";
      outString += String(BattVolts, 2);
      outString += ("V");
      #ifdef T_BEAM_V1_2
        outString = outString + "/" + String(axp.getBatteryPercent()) + "%";
        if (axp.isCharging()) {
          outString += "+"; // is charging -> indicate with "+"
        } else if  (!axp.isVbusIn()) {
          outString += "-"; // not charging and no vbus -> is discharging -> indicate with "-"
        }
      #endif
    }
    if (InpVolts > 1.0) {
       if (!outString.endsWith(" "))
         outString += " ";
      outString += "P=";
      outString += String(InpVolts, 2);
      outString += ("V");
    }
  }

  // finally, we may add DAO extension
  if (may_add_dao_extension) {
    if (!outString.endsWith(" "))
       outString += " ";
    outString += aprsLatLonDAO;
  }

  #ifdef KISS_PROTOCOL
    sendToTNC(outString);
  #endif
  if (usb_serial_data_type & 2) {
    Serial.println(outString);
    stats_to_serial++;
  }
}

#ifdef BUZZER
/**
 * Buzzer sound playback
 * @param melody - must be an array. Consisting of an even number of values. frequency and duration
 * @param array_size - number of elements in the array
 */
void buzzer(int* melody, int array_size){
  for(int i=0; i<array_size; i+=2){
      ledcWriteTone(0, *melody);
      melody++;
      delay(*melody);
      melody++;
  }
  ledcWriteTone(0,0); // turn off buzzer
}
#endif


void lora_set_speed(ulong lora_speed) {
#ifdef HAS_SX127X
  if(lora_speed==1200){
    rf95.setModemConfig(BG_RF95::Bw125Cr47Sf512);
  }
  else if(lora_speed==610){
    rf95.setModemConfig(BG_RF95::Bw125Cr48Sf1024);
  }
  else if(lora_speed==180){
    rf95.setModemConfig(BG_RF95::Bw125Cr48Sf4096);
  }
  else if(lora_speed==210){
    rf95.setModemConfig(BG_RF95::Bw125Cr47Sf4096);
  }
  else if(lora_speed==240){
    rf95.setModemConfig(BG_RF95::Bw125Cr46Sf4096);
  }
  else {
    rf95.setModemConfig(BG_RF95::Bw125Cr45Sf4096);
  }
#elif HAS_SX126X
  if (lora_speed == 1200) {
    radio.setCodingRate(7);
    radio.setSpreadingFactor(9);
  } else if (lora_speed == 610) {
    radio.setCodingRate(8);
    radio.setSpreadingFactor(10);
  } else if (lora_speed == 180) {
    radio.setCodingRate(8);
    radio.setSpreadingFactor(12);
  } else if (lora_speed == 210) {
    radio.setCodingRate(7);
    radio.setSpreadingFactor(12);
  } else if (lora_speed == 240) {
    radio.setCodingRate(6);
    radio.setSpreadingFactor(12);
  } else {
    radio.setCodingRate(6);
    radio.setSpreadingFactor(12);
  }
  radio.setBandwidth(125);
  radio.setCRC(true);
#endif
}

#if defined(ENABLE_WIFI)
void send_to_aprsis(const String &s)
{
  to_aprsis_data = s;
  return;
}
#endif


#if defined(ENABLE_WIFI)
// We separated sendpacket_to_aprsis() from sendpacket() due to the wish of a user:
// Messages to aprsis are faster than those sent to rf, received by another igate and sent from there to aprsis.
// But the user likes to examine RF-conditions (that means, who heard him on RF).
// Thus we send some of our packets with delay.
void send_own_beacon_to_aprsis_cached_or_now(const String &message) {
  static String cached_message;
  static uint32_t t_last_beacon = 0L;
  static uint8_t beacon_number = 0;

  if (!tx_own_beacon_from_this_device_or_fromKiss__to_aprsis) {
    return;
  }

  if (message != "") {
    if (cached_message == "")
      beacon_number++;
    if (!fixed_beacon_enabled && millis() - t_last_beacon < sb_max_interval) {
      // we are moving and have an own aprsis connection -> send immediately
      t_last_beacon = millis();
      send_to_aprsis(message);
      cached_message = "";
    } else {
      t_last_beacon = millis();
      cached_message = String(message); // make a local copy, and maybe overwrite old cached one
    }
  } else {
    if ((t_last_beacon + 180*1000L) < millis()) {
      // packet too old
      cached_message = "";
    }
  }

  if (cached_message == "") return;

  if ((beacon_number % 4) || ((t_last_beacon + 120*1000L) < millis())) {
    send_to_aprsis(cached_message);
    cached_message = "";
  }
}
#endif

#define LORA_FLAGS_NODELAY 1

void sendpacket(uint8_t sp_flags){
  if (sendpacket_was_called_twice)
    return;
  #ifdef BUZZER
    int melody[] = {1000, 50, 800, 100};
    buzzer(melody, sizeof(melody)/sizeof(int));
  #endif
  batt_read();
  prepareAPRSFrame(sp_flags);
  if (lora_tx_enabled && tx_own_beacon_from_this_device_or_fromKiss__to_frequencies) {
    if (tx_own_beacon_from_this_device_or_fromKiss__to_frequencies % 2) {
      loraSend(txPower, lora_freq, lora_speed, (sp_flags & SP_ENFORCE_COURSE) ? LORA_FLAGS_NODELAY : 0, outString);  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
      stats_tx_freq_main++;
    }
    if (((tx_own_beacon_from_this_device_or_fromKiss__to_frequencies > 1 && lora_digipeating_mode > 1) || tx_own_beacon_from_this_device_or_fromKiss__to_frequencies == 5) && lora_freq_cross_digi > 1.0 && lora_freq_cross_digi != lora_freq) {
      loraSend(txPower_cross_digi, lora_freq_cross_digi, lora_speed_cross_digi, (sp_flags & SP_ENFORCE_COURSE) ? LORA_FLAGS_NODELAY : 0, outString);  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
      stats_tx_freq_secondary++;
    }
  }
  stats_tx_beacon++;
  #if defined(ENABLE_WIFI)
    if (tx_own_beacon_from_this_device_or_fromKiss__to_aprsis)
      send_own_beacon_to_aprsis_cached_or_now(outString);
  #endif
  sendpacket_was_called_twice = true;
  lastPositionTX = millis();
  // reset timer for automatic fixed beacon after manual beacon
  next_fixed_beacon = millis() + fix_beacon_interval;
  nextTX = sb_max_interval;
}

void sendStatusPacket(const String &message) {
  outString = String(Tcall);
  outString += ">";
  outString += MY_APRS_DEST_IDENTIFYER;
  if (!relay_path.isEmpty()) {
    if (relay_path.length() < 3) {
      int ssid = relay_path.toInt();
      if (ssid < 0 || ssid > /* 15 // no, max hop 3 */ 3 || relay_path == "0")
        goto out_relay_path;
      if (ssid > 0) {
        if (ssid <= /* 15 // no, max hop 3 */ 3) {
          char buf[4];
          sprintf(buf, "-%d", ssid);
          outString += buf;
          goto out_relay_path;
        }
      } // else: relay_path.toInt("Q") or relay_path.toInt("QQ") is 0 (not -1 - wwtf ;) . Q and QQ is valid; fall through
    }
  }
  outString = outString + "," + relay_path;
out_relay_path:
  outString = outString + String(":>") + message;
  stats_tx_beacon++;
  enableOled_now(); // enable OLED
  writedisplaytext("((TXstat))", String("#") + String(stats_tx_status), outString, "", "", "");

  #ifdef KISS_PROTOCOL
    sendToTNC(outString);
  #endif
  if (usb_serial_data_type & 2) {
    Serial.println(outString);
    stats_to_serial++;
  }

  if (lora_tx_enabled && tx_own_beacon_from_this_device_or_fromKiss__to_frequencies) {
    if (tx_own_beacon_from_this_device_or_fromKiss__to_frequencies % 2) {
      loraSend(txPower, lora_freq, lora_speed, 0, outString);  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
      stats_tx_freq_main++;
    }
    if (((tx_own_beacon_from_this_device_or_fromKiss__to_frequencies > 1 && lora_digipeating_mode > 1) || tx_own_beacon_from_this_device_or_fromKiss__to_frequencies == 5) && lora_freq_cross_digi > 1.0 && lora_freq_cross_digi != lora_freq) {
      loraSend(txPower_cross_digi, lora_freq_cross_digi, lora_speed_cross_digi, 0, outString);  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
      stats_tx_freq_secondary++;
    }
  }
  time_last_status_packet_sent = millis();

  #if defined(ENABLE_WIFI)
    if (tx_own_beacon_from_this_device_or_fromKiss__to_aprsis)
      send_to_aprsis(outString);
  #endif
}


#ifdef IF_SEMAS_WOULD_WORK
void sema_lock_or_unlock__common(xSemaphoreHandle *sema, int acquire_lock) {
#else
void sema_lock_or_unlock__common(volatile boolean *sema, int acquire_lock) {
#endif
  if (acquire_lock) {
    #ifdef IF_SEMAS_WOULD_WORK
      int n = 0;
      while (xSemaphoreTake(*sema, 0) != pdTRUE) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if (!(n % 100))
          esp_task_wdt_reset();
        if (n++ > 30000)
          ESP.restart();
      }
    #else
      for (int n = 0; *sema; n++) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if (!(n % 100))
          esp_task_wdt_reset();
        if (n > 30000)
          ESP.restart();
      }
      *sema = true;
    #endif
  } else {
    #ifdef IF_SEMAS_WOULD_WORK
      xSemaphoreGive(*sema);
    #else
      *sema = false;
    #endif
  }
}

void sema_lock_or_unlock__lora(int acquire_lock) {
  return sema_lock_or_unlock__common(&sema_lora_chip, acquire_lock);
}
void sema_lock_or_unlock__handle_aprs_message_addressed_to_us(int acquire_lock) {
  return sema_lock_or_unlock__common(&sema_handle_aprs_message_addressed_to_us, acquire_lock);
}
void sema_lock_or_unlock__is_call_blacklisted(int acquire_lock) {
  return sema_lock_or_unlock__common(&sema_is_call_blacklisted, acquire_lock);
}


/**
 * Send message as APRS LoRa packet
 * @param lora_LTXPower
 * @param lora_FREQ
 * @param lora_SPEED
 * @param flags
 * @param message
 */


void loraSend(byte lora_LTXPower, float lora_FREQ, ulong lora_SPEED, uint8_t flags, const String &message) {

  if (!lora_tx_enabled)
    return;

  // kind of csma/cd. TODO: better approach: add to a tx-queue
  // in SF12: preamble + lora header = 663.552 -> we wait 1300ms for check if lora-chip is in decoding
  // See https://www.rfwireless-world.com/calculators/LoRaWAN-Airtime-calculator.html
  // In detail:       At our supported speed "names":
  // SF7: 28.928ms
  // SF8: 57.856ms
  // SF9: 115.712ms   1200
  // SF10: 231.424ms  610
  // SF11: 331.776ms
  // SF12: 663.552ms  300,240, 210, 180
  uint32_t wait_for_signal = 700;
  if (lora_speed  == 610) wait_for_signal = 250;
  else if (lora_speed  == 1200) wait_for_signal = 125;

  // sema lock for lora chip operations
  sema_lock_or_unlock__lora(1);

#ifdef T_BEAM_V1_0
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);                           // switch LoRa chip on
#elif T_BEAM_V1_2
  axp.setALDO2Voltage(3300);
  axp.enableALDO2();                                                    // switch LoRa chip on
#endif
#if HAS_SX126X
  radio.standby(); // wake up if was set to sleep()
#endif

  randomSeed(millis());

  // If lora chip is busy by still sending a packet, we'll get an exception
  // on rf95 commands like rf95.setFrequency(). Assure the chip is available
  // before any access to it

  for (int n = 0; n < 30; n++) {
    delay(wait_for_signal);
    if (n == 1 && (flags & LORA_FLAGS_NODELAY)) {
      // send without delay (on turn), we may wait one round for checking ifg channel is free
      break;
    }
    esp_task_wdt_reset();
    #ifdef HAS_SX127X
      if (rf95.SignalDetected()) {
        continue;
      }
    #endif
    vTaskDelay(100 / portTICK_PERIOD_MS);
    #ifdef HAS_SX127X
      if (!rf95.SignalDetected() && random(256) < 64) break;
    #else
      if (random(256) < 64) break;
    #endif
  }
  esp_task_wdt_reset();

  #ifdef ENABLE_LED_SIGNALING
    digitalWrite(TXLED, TXLED_HIGH);
  #endif

  lora_set_speed(lora_SPEED);
  #ifdef HAS_SX127X
    //byte array
    byte  lora_TXBUFF[LORA_MAX_MESSAGE_LEN];      //buffer for packet to send
    int messageSize = min(message.length(), sizeof(lora_TXBUFF) - 1);
    message.toCharArray((char*)lora_TXBUFF, messageSize + 1, 0);

    rf95.setFrequency(lora_FREQ);
    rf95.setTxPower(lora_LTXPower);
    rf95.setPreambleLength(lora_FREQ == lora_freq ? preambleLen : preambleLen_default);
    esp_task_wdt_reset();
    rf95.sendAPRS(lora_TXBUFF, messageSize);
    rf95.waitPacketSent();
  #elif HAS_SX126X
    radio.setFrequency(lora_FREQ);
    radio.setOutputPower(lora_LTXPower);
    radio.setPreambleLength(lora_FREQ == lora_freq ? preambleLen : preambleLen_default);
    radio.setCurrentLimit(140);
    radio.transmit("\x3c\xff\x01" + message);
  #endif
  esp_task_wdt_reset();


  if (lora_SPEED != lora_speed_rx_curr || lora_FREQ != lora_freq_rx_curr) {
    // cross-digipeating may have altered our LoRa Speed and RX-frequency. Revert frequency change needed for this transmission.
    if (lora_SPEED != lora_speed_rx_curr) {
      lora_set_speed(lora_speed_rx_curr);
    }
    if (lora_FREQ != lora_freq_rx_curr) {
      #ifdef HAS_SX127X
        // flush cache. just to be sure, so that no cross-digi-qrg packet comes in the input-buffer of the main qrg.
        // With no buffer / length called, recvAPRS directly calls clearRxBuf()
        rf95.recvAPRS(0, 0);
        rf95.setFrequency(lora_freq_rx_curr);
      #elif HAS_SX126X
        // if packet is in queue, throw away
        if (flag_lora_packet_available) {
          String tmp_buf;
          radio.readData(tmp_buf);
        }
        radio.setFrequency(lora_freq_rx_curr);
      #endif
    }
  }
  flag_lora_packet_available = false;
  // setting rx TO, default to allow rx of long preamble packets
  #ifdef HAS_SX127X
    rf95.setPreambleLength(rxTimeoutSymbols);
  #elif HAS_SX126X
    radio.setPreambleLength(rxTimeoutSymbols);
    radio.setRxBoostedGainMode(true);
  #endif

  #ifdef ENABLE_LED_SIGNALING
    digitalWrite(TXLED, TXLED_LOW);
  #endif

  // if lora_rx is disabled AND lora_digipeating_mode == 0 AND no SerialBT.hasClient is connected,
  // we can savely go to sleep
  if ( !lora_rx_enabled && lora_digipeating_mode == 0
          #if defined(ENABLE_BLUETOOTH) && defined(KISS_PROTOCOL)
            && (!enable_bluetooth || serial_bt_client_is_connected)
          #endif
      ) {
    #ifdef HAS_SX126X
      radio.sleep();
    #endif
    #ifdef T_BEAM_V1_0
      axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF);                           // switch LoRa chip off
    #elif T_BEAM_V1_2
      axp.disableALDO2();                                                    // switch LoRa chip off
    #endif
  } else {
    #ifdef HAS_SX126X
      radio.standby();
      radio.startReceive();
    #endif
  }

  // release lock
  esp_task_wdt_reset();
  sema_lock_or_unlock__lora(0);

}


void batt_read(){
  static float InpVolts_measures[5] = { 4.5, 4.5, 4.5, 4.5, 4.5 };
  static int InpVolts_measures_pos = 0.0;

#if defined(T_BEAM_V1_0) || defined(T_BEAM_V1_2)
  BattVolts = ((float ) axp.getBattVoltage())/1000.0;
  InpVolts = ((float ) axp.getVbusVoltage())/1000.0;
#elif T_BEAM_V0_7
  InpVolts = (((float)analogRead(35) / 8192.0) * 2.0 * 3.3 * (1100.0 / 1000.0))+0.41;    // fixed thanks to Luca IU2FRL
  //InpVolts = adc1_get_raw(ADC1_CHANNEL_7)/1000;
#elif HELTEC_WIRELESS_TRACKER
  digitalWrite(ADC_CTRL, HIGH);
  //InpVolts = analogRead(BATTERY_PIN)*4.9/1000.0; // Values from the data sheet.
  // ..but the result seems to high. This approach is from richonguzman
  int adc_value = analogRead(BATTERY_PIN);
  digitalWrite(ADC_CTRL, LOW);
  double voltage = (adc_value * 3.3 ) / 4095.0;
  double inputDivider = (1.0 / (390.0 + 100.0)) * 100.0;  // The voltage divider is a 390k + 100k resistor in series, 100k on the low side.
  InpVolts = (voltage / inputDivider) + 0.285; // Yes, this offset is excessive, but the ADC on the ESP32s3 is quite inaccurate and noisy. Adjust to own measurements.
#else
  InpVolts = analogRead(BATTERY_PIN)*7.221/4096;
#endif


  InpVolts_measures[InpVolts_measures_pos] = InpVolts;
  InpVolts_measures_pos = (InpVolts_measures_pos + 1) % 5;
  InpVolts = (InpVolts_measures[0]+InpVolts_measures[1]+InpVolts_measures[2]+InpVolts_measures[3]+InpVolts_measures[4])/5;

}


void setup_oled_timer_values() {
  // Hold the OLED ON at first boot (or duriing soft_reconfiguration)
  // oled_timeout == 0 is a special case for 'always on'.
  // If user switches off OLED (enabled_oled == false), but sets oled_timeout to 0 (always on),
  // we add SHOW_OLED_TIME to timeout (instead of 0), for keep it running for 15s after setup();
  // if enabled_oled is true and oled_timeout is 0, this does not harm.
  oled_timer = millis() + (oled_timeout ? oled_timeout : SHOW_OLED_TIME);
  time_to_refresh = millis() + showRXTime;
}

void enableOled() {
  if (!enabled_oled)
    return;
  #if defined(HELTEC_WIRELESS_TRACKER) && defined(VEXT_CTRL) && defined(HAS_TFT)
    if (gps_state && !gps_task_enabled)
      return;
  #endif
  // This function enables OLED display after pressing a button
  oled_timer = millis() + oled_timeout;
}

void enableOled_now() {
  if (!enabled_oled)
    return;
  #if defined(HELTEC_WIRELESS_TRACKER) && defined(VEXT_CTRL) && defined(HAS_TFT)
    if (gps_state && !gps_task_enabled)
      return;
  #endif
  if (!display_is_on) {
    display.dim(false);
    display_is_on = true;
  }
  oled_timer = millis() + oled_timeout;
}

void writedisplaytext(String HeaderTxt, String Line1, String Line2, String Line3, String Line4, String Line5) {
  // some assurances
  // Line1 has a larger font
  if (Line1.length() > OLED_LINE_LEN_MAX) Line1.remove(OLED_LINE_LEN_MAX, Line1.length()-OLED_LINE_LEN_MAX);
  // line 2 can grow in some cases. Then the other lines are empty. TODO: algorithmic approach
  if (Line2.length() > OLED_LINE_LEN_MAX && Line3.length() == 0 && Line4.length() == 0 && Line5.length() == 0) {
      if (Line2.length() > OLED_LINE_LEN_MAX*4)
        Line2.remove(OLED_LINE_LEN_MAX*4, Line2.length()-OLED_LINE_LEN_MAX*4);
  } else {
    if (Line2.length() > OLED_LINE_LEN_MAX) Line2.remove(OLED_LINE_LEN_MAX, Line2.length()-OLED_LINE_LEN_MAX);
    if (Line3.length() > OLED_LINE_LEN_MAX) Line3.remove(OLED_LINE_LEN_MAX, Line3.length()-OLED_LINE_LEN_MAX);
    if (Line4.length() > OLED_LINE_LEN_MAX) Line4.remove(OLED_LINE_LEN_MAX, Line4.length()-OLED_LINE_LEN_MAX);
    if (Line5.length() > OLED_LINE_LEN_MAX) Line5.remove(OLED_LINE_LEN_MAX, Line5.length()-OLED_LINE_LEN_MAX);
  }

#ifdef DEVELOPMENT_DEBUG
  if (debug_verbose > 1) {
    if (HeaderTxt.length() > OLED_LINE_LEN_MAX) {
      Serial.printf("HeaderTxt: %s (%d)\r\n", HeaderTxt.c_str(), HeaderTxt.length());
    }
    if (Line1.length() > OLED_LINE_LEN_MAX) {
      Serial.printf("Line1: %s (%d)\r\n", Line1.c_str(), Line1.length());
    }
    if (Line2.length() > OLED_LINE_LEN_MAX) {
      Serial.printf("Line2: %s (%d)\r\n", Line2.c_str(), Line2.length());
    }
    if (Line3.length() > OLED_LINE_LEN_MAX) {
      Serial.printf("Line3: %s (%d)\r\n", Line3.c_str(), Line3.length());
    }
    if (Line4.length() > OLED_LINE_LEN_MAX) {
      Serial.printf("Line4: %s (%d)\r\n", Line4.c_str(), Line4.length());
    }
    if (Line5.length() > OLED_LINE_LEN_MAX) {
      Serial.printf("Line5: %s (%d)\r\n", Line5.c_str(), Line5.length());
    }
  }
#endif
  OledHdr = HeaderTxt;
  OledLine1 = Line1;
  OledLine2 = Line2;
  OledLine3 = Line3;
  OledLine4 = Line4;
  OledLine5 = Line5;
  if (display_is_on) {
    write2display();
#ifdef DEVELOPMENT_DEBUG
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(0,0);
    display.println(HeaderTxt);
    display.setTextSize(1);
    display.setCursor(0,16);
    display.println(Line1);
    display.setCursor(0,26);
    display.println(Line2);
    display.setCursor(0,36);
    display.println(Line3);
    display.setCursor(0,46);
    display.println(Line4);
    display.setCursor(0,56);
    display.println(Line5);
    display.display();
#endif
  }

  // if oled is continuosly on (or currently on), refresh every second (DL3EL)
  if (oled_timeout == 0 || display_is_on) {
    // refresh display once a second
    time_to_refresh = millis() + 1000;
  } else {
    time_to_refresh = millis() + showRXTime;
  }
  // save last display output for blinker
#ifdef DEVELOPMENT_DEBUG
  OledHdr = HeaderTxt;
  OledLine1 = Line1;
  OledLine2 = Line2;
  OledLine3 = Line3;
  OledLine4 = Line4;
  OledLine5 = Line5;
#endif
}

void timer_once_a_second() {
  static uint32_t t_next_run = 0L;
  struct tm timeinfo;

  if (millis() < t_next_run)
    return;

  t_next_run = millis() + 1000;

  batt_read();

  // update gps time string once a second
  if (getLocalTimeTheBetterWay(&timeinfo)) {
    strftime(gps_time_s, sizeof(gps_time_s), "%H:%M:%S", &timeinfo);
  }

#ifdef ENABLE_BLUETOOTH
  if (enable_bluetooth) {
    serial_bt_client_is_connected = SerialBT.hasClient();
    new_serial_bt_client_connected = false;
    if (serial_bt_client_was_connected) {
      if (!serial_bt_client_is_connected) {
        serial_bt_client_was_connected = false;
      }
    } else {
      if (serial_bt_client_is_connected) {
        serial_bt_client_was_connected = true;
        new_serial_bt_client_connected = true;
      }
    }
  } else {
    serial_bt_client_is_connected = false;
  }
#endif

  // only when OLED is on
  if (display_is_on) {
    // refresh display once a second
    if (!freeze_display)
      fillDisplayLine1(0); //update time & uptime

#ifdef notdef  // Unfortunately destroys line3-5 of currently displayed ((RX)) packet
    // some assurances
    // Line1 has a larger font
    if (OledLine1.length() > OLED_LINE_LEN_MAX) OledLine1.remove(OLED_LINE_LEN_MAX, OledLine1.length()-OLED_LINE_LEN_MAX);
    // line 2 can grow in some cases. Then the other lines are empty. TODO: algorithmic approach
    if (OledLine2.length() > OLED_LINE_LEN_MAX && OledLine3.length() == 0 && OledLine4.length() == 0 && OledLine5.length() == 0) {
      if (OledLine2.length() > OLED_LINE_LEN_MAX*4)
        OledLine2.remove(OLED_LINE_LEN_MAX*4, OledLine2.length()-OLED_LINE_LEN_MAX*4);
    } else {
      if (OledLine2.length() > OLED_LINE_LEN_MAX) OledLine2.remove(OLED_LINE_LEN_MAX, OledLine2.length()-OLED_LINE_LEN_MAX);
      if (OledLine3.length() > OLED_LINE_LEN_MAX) OledLine3.remove(OLED_LINE_LEN_MAX, OledLine3.length()-OLED_LINE_LEN_MAX);
      if (OledLine4.length() > OLED_LINE_LEN_MAX) OledLine4.remove(OLED_LINE_LEN_MAX, OledLine4.length()-OLED_LINE_LEN_MAX);
      if (OledLine5.length() > OLED_LINE_LEN_MAX) OledLine5.remove(OLED_LINE_LEN_MAX, OledLine5.length()-OLED_LINE_LEN_MAX);
    }

#ifdef DEVELOPMENT_DEBUG
    if (debug_verbose > 1) {
      if (OledHdr.length() > OLED_LINE_LEN_MAX) {
        Serial.printf("OledHdr [timer_once_a_second]: %s (%d)\r\n", OledHdr.c_str(), OledHdr.length());
      }
      if (OledLine1.length() > OLED_LINE_LEN_MAX) {
        Serial.printf("OledLine1 [timer_once_a_second]: %s (%d)\r\n", OledLine1.c_str(), OledLine1.length());
      }
      if (OledLine2.length() > OLED_LINE_LEN_MAX) {
        Serial.printf("OledLine2 [timer_once_a_second]: %s (%d)\r\n", OledLine2.c_str(), OledLine2.length());
      }
      if (OledLine3.length() > OLED_LINE_LEN_MAX) {
        Serial.printf("OledLine3 [timer_once_a_second]: %s (%d)\r\n", OledLine3.c_str(), OledLine3.length());
      }
      if (OledLine4.length() > OLED_LINE_LEN_MAX) {
        Serial.printf("OledLine4 [timer_once_a_second]: %s (%d)\r\n", OledLine4.c_str(), OledLine4.length());
      }
      if (OledLine5.length() > OLED_LINE_LEN_MAX) {
        Serial.printf("OledLine5 [timer_once_a_second]: %s (%d)\r\n", OledLine5.c_str(), OledLine5.length());
      }
    }
#endif
    write2display();
#ifdef DEVELOPMENT_DEBUG
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(0,0);
    display.println(OledHdr);
    display.setTextSize(1);
    display.setCursor(0,16);
    display.println(OledLine1);
    display.setCursor(0,26);
    display.println(OledLine2);
    display.setCursor(0,36);
    display.println(OledLine3);
    display.setCursor(0,46);
    display.println(OledLine4);
    display.setCursor(0,56);
    display.println(OledLine5);
    display.display();
#endif
#endif
  }
}

String display_cache_lines[6];

int write2display_cached(int line_number) {
  String OledLineCurr = "";
  int lines_modified = 0;
  if (line_number == 0)
    OledLineCurr = OledHdr;
  else if (line_number == 1)
    OledLineCurr = OledLine1;
  else if (line_number == 2)
    OledLineCurr = OledLine2;
  else if (line_number == 3)
    OledLineCurr = OledLine3;
  else if (line_number == 4)
    OledLineCurr = OledLine4;
  else if (line_number == 5)
    OledLineCurr = OledLine5;

  #ifdef HAS_TFT
    if (line_number > 5)
      return lines_modified;

    // Nothing has changed? Good ;)
    if (display_cache_lines[line_number] == OledLineCurr)
      return lines_modified;

    String S = OledLineCurr;
    // make a local copy before any modification
    display_cache_lines[line_number] = String(OledLineCurr);
    // Modify chaced lines copy instead of the original OledLineX
    display.setTextColor(WHITE);

    do {
      display.setTextSize(line_number == 0 ? 2 : 1);

      // TFT cannot handle long lines poperly. We have to cut them
      display.setCursor(0, line_number == 0 ? 0 : 6+10*line_number);
      String Snext = String(S);
      Snext.remove(0, (line_number ? OLED_LINE_LEN_MAX : OLED_LINE_LEN_MAX/2+1));
      if (S.length() > (line_number ? OLED_LINE_LEN_MAX : OLED_LINE_LEN_MAX/2+1))  {
        S.remove(OLED_LINE_LEN_MAX, S.length()-OLED_LINE_LEN_MAX);
      } else {
        // fill rest of line with blanks, else TFT will keep them :(
        for (int i = S.length(); i < (line_number ? OLED_LINE_LEN_MAX : OLED_LINE_LEN_MAX/2+1); i++)
          S +=  " ";
      }
      display.println(S);
//Serial.println("display >" + S + "<");
      S = Snext;
      line_number++;
      lines_modified++;
      if (!Snext.isEmpty() && line_number < 6) {
        // Mark next cache entry with " ", so the split lines may be updated if needed
        display_cache_lines[line_number] = " ";
      }
    } while (!S.isEmpty() && line_number < 6);

  #else
    // OLED allows no caching because we alwayas need to call display.display()
    display.setTextColor(WHITE);
    display.setTextSize(line_number == 0 ? 2 : 1);
    display.setCursor(0, line_number == 0 ? 0 : 6+10*line_number);
    display.println(OledLineCurr);
    lines_modified++;
  #endif

  return lines_modified;
}

void write2display() {
    // Only update changed lines. Avoids flickering.
    // do a full refresh i.e. if display was powered off
    boolean line_changed = false;
    #ifdef HAS_TFT
      if (display_do_full_refresh) {
        for (int i = 0; i < 6; i++)
          display_cache_lines[i] = "";
        line_changed = true;
        display_do_full_refresh = false;
      }
    #else // with oled, always call clearDisplay() -> no caching possible
      display.clearDisplay();
    #endif
    for (int i = 0; i < 6; i++) {
      int n;
      if ((n = write2display_cached(i))) {
        line_changed = true;
        if (n > 1)
          i = i+n-1;
      }
    }
    if (line_changed)
      display.display();
}


char *str_course(int resolution) {
  static char buf[7]; // room for 359.9G + \0
  if (gps_state && gps_isValid) {
    // sprintf rounds
    double cse = gps.course.deg();
    if (resolution)
      if (oled_line3and4_format == 2 || oled_line3and4_format == 4) {
        sprintf(buf, "%03.1f\xF7", cse > 359.949 ? 0 : cse);
      } else {
        sprintf(buf, "%3.1f\xF7", cse > 359.949 ? 0 : cse);
      }
    else {
      if (oled_line3and4_format == 2 || oled_line3and4_format == 4) {
        sprintf(buf, "%03.0f\xF7", cse > 359.49 ? 0 : cse);
      } else {
        sprintf(buf, "%3.0f\xF7", cse > 359.49 ? 0 : cse);
      }
    }
  } else {
    sprintf(buf, "--\xF7");
  }
  return buf;
}

char *str_speed() {
  static char buf[8]; // room for 999km/h + \0 == 8
  const char *speed_unit = "km/h";
  switch (units_speed) {
  case UNITS_SPEED_MS:
    speed_unit = "m/s";
    break;
    case UNITS_SPEED_MPH:
    speed_unit = "mph";
    break;
  case UNITS_SPEED_KN:
    speed_unit = "kn";
    break;
  }
  if (gps_state && gps_isValid) {
    sprintf(buf, "%d%s", gps_speed > 999 ? 999 : gps_speed, speed_unit);
  } else {
    sprintf(buf, "--%s", speed_unit);
  }
  return buf;
}

char *str_altitude() {
  static char buf[7]; // room for 99999' + \0 = 7
  const char *alt_unit = (units_dist == UNITS_DIST_FT ? "'" : "m");
  if (gps_state && gps_isValid) {
    sprintf(buf, "%d%s", max(-9999, min(99999, (int ) (units_dist == UNITS_DIST_FT ? gps.altitude.feet() : gps.altitude.meters()))), alt_unit);
  } else {
    sprintf(buf, "--%s", alt_unit);
  }
  return buf;
}

String getSpeedCourseAlti() {
  return ( String(str_speed()) + " " + String(str_course(1)) + " " + String(str_altitude()) );
}


String getSatAndBatInfo() {
  String line5;

  if (gps_state) {
    if (gps_task_enabled) {
      line5 = "S:" + String(gps.satellites.value()) + "/" + String((int ) gps.hdop.hdop());
    } else {
      line5 = "S:zzz";
    }
  } else {
    line5 = "S:-/-";
  }
#ifdef T_BEAM_V1_0
  int b_c_out = (int ) axp.getBattDischargeCurrent();
  int b_c_in = (int ) axp.getBattChargeCurrent();
  if (b_c_out == 0 && b_c_in == 0) {
    line5 = line5 + " P:" + String(InpVolts, 2) + "V";
  } else {
    line5 = line5 + " B:" + String(BattVolts, 2) + "V";
  }
  String charge = "";
  if (b_c_out > 0) {
    charge = "-" + String(b_c_out);
  } else if (b_c_in > 0) {
      charge = String(b_c_in);
  } else {
    charge = String((int ) axp.getVbusCurrent());
  }
  line5 = line5 + "/" + charge + "mA";
#elif T_BEAM_V1_2
  if (axp.isVbusIn()) {
    line5 = line5 + " P:" + String(InpVolts, 2) + "V";
  } else {
    line5 = line5 + " B:" + String(BattVolts, 2) + "V";
  }
  if (axp.isBatteryConnect()) {
    line5 = line5 + "/" + String(axp.getBatteryPercent()) + "%";
    if (axp.isCharging()) {
      line5 += "+"; // is charging -> indicate with "+"
    } else if (!axp.isVbusIn()) {
      line5 += "-"; // not charging and no vbus -> is discharging -> indicate with "-"
    }
  }
#else
  line5 = line5 + " P:" + String(InpVolts, 2) + "V";
#endif
#if defined(ENABLE_BLUETOOTH) && defined(KISS_PROTOCOL)
  if (line5.length() < OLED_LINE_LEN_MAX-3 && serial_bt_client_is_connected) {
    // ^ "S:0/99 B:3.52V/-190mA BT" would be too long for one oled line
    line5 += " BT";
  }
#endif
  return line5;
}

void fillDisplayLine1(int caller) {
  if (freeze_display)
    return;
  if (debug_verbose > 2)
    Serial.printf("fillDisplayLine1 caller:%d button_down:%d\r\n",caller, freeze_display);
  static uint32_t old_time = 0L;
  uint32_t t = millis() / 1000;
  char s_uptime[6];  // room for 49d17 + \0 -> 6 bytes

  if (*gps_time_s)
    OledLine1 = String(gps_time_s) + " ";
  else
    OledLine1 = "";

  // > 49d 17h 2min? millis-overflow -> mark it
  if (old_time != ~0 && t < old_time)
    old_time = ~0;

  if (old_time == ~0) {
    sprintf(s_uptime, ">49d");
  } else {
    int d = (t / 60 / 60 / 24);
    int h = ((t / 60 / 60) % 24);
    int m = ((t / 60) % 60);
    if (d)
      sprintf(s_uptime, "%2dd%2.2d", d % 100, h);
    else
      sprintf(s_uptime, "%2.2d:%2.2d", h, m);
    old_time = t;
  }

  String SMessageInfo = "";
  if (LastRXMessageInfo == 3) {
    // winlink and personal message -> "M"
    SMessageInfo = "  M";
  } else if (LastRXMessageInfo == 1) {
    // personal message -> "m"
    SMessageInfo = "  m";
  } else if (winlink_notified != 0L) {
    if (winlink_notified + 60*60*24*1000L > millis()) {
      // show winlink mail info for max 24h
      SMessageInfo = "  w";
    } else {
      // reset
      winlink_notified = 0L;
      // Remove winlink indicator bit
      if (LastRXMessageInfo & 2)
        LastRXMessageInfo &= ~2;
    }
  }
  OledLine1 = OledLine1 + String("Up:") + String(s_uptime) + SMessageInfo;
}

void fillDisplayLine2() {

  if (freeze_display)
    return;
#ifdef ENABLE_WIFI
        wifi_info = "WiFi";
        if (wifi_connection_status == WIFI_RUNNING_AS_AP) {
          wifi_info = (enable_webserver)? wifi_info + "-AP" : wifi_info + ":off";
        } else if (wifi_connection_status == WIFI_SEARCHING_FOR_AP) {
          wifi_info = (enable_webserver)? wifi_info + "-cli" : wifi_info + ":off";
        } else if (wifi_connection_status == WIFI_DISABLED) {
          wifi_info = (enable_webserver)? wifi_info + "-dis" : wifi_info + ":off";
        } else {
          wifi_info = (enable_webserver)? wifi_info + "-CLI" : wifi_info + ":off";
        }
#else
        wifi_info = "";
#endif

  if (dont_send_own_position_packets) {
    OledLine2 = wifi_info + " Own Bcn: dis";
  } else if (!(lora_tx_enabled || aprsis_enabled || usb_serial_data_type == 0 || usb_serial_data_type & 2)) {
    OledLine2 = wifi_info + " LoRa-TX: dis";
  } else {
    if (gps_isValid) {
      if (fixed_beacon_enabled) {
        OledLine2 = wifi_info+ " FB:" +String((next_fixed_beacon-millis()) / 1000) + "s";
      } else {
        ulong nextSBtx = ((lastPositionTX+nextTX)-millis())/1000;
        // do not send SB Info on very first tx, wrong value, cannot find the reason
        if (nextSBtx > 400000) {
          OledLine2 = wifi_info;
        } else {
          OledLine2 = wifi_info + " SB:" + String(nextSBtx) + "s";
        // next line will be used when debugging gps-output for smoothening km/h in oled
        //OledLine1s = OledLine1 + String(((lastPositionTX+nextTX)-millis())/1000)+"s "+ String(lastTxdistance);
        }
      }
    } else {
      OledLine2 = wifi_info;
    }
  }
}


void fillDisplayLines3to5(int force) {
  static uint32_t ratelimit_until = 0L;

  if (!force && millis() < ratelimit_until)
      return;
  ratelimit_until = millis() + (oled_timeout == 0 ? 1000 : showRXTime);

  boolean show_locator = false;
  if (oled_show_locator == 1) {
       show_locator = true;
  } else if (oled_show_locator > 1) {
    uint8_t t = millis() / 1000 % 120;
    if ( (oled_show_locator == 2 && (t % 60) >= 50) ||
         (oled_show_locator == 3 && t >= 100) ||
         (oled_show_locator == 4 && (t % 40) >= 20) ) {
       show_locator = true;
    }
  }

  if (oled_line3and4_format == 0) {
    if (show_locator) {
      OledLine3 = aprsLatLonAsMaidenheadGridLocator + " " + aprsPresetShown;
      #if defined(T_BEAM_V1_0) || defined(T_BEAM_V1_2)
        // field for extended use, i.e. temp, pressure, humidity
        if (aprsLatLonAsMaidenheadGridLocator.length() + 1 + 1 + + 12 <= 21) {
          #ifdef T_BEAM_V1_0
            float t_AXP = axp.getTemp();
          #elif T_BEAM_V1_2
            float t_AXP = axp.getTemperature();
          #endif
          char sensor_data[13] = { 0 } ; // room for 12 + \0 == 13
          if (t_AXP < -9.9) t_AXP = -9.9; else if (t_AXP > 99.9) t_AXP = 99.9;
          sprintf(sensor_data, " tAXP:%.1f%cC", t_AXP, '\xF7');
          OledLine3 = OledLine3 + String(sensor_data);
        }
      #endif
    } else {
      OledLine3 = aprsLatPreset + " " + aprsLonPreset + " " + aprsPresetShown;
    }
    OledLine4 = getSpeedCourseAlti();

  } else {
    char buf[22]; // OLED-Display can show 21 characters in a line

    // Thoughts about higher precision in nautical syntax (format 2). Also valid for
    // classic syntax(format 1), which has the same length.
    // try to adjust speed value to a nice offset, depending on it's unit
    // Example:
    // 356° 9999m 23-45.345N
    // 999km/h p 124-45.345E		// looks good
    // 356° 9999m 23-45.345N
    //   0km/h p 124-45.345E		// quite ok. Degree and 'k' are at the same position.
    // 356° 9999m 23-45.345N
    //     0kt p 124-45.345E		// looks bad
    // ^^^^ too much blanks for min value
    // 356° 9999m 23-45.345N
    //   539kt p 124-45.345E		// looks bad
    //   ^^ too much blanks for max value
    // String offset comparison:
    // 999km/h
    //   0km/h
    // 277m/s
    //   0m/s
    // 620mph   // has same max len(max value) and same len of unit than m/s
    //   0mph
    // 539kt
    //   0kt
    // You see the additional blanks at right side
    // ->
    // 356° 9999m 23-45.345N
    // 539kt   p 124-45.345E		// looks nice
    //      ^^ two more blanks
    // 356° 9999m 23-45.345N
    //   0kt   p 124-45.345E		// quite ok. Defree and 'k' are at the same position.
    // 356° 9999m 23-45.345N
    // 277m/s  p 124-45.345E		// quite ok. Defree and 'k' are at the same position.
    //       ^  one  more blank
    // 356° 9999m 23-45.345N
    //   1m/s  p 124-45.345E		// quite ok. Defree and 'k' are at the same position.
    // hack: append blanks, so it aligns more to left.
    // str_speed() assured length 7 (length 999km/h)

    char *spd = str_speed();
    if (units_speed == UNITS_SPEED_KN) {
      strcat(spd, "  ");
    } else if (units_speed == UNITS_SPEED_MPH || units_speed == UNITS_SPEED_MS) {
      strcat(spd, " ");
    }
    // end of hack

    if (show_locator) {
      // field for extended use, i.e. temp, pressure, humidity
      char sensor_data[12] = { 0 } ; // room for 11 + \0 == 12
      #if defined(T_BEAM_V1_0) || defined(T_BEAM_V1_2)
        #ifdef T_BEAM_V1_0
          float t_AXP = axp.getTemp();
        #elif T_BEAM_V1_2
          float t_AXP = axp.getTemperature();
        #endif
        if (t_AXP < -9.9) t_AXP = -9.9; else if (t_AXP > 99.9) t_AXP = 99.9;
        sprintf(sensor_data, "tAXP:%.1f%cC", t_AXP, '\xF7');
      #endif
      // show maidenhead grid locator
      if (oled_line3and4_format < 3) {
        sprintf(buf, "%4.4s%6.6s %-10.10s", str_course(0), str_altitude(), aprsLatLonAsMaidenheadGridLocator.c_str());
        OledLine3 = String(buf);
        sprintf(buf, "%7.7s %1s %-11.11s", spd, aprsPresetShown.c_str(), sensor_data);
        OledLine4 = String(buf);
      } else {
        sprintf(buf, "%11.11s %1s %7.7s", aprsLatLonAsMaidenheadGridLocator.c_str(), aprsPresetShown.c_str(), spd);
        OledLine3 = String(buf);
        sprintf(buf, "%11.11s %4.4s%5.5s", sensor_data, str_course(0), str_altitude());
        OledLine4 = String(buf);
      }
    } else {
      char b[12]; // room for 012-34.567E + \0 = 12
      if ((oled_line3and4_format % 2)) {
        // In this case, we could not use the aprsLatPresetNiceLocation as base for our adaption,
        // because it may be in heigher precision. Thus 012-34.937E would be cut to 012-34.93E;
        // rounding has taken place at aprsLatPreset. We use this.
        const char *p = aprsLatPreset.c_str();
        sprintf(b, "%.2s\xF7%s", p, p+2);
        int pos_lastchar = strlen(b)-1;
        // replace ...N by ...'N
        b[pos_lastchar+1] = b[pos_lastchar]; b[pos_lastchar] = '\''; b[pos_lastchar+2] = 0;
      } else {
        sprintf(b, "%.10s", aprsLatPresetNiceNotation.c_str());
      }
      if (oled_line3and4_format < 3) {
        sprintf(buf, "%4.4s%6.6s %-10.10s", str_course(0), str_altitude(), b);
      } else {
        // coordinates left aligned
        sprintf(buf, "%11.11s %1s %7.7s", b, aprsPresetShown.c_str(), spd);
      }
      OledLine3 = String(buf);
      if ((oled_line3and4_format % 2)) {
        const char *p = aprsLonPreset.c_str();
        sprintf(b, "%.3s\xF7%s", p, p+3);
        int pos_lastchar = strlen(b)-1;
        // replace ...N by ...'N
        b[pos_lastchar+1] = b[pos_lastchar]; b[pos_lastchar] = '\''; b[pos_lastchar+2] = 0;
      } else {
        sprintf(b, "%.11s", aprsLonPresetNiceNotation.c_str());
      }
      if (oled_line3and4_format < 3) {
        sprintf(buf, "%7.7s %1s %-11.11s", spd, aprsPresetShown.c_str(), b);
      } else {
        // only beautiful up to 999m or 304ft. > 999: no blank between ° and alt. > 99999: alt unit is cut.
        sprintf(buf, "%11.11s %4.4s%5.5s", b, str_course(0), str_altitude());
      }
      OledLine4 = String(buf);
    }
  }

  OledLine5 = getSatAndBatInfo();
}

String compute_time_since_received(uint32_t elapsed) {
  String recv_p;

  if (elapsed < 60) {
    recv_p = String(elapsed) + "s";
  } else if (elapsed < 3600) {
    recv_p = String(elapsed/60) + "m";
  } else if (elapsed < 86400) {
    recv_p = String(elapsed/3600) + "h";
  } else {
    recv_p = String(elapsed/86400) + "d";
  }
  return (recv_p);
}


void displayInvalidGPS() {
  uint32_t gpsage;
  if (freeze_display)
    return;
  String gpsage_p = " GPS: dis";
  fillDisplayLine1(1);
  if (gps_state) {
    //show GPS age (only if last retrieval was invalid)
    gpsage = gps.location.age()/1000;
    if (gpsage > 49700) {
      gpsage_p = " GPS: no fix";
    } else {
      gpsage_p = " GPS age:" + compute_time_since_received(gpsage);
    }
  }
  OledLine2 = wifi_info + gpsage_p;
  fillDisplayLines3to5(0);
  writedisplaytext(Tcall, OledLine1, OledLine2, OledLine3, OledLine4, OledLine5);
}

#if defined(KISS_PROTOCOL)
/**
 *
 * @param TNC2FormatedFrame
 */
void sendToTNC(const String& TNC2FormatedFrame) {
  if (tncToSendQueue){
    auto *buffer = new String();
    buffer->concat(TNC2FormatedFrame);
    stats_to_kiss++;
    if (xQueueSend(tncReceivedQueue, &buffer, (1000 / portTICK_PERIOD_MS)) != pdPASS){
      // remove buffer on error
      delete buffer;
    }
  }
}
#endif
#if defined(ENABLE_WIFI)
/**
 *
 * @param TNC2FormatedFrame
 */
void sendToWebList(const String& TNC2FormatedFrame, const int RSSI, const int SNR, char QRG) {
  if (webListReceivedQueue){
    auto *receivedPacketData = new tReceivedPacketData();
    receivedPacketData->packet = new String();
    receivedPacketData->packet->concat(TNC2FormatedFrame);
    // strip EOL
    receivedPacketData->packet->trim();
    receivedPacketData->RSSI = RSSI;
    receivedPacketData->SNR = SNR;
    receivedPacketData->QRG = QRG;
    getLocalTimeTheBetterWay(&receivedPacketData->rxTime);

    if (xQueueSend(webListReceivedQueue, &receivedPacketData, (1000 / portTICK_PERIOD_MS)) != pdPASS){
      // remove buffer on error
      delete receivedPacketData->packet;
      delete receivedPacketData;
    }
  }
}
#endif

String prepareCallsign(const String& callsign) {
  String tmpString = "";
  for (int i=0; i<callsign.length();++i){  // remove unneeded "spaces" from callsign field
    if (callsign.charAt(i) != ' ') {
      tmpString += callsign.charAt(i);
    }
  }
  tmpString.toUpperCase();
  return tmpString;
}

void set_callsign() {
  #ifdef ENABLE_PREFERENCES
    String s = prepareCallsign(preferences.getString(PREF_APRS_CALLSIGN, ""));
  #else
    String s = "";
  #endif
  if (s.isEmpty()) {
    s = prepareCallsign(String(CALLSIGN));
    if (s.isEmpty()) {
      s = String("N0CALL");
    }
    #ifdef ENABLE_PREFERENCES
      preferences.putString(PREF_APRS_CALLSIGN, s);
      #if defined(ENABLE_SYSLOG)
        if (debug_verbose)
          syslog_log(LOG_DEBUG, String("FlashWrite preferences: set_Callsign()"));
      #endif
    #endif
  }
  Tcall = String(s);
  if (Tcall == "N0CALL") {
    lora_tx_enabled = false;
    txPower = txPower_cross_digi = 0;
  }
}

// telemetry frames
#if defined(ENABLE_TNC_SELF_TELEMETRY)

// DO NOT ENABLE THIS UNTIL YOU READ AND UNDERSTOOD THE IMPACT DESCRIBED ABOVE
boolean really_allow_telemetry_on_main_freq = false;

void send_telemetry_to_TNC_usb_serial_and_aprsis(const String &telemetryPacket)
{
  #if defined(KISS_PROTOCOL)
    sendToTNC(telemetryPacket);
  #endif
  if (usb_serial_data_type & 2) {
    Serial.println(telemetryPacket);
    stats_to_serial++;
  }
  #if defined(ENABLE_WIFI)
    send_to_aprsis(telemetryPacket);
    // another hack: send_to_aprsis has no queue. Webserver-code needs enough time to send. Are 500ms enough?
    esp_task_wdt_reset();
    delay(500);
    esp_task_wdt_reset();
  #endif
}

char encode_int_in_char(int i) {
  if (i < 0 || i > 61)
    return '0';
  if (i < 10) return '0' + i;
  if (i < 36) return 'A' + (i - 10);
  return 'a' + (i - 36);
}


// These variables should have been inside sendTelemetryFrame, declared static,
// but due to some obscure phaenomen they loose their assigned value. WTF!
// Bufferoverflow somewhere in the function??
uint8_t EqnsParmUnitBITS_frame_curr = 0;
uint32_t next_time_to_send_telemetry_EqnsParmUnitBITS = 0L;
#ifdef T_BEAM_V1_0
  // We may add axp temperature either if we have no battery (B V, B C out, B C in 0),
  // or if no USB is plugeed in (B C in will be 0). -> We could use the position
  // of B C in. We decided this on boot and remember,
  boolean may_add_temperature = (!axp.isVBUSPlug() || (float ) axp.getBattVoltage() < 1000.0);
#elif T_BEAM_V1_2
  //boolean may_add_temperature = (!axp.isVbusInsertOnSource() || (float ) axp.getBattVoltage() < 1000.0);
  // until we can't get current load (axp.xxxCurrent() is not available by the 2101 driver),
  // we always have a free field for adding a temperature measurement
  boolean may_add_temperature = true;
#endif

#define ALSO_SEND_Telemetry_BITS 0	// Set this to 1 if you need to send also the "digital BITS packet"
void sendTelemetryFrame() {
  const uint8_t EqnsParmUnitBITS_frames = (ALSO_SEND_Telemetry_BITS ? 4 : 3);
  String tel_sequence_str;
  String tel_path_str;

  if (!enable_tel)
    return;
  if (!lora_tx_enabled)
    tel_allow_tx_on_rf = 0;

  // Format telemetry path
  if (tel_path == "") {
    tel_path_str = tel_path;
  } else {
    tel_path_str = "," + tel_path;
  }
  String telemetryBase = Tcall + ">" + MY_APRS_DEST_IDENTIFYER + tel_path_str + ":";
  String telemetryBaseRF = Tcall + ">" + MY_APRS_DEST_IDENTIFYER + ":";  // No digi path on RF. Description see below

  // equations, unit, names, bits packets: .. . They never change.
  // For more details, see aprs spec!
  // Ratelimit them. Once a day is enough. This is not really important for
  // messages sent to aprsis (except they come as // 3rd party traffic back
  // to RF); or: if someone has the idea to send this over our slow LoRa..

  if (millis() > next_time_to_send_telemetry_EqnsParmUnitBITS) {
    // hack: Send one of the messages along with one one telemetryData frame.
    // If all are sent, remember the time we last sent all of them.
    String s;

    switch (EqnsParmUnitBITS_frame_curr) {
    case 0:
      // Equations are defined only for the 5 analog channels. May be less then 5.
      // Most important for correct interpretation -> position 0.
      // Item length may vary
      s = String("EQNS.");
      //s = s + "0,5.1,3000";
      s = s + "0,33.8,0";
      #if defined(T_BEAM_V1_0) || defined(T_BEAM_V1_2)
        //s = s + ",0,10,0" + ",0,10,0" + ",0,28,3000" + ",0,10,0";
        s = s + ",0,10,0" + ",0,16.9,0" + ",0,10,0";
        if (may_add_temperature)
          s = s + ",0,0.25,-5";
        else
          s = s + ",0,10,0";
      #endif
      break;
    case 1:
      // up to 5 analog and 8 digital channels. If not needed, could break at any item.
      // if you use 4 analog and 2 digital channel, set names to "A,B,C,D,,X,Y"
      // Item lengths are strict. Look at spec!
      s = String("PARM.");
      //s = s + "B Volt";
      s = s + "P V";
      #if defined(T_BEAM_V1_0) || defined(T_BEAM_V1_2)
        //s = s + ",B In" + ",B Out" + ",AC V" + ",AC C";
        s = s + ",P C" + ",B V" + ",BCout";
        if (may_add_temperature)
          s = s + ",Temp";
        else
          s = s + ",BCin";
      #else
        //if (EqnsParmUnitBITS_frames == 3) s = s + ",,,,," /* + "yourDigitalParamNames1,2,3,4,5,6,7,8" */;
      #endif
      break;
    case 2:
      // up to 5 analog and 8 digital channels. If not needed, could break at any item.
      // if you use 4 analog and 2 digital channel, set names to "A,B,C,D,,X,Y"
      // Item lengths are strict. Look at spec!
      s = String("UNIT.");
      s = s + "mV";
      #if defined(T_BEAM_V1_0) || defined(T_BEAM_V1_2)
        //s = s + ",mA" + ",mA" + ",mV" + ",mA";
        s = s + ",mA" + ",mV" + ",mA";
        if (may_add_temperature)
          s = s + ",C";
        else
          s = s + ",mA";
      #else
        //if (EqnsParmUnitBITS_frames == 3) s = s + ",,,,," /* + "yourDigitalUnitsNames1,2,3,4,5,6,7,8" */;
      #endif
      break;
    case 3:
      //Example for the up 8 digital BITS channel (currently not used)
      //It contins exact 8 bytes of 0 or 1, followed by up to 23 bytes procect title.
      s = String("BITS.");
      s = s + "00000000" + ",SQ9MDD LoRa Tracker";
      break;
    }

    if (EqnsParmUnitBITS_frame_curr == EqnsParmUnitBITS_frames-1) {
      next_time_to_send_telemetry_EqnsParmUnitBITS = millis() + 24*60*60*1000L;
      EqnsParmUnitBITS_frame_curr = 0;
    } else {
      EqnsParmUnitBITS_frame_curr++;
    }

    // Pad telemetry message address to 9 characters. Plus string termination \0. Plus 2x ':'
    char Tcall_message_char[2+9+1];
    //sprintf_P(Tcall_message_char, ":%-9s:", Tcall.c_str());
    sprintf_P(Tcall_message_char, ":%-9.9s:", Tcall.c_str());
    String Tcall_message = String(Tcall_message_char);

    send_telemetry_to_TNC_usb_serial_and_aprsis(telemetryBase + Tcall_message + s);

    // We don't like to see telemetry on RF.
    // But since we have been asked several times, here's a suggestion we could live with it:
    // When digipeating on cross-qrg, fast "1200" mode (comparable to 2m AFSK APRS) is recommended anyway.
    // It may be acceptable to send to rf in this case. But only as direct message, without digipeaters.
    // On our slow main qrg, it's still not acceptable to flood the channel with telemetry.
    // That's why it's commented out here in the source. If you really need this, i.e. for a balloon mission,
    // you could enable it. It's here as a sample, only for providing the correct way to use that "feature".
    // Telemetry on secondary qrg is sent if config variable is 2 or 3. On main qrg: 1 or 3.

    if (tel_allow_tx_on_rf) {
      String telemetryPacket = telemetryBaseRF + Tcall_message + s;
      if (tel_allow_tx_on_rf & 2) {
        if (lora_freq_cross_digi != lora_freq && lora_speed_cross_digi >= 1200) {
          loraSend(txPower_cross_digi, lora_freq_cross_digi, lora_speed_cross_digi, 0, telemetryPacket);
          stats_tx_freq_secondary++;
        }
      }
      if ((tel_allow_tx_on_rf & 1) && really_allow_telemetry_on_main_freq) {
        loraSend(txPower, lora_freq, (lora_speed < 300) ? 300 : lora_speed, 0, telemetryPacket);
        stats_tx_freq_main++;
      }
    }
    stats_tx_telemetry++;
  }

  // sequence number for measurement packet
  #ifndef ENABLE_PREFERENCES
    // no preferences enabled -> use variant -1.
    if (tel_mic == 0) tel_mic = -2;
  #endif

  // Determine if MIC, digit sequence number or alphanumeric sequence number
  if (tel_mic == 1) {
    tel_sequence_str = "MIC";
  } else if (tel_mic == -1) {
    // Unfortunately, aprs.fi comments this with "[Invalid telemetry packet]".
    //   -> See also the less efficient approach tel_mic == -2
    // a much better approach, without storing sequence number back to flash.
    // On most trackers, the system time is set correct, either due to GPS time, or by NTP.
    // Telemetry sequence can be any number or letter. It has a size of 3 characters.
    // For convenience, we store in pos 0 the month. If we use letters A-Z, a-z and and numbers 0-9, we could adress 62 days (exactly 2 months), before we start from new.
    // Pos 1 and 2: encodes the time. -> We get a resolution of min 23s, which is more than we need in typical usecases: 24*60*60.0/((26*2+10)**2) = 22.476s
    struct tm timeinfo{};
    if (getLocalTimeTheBetterWay(&timeinfo)) {
      char buf[4];
      int t = (timeinfo.tm_mon % 2) * 31 + ((timeinfo.tm_mday - 1) % 31);
      buf[0] = encode_int_in_char(t);
      // 24*60*60/(26*2+10)**2 = 22.476s
      t = ((timeinfo.tm_hour *60*60) + timeinfo.tm_min * 60 + timeinfo.tm_sec) / 23;
      buf[1] = encode_int_in_char(t / 62);
      buf[2] = encode_int_in_char(t % 62);
      buf[3] = 0;
      // special case: reserved word "MIC". Hmm.. ...risk a doublette; fake time to next interval += 23s -> "MID" ;)
      if (!strcmp(buf, "MIC"))
        buf[2] = 'D';
      tel_sequence_str = String(buf);
    } else {
      // fall back to MIC format
      tel_sequence_str = String("MIC");
    }
 } else if (tel_mic == -2) {
    // This variant has a resolution of 10min and overflows in a week; in contrast to
    // tel_mic -1 variant (numeric+letter based), which has a resolution of 23 seconds
    // and overflows in two months.
    // assumption: we send telemetry at min every 10min (enforced by web interface)
    // We could encode week day in pos 0 (-> 0-6). Pos 1+2 are time dependend
    // values. 6*24*6+22*6+3 = 999 packets, last sent at saturday 22:30. We need room
    // for 8 packets. Let's send them as 000 up to 006, and the last (from sat, 23:50) to 999, and see what happens ;)
    // Because amateur radio projects like balloons often use the weekend, it's better
    // to have the overflow in mid of the week instead of saturday evening. We start our
    // week at thursday instead of sunday -> (tm_wday += 4).
    struct tm timeinfo{};
    if (getLocalTimeTheBetterWay(&timeinfo)) {
      char buf[4];
      // resolution 6 packets in an hour.
      int t = (24*60*((timeinfo.tm_wday + 4) % 7) + 60*timeinfo.tm_hour + timeinfo.tm_min) / 10.0;
      if (t >= 1007)
        t = 999;
      sprintf_P(buf, "%03u", t % 1000);
      tel_sequence_str = String(buf);
    } else {
      // fall back to MIC format
      tel_sequence_str = String("MIC");
    }
  } else {
    // Get the current saved telemetry sequence
    #ifdef ENABLE_PREFERENCES
      tel_sequence = preferences.getUInt(PREF_TNC_SELF_TELEMETRY_SEQ, 0) % 1000;
    #endif
    // Pad to 3 digits. Plus string termination \0
    char tel_sequence_char[3+1];
    sprintf_P(tel_sequence_char, "%03u", tel_sequence);
    tel_sequence_str = String(tel_sequence_char);

    // Update the telemetry sequence number
    if (tel_sequence >= 999) {
      tel_sequence = 0;
    } else {
      tel_sequence = tel_sequence + 1;
    }
    #ifdef ENABLE_PREFERENCES
      preferences.putUInt(PREF_TNC_SELF_TELEMETRY_SEQ, tel_sequence);
      #if defined(ENABLE_SYSLOG)
        if (debug_verbose)
          syslog_log(LOG_DEBUG, String("FlashWrite preferences: sendTelemetryFrame()"));
      #endif
    #endif
  }

  // measurement
  // min(): because we obviously measured 4.3.11. Result is 257 and sent value 2.
  // Working with uint8_t is a good decision, because due to spec, the value must not
  // be greater than 255, because it's the numeric representation of an 8 bit value.
  #if defined(T_BEAM_V1_0) || defined(T_BEAM_V1_2)
    // No, batteries or external power may not be present -> do not start at 3000 mW
    //uint8_t dc_volt = max(0, min((int ) (((((float ) axp.getVbusVoltage()) - 3000.0) / 28), 255)));
    uint8_t dc_volt = max(0, min((int ) (((float ) axp.getVbusVoltage()) / 33.8), 255));
    //uint8_t b_volt = max(0, min((int ) (((((float ) axp.getBattVoltage()) - 3000.0) / 5.1), 255)));
    uint8_t b_volt = max(0, min((int ) (((float ) axp.getBattVoltage()) / 16.9), 255));
    #ifdef T_BEAM_V1_0
      uint8_t dc_c = max(0, min((int ) (axp.getVbusCurrent() / 10), 255));
      uint8_t b_c_out = max(0, min((int ) (axp.getBattDischargeCurrent() / 10), 255));
      uint8_t b_c_in = may_add_temperature ? 0 : min((int ) (axp.getBattChargeCurrent() / 10), 255);
      uint8_t axp_temperature = may_add_temperature ? max(min((int ) ((axp.getTemp() + 5 + 0.125) / 0.25), 255), 0) : 0;
    #elif T_BEAM_V1_2
      uint8_t dc_c = 0;
      uint8_t b_c_out = 0;
      uint8_t b_c_in = 0;
      uint8_t axp_temperature = max(min((int ) ((axp.getTemperature() + 5 + 0.125) / 0.25), 255), 0);
    #endif
  #else
    batt_read();
    //uint8_t b_volt = max(0, min((int ) ((InpVolts * 1000) - 3000 / 5.1), 255));
    uint8_t dc_volt = max(0, min((int ) (InpVolts * 1000 / 33.8), 255));
  #endif


  String telemetryData = String("T#") + tel_sequence_str;
  // aprs spec says a an analog field has a length of 3. -> 0 is 000.
  // This is nearly impossible to do with String functions. We need a helper buffer
  // aprs spec that Telemetry Report format is exactly 5 analog values and 8 digital values.
  // We break with this standard here, because we don't like waste bandwith
  char buf[5]; // ",000" + \0 == 5
  sprintf(buf, ",%03u", dc_volt); telemetryData += String(buf);
  #if defined(T_BEAM_V1_0) || defined(T_BEAM_V1_2)
    sprintf(buf, ",%03u", dc_c); telemetryData += String(buf);
    sprintf(buf, ",%03u", b_volt); telemetryData += String(buf);
    sprintf(buf, ",%03u", b_c_out); telemetryData += String(buf);
    if (may_add_temperature) {
      sprintf(buf, ",%03u", axp_temperature); telemetryData += String(buf);
    } else {
      sprintf(buf, ",%03u", b_c_in); telemetryData += String(buf);
    }
  #else
    //telemetryData += ",000,000,000,000"; // <- would be needed if we'd standard conform!
  #endif
  //telemetryData += ",00000000"; // <- would be needed if we'd standard conform!
  //telemetryData += "Optional Comment" // If you need comment, it's important that your repot has exactly 5 analog and 8 digital data values, else you'd to harm to parsers. No leading ',' needed.
  //do_serial_println("Telemetry: " + telemetryBase + telemetryData);

  // Flash the light when telemetry is being sent
  #ifdef ENABLE_LED_SIGNALING
    digitalWrite(TXLED, TXLED_HIGH);
  #endif

  // Show when telemetry is being sent
  stats_tx_telemetry++;
  enableOled_now(); // enable OLED
  writedisplaytext("((TEL TX))",String("#") + stats_tx_telemetry,telemetryData,"","","");

  send_telemetry_to_TNC_usb_serial_and_aprsis(telemetryBase + telemetryData);

  // We don't like to see telemetry on RF.
  // But since we have been asked several times, here's a suggestion we could live with it:
  // When digipeating on cross-qrg, fast "1200" mode (comparable to 2m AFSK APRS) is recommended anyway.
  // It may be acceptable to send to rf in this case. But only as direct message, without digipeaters.
  // On our slow main qrg, it's still not acceptable to flood the channel with telemetry.
  // That's why it's commented out here in the source. If you really need this, i.e. for a balloon mission,
  // you could enable it. It's here as a sample, only for providing the correct way to use that "feature".
  // Telemetry on secondary qrg is sent if config variable is 2 or 3. On main qrg: 1 or 3.

  if (tel_allow_tx_on_rf) {
    String telemetryPacket = telemetryBaseRF + telemetryData;
    if (tel_allow_tx_on_rf & 2) {
      if (lora_freq_cross_digi != lora_freq && lora_speed_cross_digi >= 1200) {
        loraSend(txPower_cross_digi, lora_freq_cross_digi, lora_speed_cross_digi, 0, telemetryPacket);
        stats_tx_freq_secondary++;
      }
    }
    if ((tel_allow_tx_on_rf & 1) && really_allow_telemetry_on_main_freq) {
      loraSend(txPower, lora_freq, (lora_speed < 300) ? 300 : lora_speed, 0, telemetryPacket);
      stats_tx_freq_main++;
    }
  }

  // Flash the light when telemetry is being sent
  #ifdef ENABLE_LED_SIGNALING
    digitalWrite(TXLED, TXLED_LOW);
  #endif
}

#endif // ENABLE_TNC_SELF_TELEMETRY


#ifdef ENABLE_WIFI
// setup wifi variables. AP-array: first element with preferences from flash. increase apcnt if successfull.
void init_wifi_STA_and_AP_settings() {
    wifi_ModeSTA_SSID = "";
    wifi_ModeSTA_PASS = "";
    apcnt = 0;
    if (/* !preferences.getString(PREF_WIFI_PASSWORD, "").isEmpty( && ) */ // no, connection to open wifi AP is allowed */
        !preferences.getString(PREF_WIFI_SSID, "").isEmpty()) {
      // save old wifi credentials on pos 1, assuming that thhis is the best chance to reconnect
      wifi_ModeSTA_PASS = preferences.getString(PREF_WIFI_PASSWORD, "");
      wifi_ModeSTA_SSID = preferences.getString(PREF_WIFI_SSID, "");
      strncpy(APs[apcnt].ssid, wifi_ModeSTA_SSID.c_str(), sizeof(APs[apcnt].ssid)-1);
      APs[apcnt].ssid[sizeof(APs[apcnt].ssid)-1] = 0;
      strncpy(APs[apcnt].pw, wifi_ModeSTA_PASS.c_str(), sizeof(APs[apcnt].pw)-1);
      APs[apcnt].pw[sizeof(APs[apcnt].pw)-1] = 0;
      Serial.printf("Preferences AP %s found with PW '%s' and stored at pos %d\r\n", APs[apcnt].ssid, APs[apcnt].pw, apcnt);
      apcnt = 1;
    }

    if (!preferences.getString(PREF_AP_PASSWORD, "").isEmpty()) {
      wifi_ModeAP_PASS = preferences.getString(PREF_AP_PASSWORD, "");
      Serial.printf("Preferences Self-AP PW found '%s' and stored\r\n", wifi_ModeAP_PASS.c_str());
    }
}
#endif // ENABLE_WIFI


// SPIFFS for wifi.cfg
void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {

  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if(!root) {
    Serial.println("− failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(" − not a directory");
    root.close();
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
       Serial.println(file.name());
       if (levels){
          listDir(fs, file.name(), levels -1);
       }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      if (strlen(file.name()) < 12) {
        Serial.print("\t");
      }
      Serial.print("\tSIZE: ");
      Serial.println(file.size());
    }
    file.close();
    file = root.openNextFile();
  }
  root.close();
}


// write configuration to file
int writeFile(fs::FS &fs, const String &callername, const char *filename, const String &jsonData) {
  File file = fs.open(filename, FILE_WRITE);
  int ret = 0;
  if (!file) {
    do_serial_println(callername + ": failed to open file for writing");
    return -2;
  }
  if (file.print(jsonData)) {
    do_serial_println(callername + ": file " + filename + " written");
    #if defined(ENABLE_SYSLOG)
      if (debug_verbose)
        syslog_log(LOG_DEBUG, String("FlashWrite filesystem: writeFile(") + String(filename) + String(")"));
    #endif
  } else {
    do_serial_println(callername + ": write of " + filename + " failed!");
    ret = -3;
  }
  file.close();
  return ret;
}

int save_to_file(const String &callername, const char *filename, const String &jsonData) {
  int err = 0;
  if (SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    err = writeFile(SPIFFS, callername, filename, jsonData);
    SPIFFS.end();
  } else {
    do_serial_println(callername + ": SPIFFS Mount failed, nothing written.");
    Serial.println("SPIFFS Mount Failed");
    err = -1;
  }
  return err;
}



// readFile - is more than reading a file from flash. It reads and parses json data in the file and assigns some of our variables.
// The correct function name would be readFile_parseJson_and_assignVariable(). This is would be too ugly.
boolean readFile(fs::FS &fs, const char *filename) {
  //#define JSON_MAX_FILE_SIZE 2560
  //static StaticJsonDocument<JSON_MAX_FILE_SIZE> JSONBuffer;                         //Memory pool
  //best would be, that readFile returns a pointer to the local JSONBuffer on success, else NULL
  boolean err = false;
  char JSONMessage[JSON_MAX_FILE_SIZE];

  if (!SPIFFS.exists(filename)) {
    Serial.printf("readFile: %s does not exist\r\n", filename);
    return false;
  }
  File file = fs.open(filename, FILE_READ);
  if (!file) {
    Serial.printf("readFile: failed to open file %s for reading\r\n", filename);
    return false;
  }
  if (file.isDirectory()) {
    Serial.printf("readFile: %s is a directory, not a file\r\n", filename);
    file.close();
    return false;
  }
  Serial.printf("readFile: opened file %s for reading\r\n", filename);

  int pos = 0;
  for (; pos < JSON_MAX_FILE_SIZE-1; pos++) {
    if (!file.available())
      break;
    JSONMessage[pos] = file.read();
  }
  JSONMessage[pos] = 0;

  // uncomment if you need to see the content of the data on SPIFFS
  // Serial.printf("readFile, content: [%s]\r\n", JSONMessage);

  if (file.size() > JSON_MAX_FILE_SIZE) {
    Serial.printf("readFile: Warning, file too big: %d Byte (max: %d)\r\n", file.size(), JSON_MAX_FILE_SIZE);
  }


  //https://arduinojson.org/v6/doc/upgrade/

  auto error = deserializeJson(JSONBuffer, JSONMessage);
  if (error) {
    Serial.print(F("readFile: deserializeJson() failed with code "));
    Serial.println(error.c_str());
    err = true;
    goto end;
  }

#ifdef ENABLE_WIFI
  if (!strcmp(filename, "/wifi.cfg")) {
    String wifi_ssid_old = "";
    String wifi_password_old = "";

    // read old data structure, if available
    const char *p;
    if (JSONBuffer.containsKey("SSID1") && JSONBuffer.containsKey("password1")) {
      if ((p = JSONBuffer["SSID1"]))
        wifi_ssid_old = String(p);
      if ((p = JSONBuffer["password1"]))
        wifi_password_old = String(p);
      if (wifi_ssid_old.length() && (wifi_password_old.length() < 8 || wifi_password_old.length() > 63 || wifi_ssid_old == "EnterSSIDofYourAccesspoint")) {
        wifi_ssid_old = "";
        wifi_password_old = "";
      } else {
        if (apcnt < MAX_AP_CNT && strcmp(wifi_ssid_old.c_str(), wifi_ModeSTA_SSID.c_str()) && strcmp(wifi_password_old.c_str(), wifi_ModeSTA_PASS.c_str())) {
          strncpy(APs[apcnt].ssid, wifi_ssid_old.c_str(), sizeof(APs[apcnt].ssid)-1);
          APs[apcnt].ssid[sizeof(APs[apcnt].ssid)-1] = 0;
          strncpy(APs[apcnt].pw, wifi_password_old.c_str(), sizeof(APs[apcnt].pw)-1);
          APs[apcnt].pw[sizeof(APs[apcnt].pw)-1] = 0;
          Serial.printf("readFile: wifi.cfg, old structure AP %s found with PW '%s' (%d)\r\n", APs[apcnt].ssid, APs[apcnt].pw,apcnt);
          apcnt++;
        } else {
          Serial.printf("readFile: wifi.cfg, Preferences AP %s found with PW '%s' in wifi.cfg and not stored again\r\n", wifi_ModeSTA_SSID.c_str(), wifi_ModeSTA_PASS.c_str());
        }
      }
    }
    if (JSONBuffer.containsKey("SSID2") && JSONBuffer.containsKey("password2")) {
      if ((p = JSONBuffer["SSID2"]))
        wifi_ssid_old = String(p);
      if ((p = JSONBuffer["password2"]))
        wifi_password_old = String(p);
      if (wifi_password_old.length()  && (wifi_password_old.length() < 8 || wifi_password_old.length() > 63 || wifi_ssid_old == "EnterSSIDofYourAccesspoint")) {
        wifi_ssid_old = "";
        wifi_password_old = "";
      } else {
        if (apcnt < MAX_AP_CNT && strcmp(wifi_ssid_old.c_str(), wifi_ModeSTA_SSID.c_str()) && strcmp(wifi_password_old.c_str(), wifi_ModeSTA_PASS.c_str())) {
          strncpy(APs[apcnt].ssid, wifi_ssid_old.c_str(), sizeof(APs[apcnt].ssid)-1);
          strncpy(APs[apcnt].pw, wifi_password_old.c_str(), sizeof(APs[apcnt].pw)-1);
          Serial.printf("readFile: wifi.cfg, old structure AP %s found with PW '%s' (%d)\r\n", APs[apcnt].ssid, APs[apcnt].pw,apcnt);
          apcnt++;
        } else {
          Serial.printf("readFile: wifi.cfg, Preferences AP %s found with PW '%s' in wifi.cfg and not stored again\r\n", wifi_ModeSTA_SSID.c_str(), wifi_ModeSTA_PASS.c_str());
        }
      }
    }

    // Key for self AP password: new syntax: "SelfAP_PW" ; old syntax: "ap_password"
    if ( (JSONBuffer.containsKey("SelfAP_PW") && (p = JSONBuffer["SelfAP_PW"])) ||
         (JSONBuffer.containsKey("ap_password") && (p = JSONBuffer["ap_password"])) ) {
        String ap_password = String(p);
        if (ap_password.length() > 7) {
          wifi_ModeAP_PASS = String(ap_password);
          Serial.printf("readFile: wifi.cfg, valid Self-AP PW to be used '%s'\r\n", wifi_ModeAP_PASS.c_str());
        }
    }

    if (JSONBuffer.containsKey("AP")) {
      for (JsonObject AccessPoint : JSONBuffer["AP"].as<JsonArray>()) {
        if (strcmp(AccessPoint["SSID"], wifi_ModeSTA_SSID.c_str()) || strcmp(AccessPoint["password"], wifi_ModeSTA_PASS.c_str())) {
          strncpy(APs[apcnt].ssid, AccessPoint["SSID"], sizeof(APs[apcnt].ssid)-1);
          strncpy(APs[apcnt].pw, AccessPoint["password"], sizeof(APs[apcnt].pw)-1);
          // delay(3000); // uncomment if serial prints are not showing
          // uncomment if you need to see the content of the data on SPIFFS
          // Serial.printf("readFile: content JSON: [%d] [%s %s]\r\n", apcnt, APs[apcnt].ssid, APs[apcnt].pw);

          if (!sizeof(APs[apcnt].ssid) || (sizeof(APs[apcnt].pw > 0 &&  (sizeof(APs[apcnt].pw) < 8 || sizeof(APs[apcnt].pw) > 63))) || !strcmp(APs[apcnt].ssid, "EnterSSIDofYourAccesspoint") || !strcmp(APs[apcnt].ssid, "EnterSSIDofYour2ndAccesspoint")) {
            delay(3000); // something is wrong, make sure, that msg is displayed
            Serial.printf("readFile: SSID: %s missing or PW: '%s' < 8 Byte || > 63 Byte, Filesize: %d\r\n", APs[apcnt].ssid, APs[apcnt].pw, file.size());
          } else {
            if (!apcnt) {
              // take first configured AP as active, if nothing has been found in flash
              // keyword "prio=1" will override
              wifi_ModeSTA_SSID = String(APs[apcnt].ssid);
              wifi_ModeSTA_PASS = String(APs[apcnt].pw);
            } else {
              if (AccessPoint["prio"]) {
                strcpy(APs[0].ssid, APs[apcnt].ssid);
                strcpy(APs[0].pw, APs[apcnt].pw);
                strcpy(APs[apcnt].ssid, wifi_ModeSTA_SSID.c_str());
                strcpy(APs[apcnt].pw, wifi_ModeSTA_PASS.c_str());
                wifi_ModeSTA_SSID = String(APs[0].ssid);
                wifi_ModeSTA_PASS = String(APs[0].pw);
              }
            }
            Serial.printf("readFile: wifi.cfg, valid AP %s found with PW '%s' (%d)\r\n", APs[apcnt].ssid, APs[apcnt].pw, apcnt);
            apcnt++;
          }
        } else {
            if (wifi_ModeSTA_SSID == "")
              Serial.printf("readFile: wifi.cfg, No AP configured\r\n");
            else
              Serial.printf("readFile: wifi.cfg, Preferences AP %s found with PW '%s' in wifi.cfg and not stored again\r\n", wifi_ModeSTA_SSID.c_str(), wifi_ModeSTA_PASS.c_str());
        }
        if (apcnt == MAX_AP_CNT) {
          Serial.printf("readFile: wifi.cfg, maximum Number of possible APs (%d) reached.\r\n", MAX_AP_CNT);
          break;
        }
      }
    } else {
      delay(3000); // something is wrong, make sure, that msg is displayed
      Serial.println("readFile: wifi.cfg, no valid AP found)");
    }
    Serial.printf("readFile: wifi.cfg, %d valid entries found.", apcnt);
    if (apcnt)
      Serial.printf(" AP %s is selected as frist priority", wifi_ModeSTA_SSID.c_str());
    Serial.println("");
    goto end;
  }

#endif // ENABLE_WIFI

  if (!strcmp(filename, "/preferences.cfg")) {
    if (JSONBuffer.containsKey(PREF_APRS_CALLSIGN)) {
      // Serial.printf("Checked preferences.cfg: is ok. Found %s: %s. Filesize: %d\r\n", PREF_APRS_CALLSIGN, JSONBuffer[PREF_APRS_CALLSIGN], file.size());
      Serial.printf("readFile: Checked preferences.cfg: is ok. Found %s. Filesize: %d\r\n", PREF_APRS_CALLSIGN, file.size());
      Serial.println("readFile: Preferences: reading from /preferences.cfg");
      load_preferences_cfg_file();
      // needed here, because callsign is not initialized by load_preferences_from_flash()
      String s = jsonElementFromPreferenceCFGString(PREF_APRS_CALLSIGN, 0);
      s = prepareCallsign(s);
      if (s.isEmpty())
        s = prepareCallsign(String(CALLSIGN));
      if (s.isEmpty())
        s = String("N0CALL");
      preferences.putString(PREF_APRS_CALLSIGN, s);
    } else {
      Serial.println("readFile: Preferences: /preferences.cfg not available, using default values from flash");
      err = true;
    }
    goto end;
  }
  Serial.printf("readFile: Found file '%s', parsed it successfully, but I don't know what to do with the json data ;)!\r\n", filename);
  err = true;

end:
  Serial.println("readFile: end");
  file.close();
  if (err)
    return false;

  return true;
}


int compute_maidenhead_grid_fields_squares_subsquares(char *locator, int locator_size, float deg, int pos_start) {
  char *p = locator;
  int div = 24;

  if (locator_size < 4 || !(locator_size % 2) || pos_start > 2)
    return -1;

  p = p + pos_start;

  *p = 'A' + (int ) deg / 10;
  p = p+2;
  *p = '0' + ((int ) deg % 10);
  p = p+2;

  deg = (deg - (int ) deg);

  for (;;) {
    deg = (deg - (int ) deg) *div;
    *p = (div == 10 ? '0' : (p-locator < 6) ? 'A' : 'a') + (int ) deg;
    div = (div == 10 ? 24 : 10);
    p = p+2;
    if (p-locator > locator_size-2) {
      break;
    }
  }
  *p = 0;

  return 0;
}

String compute_maidenhead_grid_locator(const String &sLat, const String &sLon, int ambiguity) {
  const char *p_lat = sLat.c_str();
  const char *p_lon = sLon.c_str();

  static char locator[13]; // Room for JO62QN11aa22 + \0 == 13
  char buf[4];
  float deg;

  // Resolution 180/18./10/ 24*60 /10/24/10 * 1852 = 1.93m
  sprintf(buf, "%.2s", p_lat);
  deg = atoi(buf) + atof(p_lat +3) /60.0;
  if (p_lat[strlen(p_lat)-1] == 'N')
    deg = 90.0 + deg + 0.0000001;
  else
    deg = 90.0 - deg;
  if (deg > 179.99999) deg = 179.99999; else if (deg < 0.0) deg = 0.0;
  if (compute_maidenhead_grid_fields_squares_subsquares(locator, sizeof(locator), deg, 1) < 0)
    return String("AA00");

  // Resolution up to 180/2/18./10/ 24*60 /10/24/10 * 1852 = 3.85m; 1.93m at 60 deg N/S.
  sprintf(buf, "%.3s", p_lon);
  deg = atoi(buf) + atof(p_lon +4) /60.0;
  if (p_lon[strlen(p_lon)-1] == 'E')
    deg = 180.0 + deg + 0.0000001;
  else
    deg = 180.0 - deg;
  deg = deg / 2.0;
  if (deg > 179.99999) deg = 179.99999; else if (deg < 0.0) deg = 0.0;
  if (compute_maidenhead_grid_fields_squares_subsquares(locator, sizeof(locator), deg, 0) < 0)
    return String("AA00");

  if (ambiguity >= 4 || oled_loc_amb > 1)
    locator[2] = 0; // JO -> 600' == 1111.2km in latitude
  if (ambiguity == 3 || oled_loc_amb == 1)
    locator[4] = 0; // JO62 -> 60' == 111.12km in latitude
  else if (ambiguity == 2 || oled_loc_amb == 0)
    locator[6] = 0; // JO62qn -> 2.5' == 4.63km in latitude
  else if (ambiguity == 1 || oled_loc_amb == -1)
    locator[8] = 0; // JO62qn11 -> 0.25' -> 463m in latitude
  else if (ambiguity == 0 || oled_loc_amb == -2)
    locator[10] = 0; // JO62qn11aa -> 0.0104166' -> 19.3m. At lat (and 60deg N/S) almost exactly the normal aprs resolution
  else
    locator[12] = 0; // JO62qn11aa22 -> 0.00104166' > 1.93m. High Precision achievable with DAO !W..! extension.
  // JO62qn11aa22bb would not only hard readable. It would be a precision of 0.0000434' -> 8.034cm
  return String(locator);
}


int storeLatLonPreset(String _sLat, String _sLon, int precision) {
  String sLat = String(_sLat); // make a local String copy. Contense of _sLat can change if we change the variable name with wich storLatLonPreset() was called.
  String sLon = String(_sLon);
  String tmp_aprsLatPreset;
  String tmp_aprsLonPreset;
  String tmp_aprsLatPresetDAO;
  String tmp_aprsLonPresetDAO;
  String tmp_aprsLatLonDAO;
  String tmp_aprsLatPresetNiceNotation;
  String tmp_aprsLonPresetNiceNotation;
  String tmp_aprsLatLonAsMaidenheadGridLocator;
  const char *p;
  char buf[13];
  char helper_base91[] = {"0\0"};
  char nswe;
  float f;
  double fLon = 0.0;
  double fLat = 0.0;

  p = sLat.c_str();
  // some assurance
  if ( (sLat.length()+1) == sLon.length() &&
        (sLat.endsWith("N") || sLat.endsWith("S") ||
        (sLon.endsWith("E") || sLon.endsWith("W"))) ) {
    const char *q = sLon.c_str();
    // sLon has 3 numbers for degrees instead of two.
    if (!isdigit(*q))
      return -1;
    q++;
    for (int i = 0; q[i]; i++) {
      if (i == 2) {
        if (p[2] != '-' || q[2] != '-') {
          return storeLatLonPreset("00-00.00N", "000-00.0000W", 0);
        }
      } else if (i == 5) {
        if ((p[5] != '.' || q[5] != '.')) {
          return storeLatLonPreset("00-00.00N", "000-00.0000W", 0);
        }
      } else if (p[i+1] && q[i+1] && (!isdigit(p[i]) || !isdigit(q[i]))) {
        return storeLatLonPreset("00-00.00N", "000-00.0000W", 0);
      }
    }
  } else {
    return storeLatLonPreset("00-00.00N", "000-00.0000W", 0);
  }

  if (aprsLatLonInvalidPosition) {
    // APRS special notation 00000.00W means, no valid position; 00000.00E would be a correct position
    aprsLatLonInvalidPosition = (sLon == "000-00.0000W" && sLat == "00-00.0000N");
  }

  // We have stored the manual position string in a heigher precision (in case resolution more precise than 18.52m is required; i.e. for base-91 location encoding, or DAO extenstion).
  // Furthermore, 53-32.1234N is more readable in the Web-interface than 5232.1234N
  p = sLat.c_str();
  nswe = p[strlen(p)-1];
  sprintf(buf, "%.7s", p+3);
  // strip trailing N/S. This way we could have a dynamic length for input precision at sLat
  buf[strlen(buf)-1] = 0;
  f = atof(buf);
  // round up. sprintf for float does the rounding
  sprintf(buf, "%.2s%05.2f%c", p, (f > 59.99 ? 59.99 : f), nswe);
  tmp_aprsLatPreset = String(buf);

  // p is degrees, f is minutes.decimal
  sprintf(buf, "%.2s", p);
  fLat = atof(buf) + f/60.0;
  if (nswe == 'S')
   fLat *= -1;

  if (!aprsLatLonInvalidPosition && sLat.length() > 9 && (precision == 1 || precision == 2)) {
    float fp = f;
    if (precision == 1) {
      if (fp > 59.999) fp=59.999;
    } else {
      if (fp > 59.9999) fp=59.9999;
    }
    if (precision == 1) {
      sprintf(buf, "%.2s%06.3f", p, fp);
      tmp_aprsLatLonDAO = String("!W") + String(buf[7]);
    } else {
      sprintf(buf, "%.2s%07.4f", p, fp);
      ax25_base91enc(helper_base91, 1, atoi(buf+7)*0.91);
      tmp_aprsLatLonDAO = String("!w") + helper_base91[0];
    }
    buf[7] = nswe; buf[8] = 0;;
    tmp_aprsLatPresetDAO = String(buf);
    // "NiceNotation" may be used for presenting at oled. Four decimals would be too hard to read. We use 3 decimals for both, precision 1 and precision 2.
    sprintf(buf, "%.2s-%06.3f%c", p, (f > 59.999 ? 59.999 : f), nswe);
    tmp_aprsLatPresetNiceNotation = String(buf);
  } else {
    tmp_aprsLatPresetDAO = tmp_aprsLatPreset;
    tmp_aprsLatLonDAO = "";
    sprintf(buf, "%.2s-%05.2f%c", p, (f > 59.99 ? 59.99 : f), nswe);
    tmp_aprsLatPresetNiceNotation = String(buf);
  }
  // No, we now need this always (for remote-call position -> course/distance computation). Moved it upwards
  //if (!aprsLatLonInvalidPosition && aprsLatLonPresetCOMP.isEmpty()) {
    //sprintf(buf, "%.2s", p);
    //fLat = atof(buf) + f/60.0;
    //if (nswe == 'S')
    // fLat *= -1;
  //}

  // 001-20.5000E is more readable in the Web-interface than 00120.5000E, and could not be mis-interpreted as 120.5 degrees east  (== 120 deg 30' 0" E)
  p = sLon.c_str();
  nswe = p[strlen(p)-1];
  sprintf(buf, "%.7s", p+4);
  // strip trailing W/E. This way we could have a dynamic length for input precision at sLon
  buf[strlen(buf)-1] = 0;
  f = atof(buf);
  // round up. sprintf for float does the rounding
  sprintf(buf, "%.3s%05.2f%c", p, (f > 59.99 ? 59.99 : f), nswe);
  tmp_aprsLonPreset = String(buf);

  // p is degrees, f is minutes.decimal
  sprintf(buf, "%.3s", p);
  fLon = atof(buf) + f/60.0;
  if (nswe == 'W')
   fLon *= -1;

  if (!aprsLatLonInvalidPosition && sLon.length() > 10 && (precision == 1 || precision == 2)) {
    float fp = f;
    if (precision == 1) {
      if (fp > 59.999) fp=59.999;
    } else {
      if (fp > 59.9999) fp=59.9999;
    }
    if (precision == 1) {
      sprintf(buf, "%.3s%06.3f", p, fp);
      tmp_aprsLatLonDAO = tmp_aprsLatLonDAO + String(buf[8]) + "!";
    } else {
      sprintf(buf, "%.3s%07.4f", p, fp);
      ax25_base91enc(helper_base91, 1, atoi(buf+7)*0.91);
      tmp_aprsLatLonDAO = tmp_aprsLatLonDAO + helper_base91[0] + "!";
    }
    buf[8] = nswe; buf[9] = 0;;
    tmp_aprsLonPresetDAO = String(buf);
    // "NiceNotation" may be used for presenting at oled. Four decimals would be too hard to read. We use 3 decimals (see precision 1 above).
    // Furthermore, we don't have enough space on oled anyway for one additional character.
    sprintf(buf, "%.3s-%06.3f%c", p, (f > 59.999 ? 59.999 : f), nswe);
    tmp_aprsLonPresetNiceNotation = String(buf);
    tmp_aprsLatLonAsMaidenheadGridLocator = compute_maidenhead_grid_locator(tmp_aprsLatPresetNiceNotation, tmp_aprsLonPresetNiceNotation, 0);
  } else {
    tmp_aprsLonPresetDAO = tmp_aprsLonPreset;
    tmp_aprsLatLonDAO = "";
    sprintf(buf, "%.3s-%05.2f%c", p, (f > 59.99 ? 59.99 : f), nswe);
    tmp_aprsLonPresetNiceNotation = String(buf);
    tmp_aprsLatLonAsMaidenheadGridLocator = compute_maidenhead_grid_locator(tmp_aprsLatPresetNiceNotation, tmp_aprsLonPresetNiceNotation, 1);
  }
  // No, we now need this always (for remote-call position -> course/distance computation). Moved it upwards
  //if (!aprsLatLonInvalidPosition && aprsLatLonPresetCOMP.isEmpty()) {
    //sprintf(buf, "%.3s", p);
    //fLon = atof(buf) + f/60.0;
    //if (nswe == 'W')
     //fLon *= -1;
  //}

  tmp_aprsLatPresetNiceNotation.replace(".", ",");
  tmp_aprsLonPresetNiceNotation.replace(".", ",");

  // We worked with temporary variables. We needed to take special care with these public variables:
  // I.e. we might have been called by the webserver Thread (on save config). And our main thread might in the meantime use these
  // variables for displaying, TX, etc. -> Apply the changes right after each other.
  // String(tmp_...) enforces a new String object.
  aprsLatPresetDouble = fLat;
  aprsLonPresetDouble = fLon;
  aprsLatPreset = String(tmp_aprsLatPreset);
  aprsLonPreset = String(tmp_aprsLonPreset);
  aprsLatPresetDAO = String(tmp_aprsLatPresetDAO);
  aprsLonPresetDAO = String(tmp_aprsLonPresetDAO);
  aprsLatLonDAO = String(tmp_aprsLatLonDAO);
  aprsLatPresetNiceNotation = String(tmp_aprsLatPresetNiceNotation);
  aprsLonPresetNiceNotation = String(tmp_aprsLonPresetNiceNotation);
  aprsLatLonAsMaidenheadGridLocator = String(tmp_aprsLatLonAsMaidenheadGridLocator);
  // Only when called from a function called from setup_phase2_soft_reconfiguration (after boot oder config save) with preset position (indicator: aprsLatLonPresetCOMP is empty).
  // Else: store_compressed_position() is explicitely called right after this function
  if (!aprsLatLonInvalidPosition && aprsLatLonPresetCOMP.isEmpty())
    store_compressed_position(fLat, fLon);

  return 0;
}

void init_and_validate_aprs_position_and_icon() {

  // latlon_precision depends on position_ambiguity
  if (position_ambiguity == -2) {
    latlon_precision = 1;
  } else if (position_ambiguity == 0 || position_ambiguity == -3) {
    // compressed, and uncompressed DAO '!w..! have almost the same precision
    latlon_precision = 2;
  } else {
    // ambiguity -1 and >= 1
    latlon_precision = 0;
  }

  aprsLatLonInvalidPosition = true;
  aprsLatPresetFromPreferences.toUpperCase(); aprsLatPresetFromPreferences.replace(",", "."); aprsLatPresetFromPreferences.trim();
  aprsLonPresetFromPreferences.toUpperCase(); aprsLonPresetFromPreferences.replace(",", "."); aprsLonPresetFromPreferences.trim();
  if ( aprsLatPresetFromPreferences.length() == 11 &&
       aprsLatPresetFromPreferences.indexOf('-') == 2 && aprsLatPresetFromPreferences.indexOf(' ') == -1 &&
       (aprsLatPresetFromPreferences.endsWith("N") || aprsLatPresetFromPreferences.endsWith("S")) &&
       aprsLonPresetFromPreferences.length() == 12 &&
       aprsLonPresetFromPreferences.indexOf('-') == 3 && aprsLonPresetFromPreferences.indexOf(' ') == -1 &&
       (aprsLonPresetFromPreferences.endsWith("E") || aprsLonPresetFromPreferences.endsWith("W")) ) {
    // if gps is off, use configured location in high precision. If gps is on, be more precise later if gps has fix;
    // if gps never get a fix, we don't really know if we are really at that position, so don't be so exact here.
    aprsLatLonPresetCOMP = "";
    storeLatLonPreset(aprsLatPresetFromPreferences, aprsLonPresetFromPreferences, (!fixed_beacon_enabled && gps_state) ? 0 : latlon_precision);
  }

  // if storeLatLonPreset() was successful, our aprsLatPreset and aprsLonPreset variables are now in standard aprs notation,
  // 8 bytes in lat, 9 bytes in lon. Format 1234.56N 01234.56E. If not, mark it as null coordinate, as defined in aprs spec.

  // assure valid transmissions, even on wrong configurations
  if (aprsLatPreset.length() != 8 || !(aprsLatPreset.endsWith("N") || aprsLatPreset.endsWith("S")) || aprsLatPreset.c_str()[4] != '.' ||
      aprsLonPreset.length() != 9 || !(aprsLonPreset.endsWith("E") || aprsLonPreset.endsWith("W")) || aprsLonPreset.c_str()[5] != '.' ||
      aprsLatPresetDAO.length() != 8 || !(aprsLatPresetDAO.endsWith("N") || aprsLatPresetDAO.endsWith("S")) || aprsLatPresetDAO.c_str()[4] != '.' ||
      aprsLonPresetDAO.length() != 9 || !(aprsLonPresetDAO.endsWith("E") || aprsLonPresetDAO.endsWith("W")) || aprsLonPresetDAO.c_str()[5] != '.') {
    aprsLatLonPresetCOMP = "";
    aprsLatLonInvalidPosition = true;
    storeLatLonPreset("00-00.00N", "000-00.0000W", 0);
  }

  if (aprsSymbolTable.length() != 1)
    aprsSymbolTable = String("/");
  if (aprsSymbol.length() != 1)
    aprsSymbol = String("[");

  Serial.printf("APRS fixed position set to %s %s; icon: table %s symbol %s\r\n", aprsLatPreset.c_str(), aprsLonPreset.c_str(), aprsSymbolTable.c_str(), aprsSymbol.c_str());
}


String jsonElementFromPreferenceCFGString(const char *preferenceName, const char *preferenceNameInit){
  const char *p;
  String value;
  if ((p = JSONBuffer[preferenceName])) value = String(p);
  //if (preferenceNameInit) preferences.putBool(preferenceNameInit, true);
  //preferences.putString(preferenceName, value);
  Serial.println("getPreferences.cfg " + String(preferenceName) + ": " + value);
  return value;
}

int jsonElementFromPreferenceCFGInt(const char *preferenceName, const char *preferenceNameInit){
  int value_int = JSONBuffer[preferenceName];
  //if (preferenceNameInit) preferences.putBool(preferenceNameInit, true);
  //preferences.putInt(preferenceName, value);
  Serial.println("getPreferences.cfg " + String(preferenceName) + ": " + String(value_int));
  return value_int;
}

double jsonElementFromPreferenceCFGDouble(const char *preferenceName, const char *preferenceNameInit){
  double value_d = JSONBuffer[preferenceName];
  //if (preferenceNameInit) preferences.putBool(preferenceNameInit, true);
  //preferences.putDouble(preferenceName, value);
  Serial.printf("getPreferences.cfg %s: %8.4f\r\n",preferenceName, value_d);
  return value_d;
}

boolean jsonElementFromPreferenceCFGBool(const char *preferenceName, const char *preferenceNameInit){
  boolean value_b = JSONBuffer[preferenceName];;
  //if (preferenceNameInit) preferences.putBool(preferenceNameInit, true);
  //preferences.putBool(preferenceName, value);
  Serial.printf("getPreferences.cfg %s: %d\r\n",preferenceName,value_b);
  return value_b;
}


void load_preferences_cfg_file()
{
  String s = "";

#ifdef ENABLE_WIFI
  enable_webserver = jsonElementFromPreferenceCFGInt(PREF_WIFI_ENABLE,PREF_WIFI_ENABLE_INIT);
  tncServer_enabled = jsonElementFromPreferenceCFGBool(PREF_TNCSERVER_ENABLE,PREF_TNCSERVER_ENABLE_INIT);
  gpsServer_enabled = jsonElementFromPreferenceCFGBool(PREF_GPSSERVER_ENABLE,PREF_GPSSERVER_ENABLE_INIT);
  wifi_do_fallback_to_mode_AP = jsonElementFromPreferenceCFGBool(PREF_WIFI_STA_ALLOW_FAILBACK_TO_MODE_AP_AFTER_ONCE_CONNECTED,PREF_WIFI_STA_ALLOW_FAILBACK_TO_MODE_AP_AFTER_ONCE_CONNECTED_INIT);
  wifi_txpwr_mode_AP = jsonElementFromPreferenceCFGInt(PREF_WIFI_TXPWR_MODE_AP,PREF_WIFI_TXPWR_MODE_AP_INIT);
  wifi_txpwr_mode_STA = jsonElementFromPreferenceCFGInt(PREF_WIFI_TXPWR_MODE_STA,PREF_WIFI_TXPWR_MODE_STA_INIT);
  s = jsonElementFromPreferenceCFGString(PREF_SYSLOG_SERVER,0);
  preferences.putString(PREF_SYSLOG_SERVER, s);
  s = jsonElementFromPreferenceCFGString(PREF_NTP_SERVER,0);
  preferences.putString(PREF_NTP_SERVER, s);
  #if defined(ENABLE_SYSLOG)
    if (debug_verbose)
      syslog_log(LOG_DEBUG, String("FlashWrite preferences: load_preferences_cfg()"));
  #endif

#endif // ENABLE_WIFI
  lora_freq = jsonElementFromPreferenceCFGDouble(PREF_LORA_FREQ_PRESET,PREF_LORA_FREQ_PRESET_INIT);
  lora_speed = jsonElementFromPreferenceCFGInt(PREF_LORA_SPEED_PRESET,PREF_LORA_SPEED_PRESET_INIT);
  lora_rx_enabled = jsonElementFromPreferenceCFGBool(PREF_LORA_RX_ENABLE,PREF_LORA_RX_ENABLE_INIT);
  lora_tx_enabled = jsonElementFromPreferenceCFGBool(PREF_LORA_TX_ENABLE,PREF_LORA_TX_ENABLE_INIT);
  txPower = jsonElementFromPreferenceCFGInt(PREF_LORA_TX_POWER,PREF_LORA_TX_POWER_INIT);
  preambleLen = jsonElementFromPreferenceCFGInt(PREF_LORA_TX_PREAMBLE_LEN,PREF_LORA_TX_PREAMBLE_LEN_INIT);
  lora_automatic_cr_adaption = jsonElementFromPreferenceCFGBool(PREF_LORA_AUTOMATIC_CR_ADAPTION_PRESET,PREF_LORA_AUTOMATIC_CR_ADAPTION_PRESET_INIT);
  lora_add_snr_rssi_to_path = jsonElementFromPreferenceCFGInt(PREF_LORA_ADD_SNR_RSSI_TO_PATH_PRESET,PREF_LORA_ADD_SNR_RSSI_TO_PATH_PRESET_INIT);
  kiss_add_snr_rssi_to_path_at_position_without_digippeated_flag = jsonElementFromPreferenceCFGBool(PREF_LORA_ADD_SNR_RSSI_TO_PATH_END_AT_KISS_PRESET,PREF_LORA_ADD_SNR_RSSI_TO_PATH_END_AT_KISS_PRESET_INIT);
  lora_digipeating_mode = jsonElementFromPreferenceCFGBool(PREF_APRS_DIGIPEATING_MODE_PRESET,PREF_APRS_DIGIPEATING_MODE_PRESET_INIT);
  lora_digipeating_myalias = jsonElementFromPreferenceCFGString(PREF_APRS_DIGIPEATING_MYALIAS,PREF_APRS_DIGIPEATING_MYALIAS_INIT);
  lora_cross_digipeating_mode = jsonElementFromPreferenceCFGInt(PREF_APRS_CROSS_DIGIPEATING_MODE_PRESET,PREF_APRS_CROSS_DIGIPEATING_MODE_PRESET_INIT);
  tx_own_beacon_from_this_device_or_fromKiss__to_frequencies = jsonElementFromPreferenceCFGInt(PREF_LORA_TX_BEACON_AND_KISS_TO_FREQUENCIES_PRESET,PREF_LORA_TX_BEACON_AND_KISS_TO_FREQUENCIES_PRESET_INIT);
  send_status_message_to_aprsis = jsonElementFromPreferenceCFGBool(PREF_LORA_TX_STATUSMESSAGE_TO_APRSIS_PRESET,PREF_LORA_TX_STATUSMESSAGE_TO_APRSIS_PRESET_INIT);
  tx_own_beacon_from_this_device_or_fromKiss__to_aprsis = jsonElementFromPreferenceCFGBool(PREF_LORA_TX_BEACON_AND_KISS_TO_APRSIS_PRESET,PREF_LORA_TX_BEACON_AND_KISS_TO_APRSIS_PRESET_INIT);
  lora_freq_cross_digi = jsonElementFromPreferenceCFGDouble(PREF_LORA_FREQ_CROSSDIGI_PRESET,PREF_LORA_FREQ_CROSSDIGI_PRESET_INIT);
  lora_speed_cross_digi = jsonElementFromPreferenceCFGInt(PREF_LORA_SPEED_CROSSDIGI_PRESET,PREF_LORA_SPEED_CROSSDIGI_PRESET_INIT);
  txPower_cross_digi = jsonElementFromPreferenceCFGInt(PREF_LORA_TX_POWER_CROSSDIGI_PRESET,PREF_LORA_TX_POWER_CROSSDIGI_PRESET_INIT);
  rx_on_frequencies = jsonElementFromPreferenceCFGInt(PREF_LORA_RX_ON_FREQUENCIES_PRESET,PREF_LORA_RX_ON_FREQUENCIES_PRESET_INIT);

  aprsSymbolTable = jsonElementFromPreferenceCFGString(PREF_APRS_SYMBOL_TABLE,0);
  aprsSymbol = jsonElementFromPreferenceCFGString(PREF_APRS_SYMBOL,0);
  aprsObjectName = jsonElementFromPreferenceCFGString(PREF_APRS_OBJECT_NAME,0);
  aprsComment = jsonElementFromPreferenceCFGString(PREF_APRS_COMMENT,PREF_APRS_COMMENT_INIT);
  relay_path = jsonElementFromPreferenceCFGString(PREF_APRS_RELAY_PATH,PREF_APRS_RELAY_PATH_INIT);
  showAltitude = jsonElementFromPreferenceCFGBool(PREF_APRS_SHOW_ALTITUDE,PREF_APRS_SHOW_ALTITUDE_INIT);
  altitude_ratio = jsonElementFromPreferenceCFGInt(PREF_APRS_ALTITUDE_RATIO,PREF_APRS_ALTITUDE_RATIO);
  always_send_cseSpd_AND_altitude = jsonElementFromPreferenceCFGBool(PREF_APRS_ALWAYS_SEND_CSE_SPEED_AND_ALTITUDE,PREF_APRS_ALWAYS_SEND_CSE_SPEED_AND_ALTITUDE_INIT);
  gps_state = jsonElementFromPreferenceCFGBool(PREF_APRS_GPS_EN,PREF_APRS_GPS_EN_INIT);
  gps_may_sleep = jsonElementFromPreferenceCFGBool(PREF_GPS_POWERSAVE,PREF_GPS_POWERSAVE_INIT);
  acceptOwnPositionReportsViaKiss = jsonElementFromPreferenceCFGBool(PREF_ACCEPT_OWN_POSITION_REPORTS_VIA_KISS,PREF_ACCEPT_OWN_POSITION_REPORTS_VIA_KISS_INIT);
  gps_allow_sleep_while_kiss = jsonElementFromPreferenceCFGBool(PREF_GPS_ALLOW_SLEEP_WHILE_KISS,PREF_GPS_ALLOW_SLEEP_WHILE_KISS_INIT);
  showBattery = jsonElementFromPreferenceCFGBool(PREF_APRS_SHOW_BATTERY,PREF_APRS_SHOW_BATTERY_INIT);
  add_winlink_notification = jsonElementFromPreferenceCFGInt(PREF_APRS_STATUS_WINLINK_NOTIFICATION,PREF_APRS_STATUS_WINLINK_NOTIFICATION_INIT);
  send_status_message_about_shutdown_to_rf = jsonElementFromPreferenceCFGBool(PREF_APRS_STATUS_SHUTDOWN_NOTIFICATION,PREF_APRS_STATUS_SHUTDOWN_NOTIFICATION_INIT);
  enable_tel = jsonElementFromPreferenceCFGBool(PREF_ENABLE_TNC_SELF_TELEMETRY,PREF_ENABLE_TNC_SELF_TELEMETRY);
  tel_interval = jsonElementFromPreferenceCFGInt(PREF_TNC_SELF_TELEMETRY_INTERVAL,PREF_TNC_SELF_TELEMETRY_INTERVAL_INIT);
  tel_sequence = jsonElementFromPreferenceCFGInt(PREF_TNC_SELF_TELEMETRY_SEQ,PREF_TNC_SELF_TELEMETRY_SEQ_INIT);
  tel_mic = jsonElementFromPreferenceCFGInt(PREF_TNC_SELF_TELEMETRY_MIC,PREF_TNC_SELF_TELEMETRY_MIC_INIT);
  tel_path = jsonElementFromPreferenceCFGString(PREF_TNC_SELF_TELEMETRY_PATH,PREF_TNC_SELF_TELEMETRY_PATH_INIT);
  tel_allow_tx_on_rf = jsonElementFromPreferenceCFGInt(PREF_TNC_SELF_TELEMETRY_ALLOW_RF,PREF_TNC_SELF_TELEMETRY_ALLOW_RF_INIT);
  aprsLatPresetFromPreferences = jsonElementFromPreferenceCFGString(PREF_APRS_LATITUDE_PRESET,PREF_APRS_LATITUDE_PRESET_INIT);
  aprsLonPresetFromPreferences = jsonElementFromPreferenceCFGString(PREF_APRS_LONGITUDE_PRESET,PREF_APRS_LONGITUDE_PRESET_INIT);
  position_ambiguity = jsonElementFromPreferenceCFGInt(PREF_APRS_POSITION_AMBIGUITY,PREF_APRS_POSITION_AMBIGUITY_INIT);
  jsonElementFromPreferenceCFGString(PREF_APRS_SENDER_BLACKLIST,PREF_APRS_SENDER_BLACKLIST_INIT);
  fixed_beacon_enabled = jsonElementFromPreferenceCFGBool(PREF_APRS_FIXED_BEACON_PRESET,PREF_APRS_FIXED_BEACON_PRESET);
  fix_beacon_interval = jsonElementFromPreferenceCFGInt(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET,PREF_APRS_FIXED_BEACON_INTERVAL_PRESET_INIT) * 1000;

// + SMART BEACONING
  sb_min_interval = jsonElementFromPreferenceCFGInt(PREF_APRS_SB_MIN_INTERVAL_PRESET,PREF_APRS_SB_MIN_INTERVAL_PRESET_INIT) * 1000;
  if (sb_min_interval < 10000) sb_min_interval = 10000;
  sb_max_interval = jsonElementFromPreferenceCFGInt(PREF_APRS_SB_MAX_INTERVAL_PRESET,PREF_APRS_SB_MAX_INTERVAL_PRESET_INIT) * 1000;
  // sb max interval not < 90s.
  if (sb_max_interval < 90000L)
    sb_max_interval = 90000L;
  if (sb_max_interval <= sb_min_interval) sb_max_interval = sb_min_interval + 1000;
  sb_min_speed = (float) jsonElementFromPreferenceCFGInt(PREF_APRS_SB_MIN_SPEED_PRESET,PREF_APRS_SB_MIN_SPEED_PRESET_INIT);
  if (sb_min_speed < 0) sb_min_speed = 0;
  sb_max_speed = (float ) jsonElementFromPreferenceCFGInt(PREF_APRS_SB_MAX_SPEED_PRESET,PREF_APRS_SB_MAX_SPEED_PRESET_INIT);
  if (sb_max_speed <= sb_min_speed) sb_max_speed = sb_min_speed +1;
  sb_angle = jsonElementFromPreferenceCFGDouble(PREF_APRS_SB_ANGLE_PRESET,PREF_APRS_SB_ANGLE_PRESET_INIT);
  sb_turn_slope = jsonElementFromPreferenceCFGInt(PREF_APRS_SB_TURN_SLOPE_PRESET,PREF_APRS_SB_TURN_SLOPE_PRESET_INIT);
  sb_turn_time = jsonElementFromPreferenceCFGInt(PREF_APRS_SB_TURN_TIME_PRESET,PREF_APRS_SB_TURN_TIME_PRESET_INIT);
  showRXTime = jsonElementFromPreferenceCFGInt(PREF_DEV_SHOW_RX_TIME,PREF_DEV_SHOW_RX_TIME_INIT) * 1000;

// Read OLED RX Timer
  oled_timeout = jsonElementFromPreferenceCFGInt(PREF_DEV_SHOW_OLED_TIME,PREF_DEV_SHOW_OLED_TIME_INIT) * 1000;
  shutdown_delay_time = jsonElementFromPreferenceCFGInt(PREF_DEV_AUTO_SHUT_PRESET,PREF_DEV_AUTO_SHUT_PRESET_INIT) * 1000;
  shutdown_active = jsonElementFromPreferenceCFGBool(PREF_DEV_AUTO_SHUT,PREF_DEV_AUTO_SHUT_INIT);
  reboot_interval = (uint32_t ) jsonElementFromPreferenceCFGInt(PREF_DEV_REBOOT_INTERVAL,PREF_DEV_REBOOT_INTERVAL_INIT) *60*60*1000L;
  show_cmt = jsonElementFromPreferenceCFGBool(PREF_APRS_SHOW_CMT,PREF_APRS_SHOW_CMT_INIT);
  rate_limit_message_text = jsonElementFromPreferenceCFGBool(PREF_APRS_COMMENT_RATELIMIT_PRESET,PREF_APRS_COMMENT_RATELIMIT_PRESET_INIT);
#ifdef ENABLE_BLUETOOTH
   enable_bluetooth = jsonElementFromPreferenceCFGBool(PREF_DEV_BT_EN,PREF_DEV_BT_EN_INIT);
#endif
   // TOOD: verify if it's sufficient, due to the new key
   usb_serial_data_type = jsonElementFromPreferenceCFGInt(PREF_DEV_USBSERIAL_DATA_TYPE,PREF_DEV_USBSERIAL_DATA_TYPE_INIT);
   enabled_oled  = jsonElementFromPreferenceCFGBool(PREF_DEV_OL_EN,PREF_DEV_OL_EN_INIT);
   adjust_cpuFreq_to = jsonElementFromPreferenceCFGInt(PREF_DEV_CPU_FREQ,PREF_DEV_CPU_FREQ_INIT);
   units = jsonElementFromPreferenceCFGInt(PREF_DEV_UNITS,PREF_DEV_UNITS_INIT);
   oled_line3and4_format = jsonElementFromPreferenceCFGInt(PREF_DEV_OLED_L3_L4_FORMAT,PREF_DEV_OLED_L3_L4_FORMAT_INIT);
   oled_show_locator = jsonElementFromPreferenceCFGInt(PREF_DEV_OLED_LOCATOR,PREF_DEV_OLED_LOCATOR_INIT);
   oled_loc_amb = jsonElementFromPreferenceCFGInt(PREF_DEV_OLED_LOCATOR_AMBIGUITY,PREF_DEV_OLED_LOCATOR_AMBIGUITY_INIT);

// APRSIS settings
#ifdef ENABLE_WIFI
    aprsis_enabled = jsonElementFromPreferenceCFGBool(PREF_APRSIS_EN,PREF_APRSIS_EN_INIT);
    aprsis_host = jsonElementFromPreferenceCFGString(PREF_APRSIS_SERVER_NAME,PREF_APRSIS_SERVER_NAME_INIT);
    aprsis_port = jsonElementFromPreferenceCFGInt(PREF_APRSIS_SERVER_PORT,PREF_APRSIS_SERVER_PORT_INIT);
    aprsis_filter = jsonElementFromPreferenceCFGString(PREF_APRSIS_FILTER,PREF_APRSIS_FILTER_INIT);
    aprsis_own_filters_in = jsonElementFromPreferenceCFGString(PREF_APRSIS_FILTER_LOCAL_INCOMING,PREF_APRSIS_FILTER_LOCAL_INCOMING_INIT);
    aprsis_own_filters_words_in = jsonElementFromPreferenceCFGString(PREF_APRSIS_FILTER_LOCAL_WORDS_INCOMING,PREF_APRSIS_FILTER_LOCAL_WORDS_INCOMING_INIT);
    aprsis_callsign = jsonElementFromPreferenceCFGString(PREF_APRSIS_CALLSIGN,PREF_APRSIS_CALLSIGN_INIT);
    aprsis_password = jsonElementFromPreferenceCFGString(PREF_APRSIS_PASSWORD,PREF_APRSIS_PASSWORD_INIT);
    aprsis_data_allow_inet_to_rf = jsonElementFromPreferenceCFGInt(PREF_APRSIS_ALLOW_INET_TO_RF,PREF_APRSIS_ALLOW_INET_TO_RF_INIT);
#endif

}


#ifdef ENABLE_PREFERENCES
// This function loads values from saved preferences (from flash), if available.
// https://randomnerdtutorials.com/esp32-save-data-permanently-preferences/

void load_preferences_from_flash()
{
#ifdef ENABLE_WIFI
    if (!preferences.getBool(PREF_WIFI_ENABLE_INIT)){
      preferences.putBool(PREF_WIFI_ENABLE_INIT, true);
      preferences.putInt(PREF_WIFI_ENABLE, enable_webserver);
    }
    enable_webserver = preferences.getInt(PREF_WIFI_ENABLE);

    if (!preferences.getBool(PREF_TNCSERVER_ENABLE_INIT)){
      preferences.putBool(PREF_TNCSERVER_ENABLE_INIT, true);
      preferences.putBool(PREF_TNCSERVER_ENABLE, tncServer_enabled);
    }
    tncServer_enabled = preferences.getBool(PREF_TNCSERVER_ENABLE);

    if (!preferences.getBool(PREF_GPSSERVER_ENABLE_INIT)){
      preferences.putBool(PREF_GPSSERVER_ENABLE_INIT, true);
      preferences.putBool(PREF_GPSSERVER_ENABLE, gpsServer_enabled);
    }
    gpsServer_enabled = preferences.getBool(PREF_GPSSERVER_ENABLE);

    if (!preferences.getBool(PREF_WIFI_STA_ALLOW_FAILBACK_TO_MODE_AP_AFTER_ONCE_CONNECTED_INIT)){
      preferences.putBool(PREF_WIFI_STA_ALLOW_FAILBACK_TO_MODE_AP_AFTER_ONCE_CONNECTED_INIT, true);
      preferences.putBool(PREF_WIFI_STA_ALLOW_FAILBACK_TO_MODE_AP_AFTER_ONCE_CONNECTED, wifi_do_fallback_to_mode_AP);
    }
    wifi_do_fallback_to_mode_AP = preferences.getBool(PREF_WIFI_STA_ALLOW_FAILBACK_TO_MODE_AP_AFTER_ONCE_CONNECTED);

    if (!preferences.getBool(PREF_WIFI_TXPWR_MODE_AP_INIT)){
      preferences.putBool(PREF_WIFI_TXPWR_MODE_AP_INIT, true);
      preferences.putInt(PREF_WIFI_TXPWR_MODE_AP, wifi_txpwr_mode_AP);
    }
    wifi_txpwr_mode_AP = preferences.getInt(PREF_WIFI_TXPWR_MODE_AP);

    if (!preferences.getBool(PREF_WIFI_TXPWR_MODE_STA_INIT)){
      preferences.putBool(PREF_WIFI_TXPWR_MODE_STA_INIT, true);
      preferences.putInt(PREF_WIFI_TXPWR_MODE_STA, wifi_txpwr_mode_STA);
    }
    wifi_txpwr_mode_STA = preferences.getInt(PREF_WIFI_TXPWR_MODE_STA);
#endif // ENABLE_WIFI

    // LoRa transmission settings

    if (!preferences.getBool(PREF_LORA_FREQ_PRESET_INIT)){
      preferences.putBool(PREF_LORA_FREQ_PRESET_INIT, true);
      preferences.putDouble(PREF_LORA_FREQ_PRESET, lora_freq);
    }
    lora_freq = preferences.getDouble(PREF_LORA_FREQ_PRESET);

    if (!preferences.getBool(PREF_LORA_SPEED_PRESET_INIT)){
      preferences.putBool(PREF_LORA_SPEED_PRESET_INIT, true);
      preferences.putInt(PREF_LORA_SPEED_PRESET, lora_speed);
    }
    lora_speed = preferences.getInt(PREF_LORA_SPEED_PRESET);

    if (!preferences.getBool(PREF_LORA_RX_ENABLE_INIT)){
      preferences.putBool(PREF_LORA_RX_ENABLE_INIT, true);
      preferences.putBool(PREF_LORA_RX_ENABLE, lora_rx_enabled);
    }
    lora_rx_enabled = preferences.getBool(PREF_LORA_RX_ENABLE);

    if (!preferences.getBool(PREF_LORA_TX_ENABLE_INIT)){
      preferences.putBool(PREF_LORA_TX_ENABLE_INIT, true);
      preferences.putBool(PREF_LORA_TX_ENABLE, lora_tx_enabled);
    }
    lora_tx_enabled = preferences.getBool(PREF_LORA_TX_ENABLE);

    if (!preferences.getBool(PREF_LORA_TX_POWER_INIT)){
      preferences.putBool(PREF_LORA_TX_POWER_INIT, true);
      preferences.putInt(PREF_LORA_TX_POWER, txPower);
    }
    txPower = lora_tx_enabled ? preferences.getInt(PREF_LORA_TX_POWER) : 0;

    if (!preferences.getBool(PREF_LORA_TX_PREAMBLE_LEN_INIT)){
      preferences.putBool(PREF_LORA_TX_PREAMBLE_LEN_INIT, true);
      preferences.putInt(PREF_LORA_TX_PREAMBLE_LEN, preambleLen);
    }
    preambleLen = preferences.getInt(PREF_LORA_TX_PREAMBLE_LEN);

    if (!preferences.getBool(PREF_LORA_AUTOMATIC_CR_ADAPTION_PRESET_INIT)){
      preferences.putBool(PREF_LORA_AUTOMATIC_CR_ADAPTION_PRESET_INIT, true);
      preferences.putBool(PREF_LORA_AUTOMATIC_CR_ADAPTION_PRESET, lora_automatic_cr_adaption);
    }
    lora_automatic_cr_adaption = preferences.getBool(PREF_LORA_AUTOMATIC_CR_ADAPTION_PRESET);

    if (!preferences.getBool(PREF_LORA_ADD_SNR_RSSI_TO_PATH_PRESET_INIT)){
      preferences.putBool(PREF_LORA_ADD_SNR_RSSI_TO_PATH_PRESET_INIT, true);
      preferences.putInt(PREF_LORA_ADD_SNR_RSSI_TO_PATH_PRESET, lora_add_snr_rssi_to_path);
    }
    lora_add_snr_rssi_to_path = preferences.getInt(PREF_LORA_ADD_SNR_RSSI_TO_PATH_PRESET);

    if (!preferences.getBool(PREF_LORA_ADD_SNR_RSSI_TO_PATH_END_AT_KISS_PRESET_INIT)){
      preferences.putBool(PREF_LORA_ADD_SNR_RSSI_TO_PATH_END_AT_KISS_PRESET_INIT, true);
      preferences.putBool(PREF_LORA_ADD_SNR_RSSI_TO_PATH_END_AT_KISS_PRESET, kiss_add_snr_rssi_to_path_at_position_without_digippeated_flag);
    }
    kiss_add_snr_rssi_to_path_at_position_without_digippeated_flag = preferences.getBool(PREF_LORA_ADD_SNR_RSSI_TO_PATH_END_AT_KISS_PRESET);

    if (!preferences.getBool(PREF_APRS_DIGIPEATING_MODE_PRESET_INIT)){
      preferences.putBool(PREF_APRS_DIGIPEATING_MODE_PRESET_INIT, true);
      preferences.putInt(PREF_APRS_DIGIPEATING_MODE_PRESET, lora_digipeating_mode);
    }
    lora_digipeating_mode = preferences.getInt(PREF_APRS_DIGIPEATING_MODE_PRESET);

    if (!preferences.getBool(PREF_APRS_DIGIPEATING_MYALIAS_INIT)){
      preferences.putBool(PREF_APRS_DIGIPEATING_MYALIAS_INIT, true);
      preferences.putString(PREF_APRS_DIGIPEATING_MYALIAS, lora_digipeating_myalias);
    }
    lora_digipeating_myalias = preferences.getString(PREF_APRS_DIGIPEATING_MYALIAS);

    if (!preferences.getBool(PREF_APRS_CROSS_DIGIPEATING_MODE_PRESET_INIT)){
      preferences.putBool(PREF_APRS_CROSS_DIGIPEATING_MODE_PRESET_INIT, true);
      preferences.putInt(PREF_APRS_CROSS_DIGIPEATING_MODE_PRESET, lora_cross_digipeating_mode);
    }
    lora_cross_digipeating_mode = preferences.getInt(PREF_APRS_CROSS_DIGIPEATING_MODE_PRESET);

    if (!preferences.getBool(PREF_LORA_TX_BEACON_AND_KISS_TO_FREQUENCIES_PRESET_INIT)){
      preferences.putBool(PREF_LORA_TX_BEACON_AND_KISS_TO_FREQUENCIES_PRESET_INIT, true);
      preferences.putInt(PREF_LORA_TX_BEACON_AND_KISS_TO_FREQUENCIES_PRESET, tx_own_beacon_from_this_device_or_fromKiss__to_frequencies);
    }
    tx_own_beacon_from_this_device_or_fromKiss__to_frequencies = preferences.getInt(PREF_LORA_TX_BEACON_AND_KISS_TO_FREQUENCIES_PRESET);

    if (!preferences.getBool(PREF_LORA_TX_STATUSMESSAGE_TO_APRSIS_PRESET_INIT)){
      preferences.putBool(PREF_LORA_TX_STATUSMESSAGE_TO_APRSIS_PRESET_INIT, true);
      preferences.putBool(PREF_LORA_TX_STATUSMESSAGE_TO_APRSIS_PRESET, send_status_message_to_aprsis);
    }
    send_status_message_to_aprsis = preferences.getBool(PREF_LORA_TX_STATUSMESSAGE_TO_APRSIS_PRESET);

    if (!preferences.getBool(PREF_LORA_TX_BEACON_AND_KISS_TO_APRSIS_PRESET_INIT)){
      preferences.putBool(PREF_LORA_TX_BEACON_AND_KISS_TO_APRSIS_PRESET_INIT, true);
      preferences.putBool(PREF_LORA_TX_BEACON_AND_KISS_TO_APRSIS_PRESET, tx_own_beacon_from_this_device_or_fromKiss__to_aprsis);
    }
    tx_own_beacon_from_this_device_or_fromKiss__to_aprsis = preferences.getBool(PREF_LORA_TX_BEACON_AND_KISS_TO_APRSIS_PRESET);

    if (!preferences.getBool(PREF_LORA_FREQ_CROSSDIGI_PRESET_INIT)){
      preferences.putBool(PREF_LORA_FREQ_CROSSDIGI_PRESET_INIT, true);
      preferences.putDouble(PREF_LORA_FREQ_CROSSDIGI_PRESET, lora_freq_cross_digi);
    }
    lora_freq_cross_digi = preferences.getDouble(PREF_LORA_FREQ_CROSSDIGI_PRESET);

    if (!preferences.getBool(PREF_LORA_SPEED_CROSSDIGI_PRESET_INIT)){
      preferences.putBool(PREF_LORA_SPEED_CROSSDIGI_PRESET_INIT, true);
      preferences.putInt(PREF_LORA_SPEED_CROSSDIGI_PRESET, lora_speed_cross_digi);
    }
    lora_speed_cross_digi = preferences.getInt(PREF_LORA_SPEED_CROSSDIGI_PRESET);

    if (!preferences.getBool(PREF_LORA_TX_POWER_CROSSDIGI_PRESET_INIT)){
      preferences.putBool(PREF_LORA_TX_POWER_CROSSDIGI_PRESET_INIT, true);
      preferences.putInt(PREF_LORA_TX_POWER_CROSSDIGI_PRESET, txPower_cross_digi);
    }
    txPower_cross_digi = lora_tx_enabled ? preferences.getInt(PREF_LORA_TX_POWER_CROSSDIGI_PRESET) : 0;

    if (!preferences.getBool(PREF_LORA_RX_ON_FREQUENCIES_PRESET_INIT)){
      preferences.putBool(PREF_LORA_RX_ON_FREQUENCIES_PRESET_INIT, true);
      preferences.putInt(PREF_LORA_RX_ON_FREQUENCIES_PRESET, rx_on_frequencies);
    }
    rx_on_frequencies = preferences.getInt(PREF_LORA_RX_ON_FREQUENCIES_PRESET);

    // APRS station settings

    aprsSymbolTable = preferences.getString(PREF_APRS_SYMBOL_TABLE, "");
    if (aprsSymbolTable.isEmpty()){
      preferences.putString(PREF_APRS_SYMBOL_TABLE, aprsSymbolTable.length() != 1 ? APRS_SYMBOL_TABLE : aprsSymbolTable);
      aprsSymbolTable = preferences.getString(PREF_APRS_SYMBOL_TABLE);
    }

    aprsSymbol = preferences.getString(PREF_APRS_SYMBOL, "");
    if (aprsSymbol.isEmpty()){
      preferences.putString(PREF_APRS_SYMBOL, APRS_SYMBOL);
      aprsSymbol = preferences.getString(PREF_APRS_SYMBOL, aprsSymbol.length() != 1 ? APRS_SYMBOL : aprsSymbol);
    }

    if (!preferences.getBool(PREF_APRS_OBJECT_NAME_INIT)){
      preferences.putBool(PREF_APRS_OBJECT_NAME_INIT, true);
      preferences.putString(PREF_APRS_OBJECT_NAME, aprsObjectName);
    }
    aprsObjectName = preferences.getString(PREF_APRS_OBJECT_NAME, "");

    if (!preferences.getBool(PREF_APRS_COMMENT_INIT)){
      preferences.putBool(PREF_APRS_COMMENT_INIT, true);
      preferences.putString(PREF_APRS_COMMENT, aprsComment);
    }
    aprsComment = preferences.getString(PREF_APRS_COMMENT, "");

    if (!preferences.getBool(PREF_APRS_RELAY_PATH_INIT)){
      preferences.putBool(PREF_APRS_RELAY_PATH_INIT, true);
      preferences.putString(PREF_APRS_RELAY_PATH, relay_path);
    }
    relay_path = preferences.getString(PREF_APRS_RELAY_PATH, "");

    if (!preferences.getBool(PREF_APRS_SHOW_ALTITUDE_INIT)){
      preferences.putBool(PREF_APRS_SHOW_ALTITUDE_INIT, true);
      preferences.putBool(PREF_APRS_SHOW_ALTITUDE, showAltitude);
    }
    showAltitude = preferences.getBool(PREF_APRS_SHOW_ALTITUDE);

    if (!preferences.getBool(PREF_APRS_ALTITUDE_RATIO_INIT)){
      preferences.putBool(PREF_APRS_ALTITUDE_RATIO_INIT, true);
      // preferences.putInt(PREF_APRS_ALTITUDE_RATIO, altitude_ratio); // until SHOW_ALTITUDE is obsolete, commented out
      preferences.putInt(PREF_APRS_ALTITUDE_RATIO, showAltitude ? 100 : 0);
    }
    altitude_ratio = preferences.getInt(PREF_APRS_ALTITUDE_RATIO);

    if (!preferences.getBool(PREF_APRS_ALWAYS_SEND_CSE_SPEED_AND_ALTITUDE_INIT)){
      preferences.putBool(PREF_APRS_ALWAYS_SEND_CSE_SPEED_AND_ALTITUDE_INIT, true);
      preferences.putBool(PREF_APRS_ALWAYS_SEND_CSE_SPEED_AND_ALTITUDE, always_send_cseSpd_AND_altitude);
    }
    always_send_cseSpd_AND_altitude = preferences.getBool(PREF_APRS_ALWAYS_SEND_CSE_SPEED_AND_ALTITUDE);

    if (!preferences.getBool(PREF_APRS_GPS_EN_INIT)){
      preferences.putBool(PREF_APRS_GPS_EN_INIT, true);
      preferences.putBool(PREF_APRS_GPS_EN, gps_state);
    }
    gps_state = preferences.getBool(PREF_APRS_GPS_EN);

    if (!preferences.getBool(PREF_GPS_POWERSAVE_INIT)){
      preferences.putBool(PREF_GPS_POWERSAVE_INIT, true);
      preferences.putInt(PREF_GPS_POWERSAVE, gps_may_sleep);
    }
    gps_may_sleep = preferences.getInt(PREF_GPS_POWERSAVE);

    if (!preferences.getBool(PREF_ACCEPT_OWN_POSITION_REPORTS_VIA_KISS_INIT)){
      preferences.putBool(PREF_ACCEPT_OWN_POSITION_REPORTS_VIA_KISS_INIT, true);
      preferences.putBool(PREF_ACCEPT_OWN_POSITION_REPORTS_VIA_KISS, acceptOwnPositionReportsViaKiss);
    }
    acceptOwnPositionReportsViaKiss = preferences.getBool(PREF_ACCEPT_OWN_POSITION_REPORTS_VIA_KISS);

    if (!preferences.getBool(PREF_GPS_ALLOW_SLEEP_WHILE_KISS_INIT)){
      preferences.putBool(PREF_GPS_ALLOW_SLEEP_WHILE_KISS_INIT, true);
      preferences.putBool(PREF_GPS_ALLOW_SLEEP_WHILE_KISS, gps_allow_sleep_while_kiss);
    }
    gps_allow_sleep_while_kiss = preferences.getBool(PREF_GPS_ALLOW_SLEEP_WHILE_KISS);


    if (!preferences.getBool(PREF_APRS_SHOW_BATTERY_INIT)){
      preferences.putBool(PREF_APRS_SHOW_BATTERY_INIT, true);
      preferences.putBool(PREF_APRS_SHOW_BATTERY, showBattery);
    }
    showBattery = preferences.getBool(PREF_APRS_SHOW_BATTERY);

    if (!preferences.getBool(PREF_APRS_STATUS_WINLINK_NOTIFICATION_INIT)){
      preferences.putBool(PREF_APRS_STATUS_WINLINK_NOTIFICATION_INIT, true);
      preferences.putInt(PREF_APRS_STATUS_WINLINK_NOTIFICATION, add_winlink_notification);
    }
    add_winlink_notification = preferences.getInt(PREF_APRS_STATUS_WINLINK_NOTIFICATION);

    if (!preferences.getBool(PREF_APRS_STATUS_SHUTDOWN_NOTIFICATION_INIT)){
      preferences.putBool(PREF_APRS_STATUS_SHUTDOWN_NOTIFICATION_INIT, true);
      preferences.putBool(PREF_APRS_STATUS_SHUTDOWN_NOTIFICATION, send_status_message_about_shutdown_to_rf);
    }
    send_status_message_about_shutdown_to_rf = preferences.getBool(PREF_APRS_STATUS_SHUTDOWN_NOTIFICATION);

    if (!preferences.getBool(PREF_ENABLE_TNC_SELF_TELEMETRY_INIT)){
      preferences.putBool(PREF_ENABLE_TNC_SELF_TELEMETRY_INIT, true);
      preferences.putBool(PREF_ENABLE_TNC_SELF_TELEMETRY, enable_tel);
    }
    enable_tel = preferences.getBool(PREF_ENABLE_TNC_SELF_TELEMETRY);

    if (!preferences.getBool(PREF_TNC_SELF_TELEMETRY_INTERVAL_INIT)){
      preferences.putBool(PREF_TNC_SELF_TELEMETRY_INTERVAL_INIT, true);
      preferences.putInt(PREF_TNC_SELF_TELEMETRY_INTERVAL, tel_interval);
    }
    tel_interval = preferences.getInt(PREF_TNC_SELF_TELEMETRY_INTERVAL);

    if (!preferences.getBool(PREF_TNC_SELF_TELEMETRY_SEQ_INIT)){
      preferences.putBool(PREF_TNC_SELF_TELEMETRY_SEQ_INIT, true);
      preferences.putInt(PREF_TNC_SELF_TELEMETRY_SEQ, tel_sequence);
    }
    tel_sequence = preferences.getInt(PREF_TNC_SELF_TELEMETRY_SEQ);

    if (!preferences.getBool(PREF_TNC_SELF_TELEMETRY_MIC_INIT)){
      preferences.putBool(PREF_TNC_SELF_TELEMETRY_MIC_INIT, true);
      preferences.putInt(PREF_TNC_SELF_TELEMETRY_MIC, tel_mic);
    }
    tel_mic = preferences.getInt(PREF_TNC_SELF_TELEMETRY_MIC);

    if (!preferences.getBool(PREF_TNC_SELF_TELEMETRY_PATH_INIT)){
      preferences.putBool(PREF_TNC_SELF_TELEMETRY_PATH_INIT, true);
      preferences.putString(PREF_TNC_SELF_TELEMETRY_PATH, tel_path);
    }
    tel_path = preferences.getString(PREF_TNC_SELF_TELEMETRY_PATH, "");

    if (!preferences.getBool(PREF_TNC_SELF_TELEMETRY_ALLOW_RF_INIT)){
      preferences.putBool(PREF_TNC_SELF_TELEMETRY_ALLOW_RF_INIT, true);
      preferences.putInt(PREF_TNC_SELF_TELEMETRY_ALLOW_RF, tel_allow_tx_on_rf);
    }
    tel_allow_tx_on_rf = preferences.getInt(PREF_TNC_SELF_TELEMETRY_ALLOW_RF);

    if (!preferences.getBool(PREF_APRS_LATITUDE_PRESET_INIT)){
      preferences.putBool(PREF_APRS_LATITUDE_PRESET_INIT, true);
      preferences.putString(PREF_APRS_LATITUDE_PRESET, aprsLatPresetFromPreferences.isEmpty() ? LATITUDE_PRESET : aprsLatPresetFromPreferences);
    }
    aprsLatPresetFromPreferences = preferences.getString(PREF_APRS_LATITUDE_PRESET, "");
    //LatShownP = aprsLonPreset;

    if (!preferences.getBool(PREF_APRS_LONGITUDE_PRESET_INIT)){
      preferences.putBool(PREF_APRS_LONGITUDE_PRESET_INIT, true);
      preferences.putString(PREF_APRS_LONGITUDE_PRESET, aprsLonPresetFromPreferences.isEmpty() ? LONGITUDE_PRESET : aprsLonPresetFromPreferences);
    }
    aprsLonPresetFromPreferences = preferences.getString(PREF_APRS_LONGITUDE_PRESET, "");
    //LongShownP = aprsLonPreset;

    if (!preferences.getBool(PREF_APRS_POSITION_AMBIGUITY_INIT)){
      preferences.putBool(PREF_APRS_POSITION_AMBIGUITY_INIT, true);
      preferences.putInt(PREF_APRS_POSITION_AMBIGUITY, position_ambiguity);
    }
    position_ambiguity = preferences.getInt(PREF_APRS_POSITION_AMBIGUITY);

    if (!preferences.getBool(PREF_APRS_SENDER_BLACKLIST_INIT)){
      preferences.putBool(PREF_APRS_SENDER_BLACKLIST_INIT, true);
      preferences.putString(PREF_APRS_SENDER_BLACKLIST, "");
    }
    { String s = preferences.getString(PREF_APRS_SENDER_BLACKLIST, "");
      s.toUpperCase(); s.trim(); s.replace(" ", ","); s.replace(",,", ",");
      if (!s.isEmpty() && s != "," && s.length() < sizeof(blacklist_calls)-3) {
        *blacklist_calls = ',';
        strcpy(blacklist_calls+1, s.c_str());
        strcat(blacklist_calls, ",");
      } else {
        *blacklist_calls = 0;
      }
    }

    if (!preferences.getBool(PREF_APRS_FIXED_BEACON_PRESET_INIT)){
      preferences.putBool(PREF_APRS_FIXED_BEACON_PRESET_INIT, true);
      preferences.putBool(PREF_APRS_FIXED_BEACON_PRESET, fixed_beacon_enabled);
    }
    fixed_beacon_enabled = preferences.getBool(PREF_APRS_FIXED_BEACON_PRESET);

    if (!preferences.getBool(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET_INIT)){
      preferences.putBool(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET_INIT, true);
      preferences.putInt(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET, fix_beacon_interval/1000);
    }
    fix_beacon_interval = preferences.getInt(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET) * 1000;

// SMART BEACONING

    if (!preferences.getBool(PREF_APRS_SB_MIN_INTERVAL_PRESET_INIT)){
      preferences.putBool(PREF_APRS_SB_MIN_INTERVAL_PRESET_INIT, true);
      preferences.putInt(PREF_APRS_SB_MIN_INTERVAL_PRESET, sb_min_interval/1000);
    }
    sb_min_interval = preferences.getInt(PREF_APRS_SB_MIN_INTERVAL_PRESET) * 1000;
    if (sb_min_interval < 10000) sb_min_interval = 10000;

    if (!preferences.getBool(PREF_APRS_SB_MAX_INTERVAL_PRESET_INIT)){
      preferences.putBool(PREF_APRS_SB_MAX_INTERVAL_PRESET_INIT, true);
      preferences.putInt(PREF_APRS_SB_MAX_INTERVAL_PRESET, sb_max_interval/1000);
    }
    sb_max_interval = preferences.getInt(PREF_APRS_SB_MAX_INTERVAL_PRESET) * 1000;
    // sb max interval not < 90s.
    if (sb_max_interval < 90000L)
      sb_max_interval = 90000L;
    if (sb_max_interval <= sb_min_interval) sb_max_interval = sb_min_interval + 1000;

    if (!preferences.getBool(PREF_APRS_SB_MIN_SPEED_PRESET_INIT)){
      preferences.putBool(PREF_APRS_SB_MIN_SPEED_PRESET_INIT, true);
      preferences.putInt(PREF_APRS_SB_MIN_SPEED_PRESET, sb_min_speed);
    }
    sb_min_speed = (float) preferences.getInt(PREF_APRS_SB_MIN_SPEED_PRESET);
    if (sb_min_speed < 0) sb_min_speed = 0;

    if (!preferences.getBool(PREF_APRS_SB_MAX_SPEED_PRESET_INIT)){
      preferences.putBool(PREF_APRS_SB_MAX_SPEED_PRESET_INIT, true);
      preferences.putInt(PREF_APRS_SB_MAX_SPEED_PRESET, sb_max_speed);
    }
    sb_max_speed = (float ) preferences.getInt(PREF_APRS_SB_MAX_SPEED_PRESET);
    if (sb_max_speed <= sb_min_speed) sb_max_speed = sb_min_speed +1;

    if (!preferences.getBool(PREF_APRS_SB_ANGLE_PRESET_INIT)){
      preferences.putBool(PREF_APRS_SB_ANGLE_PRESET_INIT, true);
      preferences.putDouble(PREF_APRS_SB_ANGLE_PRESET, sb_angle);
    }
    sb_angle = preferences.getDouble(PREF_APRS_SB_ANGLE_PRESET);

    if (!preferences.getBool(PREF_APRS_SB_TURN_SLOPE_PRESET_INIT)){
      preferences.putBool(PREF_APRS_SB_TURN_SLOPE_PRESET_INIT, true);
      preferences.putInt(PREF_APRS_SB_TURN_SLOPE_PRESET, sb_turn_slope);
    }
    sb_turn_slope = preferences.getInt(PREF_APRS_SB_TURN_SLOPE_PRESET);

    if (!preferences.getBool(PREF_APRS_SB_TURN_TIME_PRESET_INIT)){
      preferences.putBool(PREF_APRS_SB_TURN_TIME_PRESET_INIT, true);
      preferences.putInt(PREF_APRS_SB_TURN_TIME_PRESET, sb_turn_time);
    }
    sb_turn_time = preferences.getInt(PREF_APRS_SB_TURN_TIME_PRESET);

//

    // Read OLED RX Timer
    if (!preferences.getBool(PREF_DEV_SHOW_RX_TIME_INIT)){
      preferences.putBool(PREF_DEV_SHOW_RX_TIME_INIT, true);
      preferences.putInt(PREF_DEV_SHOW_RX_TIME, showRXTime/1000);
    }
    showRXTime = preferences.getInt(PREF_DEV_SHOW_RX_TIME) * 1000;

    // Read OLED Timeout
    if (!preferences.getBool(PREF_DEV_SHOW_OLED_TIME_INIT)){
      preferences.putBool(PREF_DEV_SHOW_OLED_TIME_INIT, true);
      preferences.putInt(PREF_DEV_SHOW_OLED_TIME, oled_timeout/1000);
    }
    oled_timeout = preferences.getInt(PREF_DEV_SHOW_OLED_TIME) * 1000;

    if (!preferences.getBool(PREF_DEV_AUTO_SHUT_PRESET_INIT)){
      preferences.putBool(PREF_DEV_AUTO_SHUT_PRESET_INIT, true);
      preferences.putInt(PREF_DEV_AUTO_SHUT_PRESET, shutdown_delay_time/1000);
    }
    shutdown_delay_time = preferences.getInt(PREF_DEV_AUTO_SHUT_PRESET) * 1000;

    if (!preferences.getBool(PREF_DEV_AUTO_SHUT_INIT)){
      preferences.putBool(PREF_DEV_AUTO_SHUT_INIT, true);
      preferences.putBool(PREF_DEV_AUTO_SHUT, shutdown_active);
    }
    shutdown_active = preferences.getBool(PREF_DEV_AUTO_SHUT);

    if (!preferences.getBool(PREF_DEV_REBOOT_INTERVAL_INIT)){
      preferences.putBool(PREF_DEV_REBOOT_INTERVAL_INIT, true);
      preferences.putInt(PREF_DEV_REBOOT_INTERVAL, reboot_interval/60/60/1000);
    }
    reboot_interval = (uint32_t ) preferences.getInt(PREF_DEV_REBOOT_INTERVAL) *60*60*1000L;

    if (!preferences.getBool(PREF_APRS_SHOW_CMT_INIT)){
      preferences.putBool(PREF_APRS_SHOW_CMT_INIT, true);
      preferences.putBool(PREF_APRS_SHOW_CMT, show_cmt);
    }
    show_cmt = preferences.getBool(PREF_APRS_SHOW_CMT);

    if (!preferences.getBool(PREF_APRS_COMMENT_RATELIMIT_PRESET_INIT)){
      preferences.putBool(PREF_APRS_COMMENT_RATELIMIT_PRESET_INIT, true);
      preferences.putBool(PREF_APRS_COMMENT_RATELIMIT_PRESET, rate_limit_message_text);
    }
    rate_limit_message_text = preferences.getBool(PREF_APRS_COMMENT_RATELIMIT_PRESET);

    if (!preferences.getBool(PREF_DEV_BT_EN_INIT)){
      preferences.putBool(PREF_DEV_BT_EN_INIT, true);
      preferences.putBool(PREF_DEV_BT_EN, enable_bluetooth);
    }
    #ifdef ENABLE_BLUETOOTH
      enable_bluetooth = preferences.getBool(PREF_DEV_BT_EN);
    #endif

    // remove old usb-serial-debug-log preference (-> switch over)
    if (preferences.getBool(PREF_DEV_LOGTOSERIAL_EN_INIT)){
      usb_serial_data_type = preferences.getBool(PREF_DEV_LOGTOSERIAL_EN) ? 1 : 0;
      preferences.remove(PREF_DEV_LOGTOSERIAL_EN_INIT);
      preferences.remove(PREF_DEV_LOGTOSERIAL_EN);
    } else {
      if (!preferences.getBool(PREF_DEV_USBSERIAL_DATA_TYPE_INIT)){
        preferences.putBool(PREF_DEV_USBSERIAL_DATA_TYPE_INIT, true);
        preferences.putInt(PREF_DEV_USBSERIAL_DATA_TYPE, usb_serial_data_type);
      }
      usb_serial_data_type = preferences.getInt(PREF_DEV_USBSERIAL_DATA_TYPE, usb_serial_data_type);
    }

    if (!preferences.getBool(PREF_DEV_OL_EN_INIT)){
      preferences.putBool(PREF_DEV_OL_EN_INIT, true);
      preferences.putBool(PREF_DEV_OL_EN,enabled_oled);
    }
    enabled_oled  = preferences.getBool(PREF_DEV_OL_EN);

    if (!preferences.getBool(PREF_DEV_CPU_FREQ_INIT)){
      preferences.putBool(PREF_DEV_CPU_FREQ_INIT, true);
      preferences.putInt(PREF_DEV_CPU_FREQ, adjust_cpuFreq_to);
    }
    adjust_cpuFreq_to = preferences.getInt(PREF_DEV_CPU_FREQ);

    if (!preferences.getBool(PREF_DEV_UNITS_INIT)){
      preferences.putBool(PREF_DEV_UNITS_INIT, true);
      preferences.putInt(PREF_DEV_UNITS, units);
    }
    units = preferences.getInt(PREF_DEV_UNITS);

    if (!preferences.getBool(PREF_DEV_OLED_L3_L4_FORMAT_INIT)){
      preferences.putBool(PREF_DEV_OLED_L3_L4_FORMAT_INIT, true);
      preferences.putInt(PREF_DEV_OLED_L3_L4_FORMAT, oled_line3and4_format);
    }
    oled_line3and4_format = preferences.getInt(PREF_DEV_OLED_L3_L4_FORMAT);

    if (!preferences.getBool(PREF_DEV_OLED_LOCATOR_INIT)){
      preferences.putBool(PREF_DEV_OLED_LOCATOR_INIT, true);
      preferences.putInt(PREF_DEV_OLED_LOCATOR, oled_show_locator);
    }
    oled_show_locator = preferences.getInt(PREF_DEV_OLED_LOCATOR);

    if (!preferences.getBool(PREF_DEV_OLED_LOCATOR_AMBIGUITY_INIT)){
      preferences.putBool(PREF_DEV_OLED_LOCATOR_AMBIGUITY_INIT, true);
      preferences.putInt(PREF_DEV_OLED_LOCATOR_AMBIGUITY, oled_loc_amb);
    }
    oled_loc_amb = preferences.getInt(PREF_DEV_OLED_LOCATOR_AMBIGUITY);

// APRSIS settings
#ifdef ENABLE_WIFI
    if (!preferences.getBool(PREF_APRSIS_EN_INIT)){
      preferences.putBool(PREF_APRSIS_EN_INIT, true);
      preferences.putBool(PREF_APRSIS_EN, aprsis_enabled);
    }
    aprsis_enabled = preferences.getBool(PREF_APRSIS_EN);

    if (!preferences.getBool(PREF_APRSIS_SERVER_NAME_INIT)){
      preferences.putBool(PREF_APRSIS_SERVER_NAME_INIT, true);
      preferences.putString(PREF_APRSIS_SERVER_NAME, aprsis_host);
    }
    aprsis_host = preferences.getString(PREF_APRSIS_SERVER_NAME, "");

    if (!preferences.getBool(PREF_APRSIS_SERVER_PORT_INIT)){
      preferences.putBool(PREF_APRSIS_SERVER_PORT_INIT, true);
      preferences.putInt(PREF_APRSIS_SERVER_PORT, aprsis_port);
    }
    aprsis_port = preferences.getInt(PREF_APRSIS_SERVER_PORT);

    if (!preferences.getBool(PREF_APRSIS_FILTER_INIT)){
      preferences.putBool(PREF_APRSIS_FILTER_INIT, true);
      preferences.putString(PREF_APRSIS_FILTER, aprsis_filter);
    }
    aprsis_filter = preferences.getString(PREF_APRSIS_FILTER, "");

    if (!preferences.getBool(PREF_APRSIS_FILTER_LOCAL_INCOMING_INIT)){
      preferences.putBool(PREF_APRSIS_FILTER_LOCAL_INCOMING_INIT, true);
      preferences.putString(PREF_APRSIS_FILTER_LOCAL_INCOMING, "");
    }
    aprsis_own_filters_in = preferences.getString(PREF_APRSIS_FILTER_LOCAL_INCOMING, "");
    if (!aprsis_own_filters_in.isEmpty() && aprsis_own_filters_in.startsWith("-")) {
      aprsis_own_filter_in_is_whitelist = false;
      aprsis_own_filters_in = String((aprsis_own_filters_in.c_str())+1);
      aprsis_own_filters_in.trim();
      aprsis_own_filters_in.replace(" ", "");
    } else {
      aprsis_own_filter_in_is_whitelist = true;
    }

    if (!preferences.getBool(PREF_APRSIS_FILTER_LOCAL_WORDS_INCOMING_INIT)){
      preferences.putBool(PREF_APRSIS_FILTER_LOCAL_WORDS_INCOMING_INIT, true);
      preferences.putString(PREF_APRSIS_FILTER_LOCAL_WORDS_INCOMING, "");
    }
    aprsis_own_filters_words_in = preferences.getString(PREF_APRSIS_FILTER_LOCAL_WORDS_INCOMING, "");
    aprsis_own_filters_words_in.trim();

    if (!preferences.getBool(PREF_APRSIS_CALLSIGN_INIT)){
      preferences.putBool(PREF_APRSIS_CALLSIGN_INIT, true);
      preferences.putString(PREF_APRSIS_CALLSIGN, aprsis_callsign);
    }
    aprsis_callsign = preferences.getString(PREF_APRSIS_CALLSIGN, "");

    if (!preferences.getBool(PREF_APRSIS_PASSWORD_INIT)){
      preferences.putBool(PREF_APRSIS_PASSWORD_INIT, true);
      preferences.putString(PREF_APRSIS_PASSWORD, aprsis_password);
    }
    aprsis_password = preferences.getString(PREF_APRSIS_PASSWORD, "");

    if (!preferences.getBool(PREF_APRSIS_ALLOW_INET_TO_RF_INIT)){
      preferences.putBool(PREF_APRSIS_ALLOW_INET_TO_RF_INIT, true);
      preferences.putInt(PREF_APRSIS_ALLOW_INET_TO_RF, aprsis_data_allow_inet_to_rf);
    }
    aprsis_data_allow_inet_to_rf = preferences.getInt(PREF_APRSIS_ALLOW_INET_TO_RF);
#endif

    refill_preferences_as_jsonData();
}
#endif // ENABLE_PREFERENCES


// Power on / off GPS
void do_poweroff_gps()
{
  gps_task_enabled = false;  // vTaskSuspend() is done by taskGPS itself (he needs to delete the wdt timer)
  #ifdef T_BEAM_V1_0
    axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);                          // switch off GPS
  #elif T_BEAM_V1_2
    axp.disableALDO3();                                                   // switch off GPS
  #elif HELTEC_WIRELESS_TRACKER
    #ifdef VEXT_CTRL
      digitalWrite(VEXT_CTRL, LOW);
      #ifdef HAS_TFT
        display_is_on = false;
      #endif
    #endif
  #endif
  gps_isValid = false;
  t_gps_fix_lost = millis();
  t_gps_got_first_fix_after_last_loss = 0L;
}

void do_poweron_gps(int also_reinit_other_devices_controlled_by_VEXT_CTRL)
{
  #ifdef T_BEAM_V1_0
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);                           // switch on GPS
  #elif T_BEAM_V1_2
    axp.setALDO3Voltage(3300);
    axp.enableALDO3();                                                    // switch on GPS
  #elif HELTEC_WIRELESS_TRACKER
    #ifdef VEXT_CTRL
      //also_reinit_other_devices_controlled_by_VEXT_CTRL
      digitalWrite(VEXT_CTRL, HIGH);
      #ifdef HAS_TFT
        // Display was also turned off. Reinitialize..
        do_tft_init();
        fillDisplayLine1(5);
        fillDisplayLine2();
        fillDisplayLines3to5(1);
        enableOled_now(); // enable OLED
      #endif
    #endif
  #endif
  t_gps_fix_lost = millis();
  t_gps_got_first_fix_after_last_loss = 0L;
  if (xHandle_GPS) {
    vTaskResume(xHandle_GPS);
  }
  gps_task_enabled = true;
}


void setup_phase2_soft_reconfiguration(boolean runtime_reconfiguration) {

  if (runtime_reconfiguration) {
    digitalWrite(TXLED, TXLED_LOW);
    Serial.printf("Init after reloading preferences for Callsign: %s\r\n", Tcall.c_str());
    set_callsign();
    Serial.printf("APRS Callsign: %s\r\n", Tcall.c_str());
  }

  // switch LoRa chip on or off
  #ifdef T_BEAM_V1_0
    axp.setPowerOutPut(AXP192_LDO2, (lora_rx_enabled || lora_digipeating_mode > 0) ? AXP202_ON : AXP202_OFF);
  #elif T_BEAM_V1_2
    if (lora_rx_enabled || lora_digipeating_mode > 0) {
      axp.setALDO2Voltage(3300);
      axp.enableALDO2();                            // switch LoRa chip on
      #if HAS_SX126X
        radio.standby();
        radio.startReceive();
      #endif
    } else {
      #if HAS_SX126X
        radio.sleep();
      #endif
      axp.disableALDO2();                           // switch LoRa chip off
    }
  #elif HAS_SX126X
    if (lora_rx_enabled || lora_digipeating_mode > 0) {
      radio.standby();
      radio.startReceive();
    } else {
      radio.sleep();
    }
  #endif

  if (gps_state) {
    if (!gps_task_enabled)
      do_poweron_gps(0);
    if (!xHandle_GPS) {
      // new process: GPS
      writedisplaytext(Tcall,"","Init:","Waiting for GPS","","");
      xTaskCreate(taskGPS, "taskGPS", 5000, nullptr, 1, &xHandle_GPS);
      writedisplaytext(Tcall,"","Init:","GPS Task Created!","","");
    } // else: resume() done by do_poweron_gps()
  } else {
    do_poweroff_gps();
  }
  Serial.printf("GPS powered %s\r\n", gps_state ? "on" : "off");
  gps_state_before_autochange = false;

  //#ifdef T_BEAM_V1_0
  //  axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);                          // switch this on if you need it
  //#elif T_BEAM_V1_2
  // axp.enableXXX();                                                       // switch this on if you need it
  //#endif

  // can reduce cpu power consumtion up to 20 %

  if (adjust_cpuFreq_to > 0) {
    Serial.print("CPU Freq ad");
    setCpuFrequencyMhz(adjust_cpuFreq_to);
    // ..survived
    Serial.printf("justed to: %d MHz\r\n", adjust_cpuFreq_to);
  }

  // LoRa Chip config
  // if we are fill-in or wide2 digi, we listen only on configured main frequency
  lora_speed_rx_curr = (rx_on_frequencies != 2 || lora_digipeating_mode > 1) ? lora_speed : lora_speed_cross_digi;
  lora_set_speed(lora_speed_rx_curr);
  Serial.printf("LoRa Speed:\t%lu\r\n", lora_speed_rx_curr);

  lora_freq_rx_curr = (rx_on_frequencies != 2 || lora_digipeating_mode > 1) ? lora_freq : lora_freq_cross_digi;
  Serial.printf("LoRa FREQ:\t%f\r\n", lora_freq_rx_curr);

  if (Tcall == "N0CALL" && lora_tx_enabled) {
    Serial.println("Disabling lora TX for SWLs");
    lora_tx_enabled = false;
    txPower = txPower_cross_digi = 0;
  }

  #ifdef HAS_SX127X
    rf95.setFrequency(lora_freq_rx_curr);

    // we tx on main and/or secondary frequency. For tx, loraSend is called (and always has desired txpower as argument)
    if (lora_tx_enabled)
      rf95.setTxPower((lora_digipeating_mode < 2 || lora_cross_digipeating_mode < 1) ? txPower : txPower_cross_digi);
    else
      rf95.setTxPower(0);

    // setting rx TO, default to allow rx of long preamble packets
    rf95.setPreambleLength(rxTimeoutSymbols);
  #endif
  #ifdef HAS_SX126X
    radio.setFrequency(lora_freq_rx_curr);

    // we tx on main and/or secondary frequency. For tx, loraSend is called (and always has desired txpower as argument)
    if (lora_tx_enabled)
      radio.setOutputPower((lora_digipeating_mode < 2 || lora_cross_digipeating_mode < 1) ? txPower : txPower_cross_digi);
    else
      radio.setOutputPower(0);

    // setting rx TO, default to allow rx of long preamble packets
    radio.setPreambleLength(rxTimeoutSymbols);

    radio.setRxBoostedGainMode(true);
    radio.setCurrentLimit(140);

    radio.setDio1Action(signal_new_packet_received);
    radio.startReceive();
  #endif
  flag_lora_packet_available = false;

  Serial.printf("LoRa PWR: %d, LoRa PWR XDigi: %d, RX Enable: %d, TX Enable: %d\r\n", txPower, txPower_cross_digi, lora_rx_enabled, lora_tx_enabled);

  // APRS fixed location and icon settings
  init_and_validate_aprs_position_and_icon();

  // init smart beaconing angle average
  for (int i=0;i<ANGLE_AVGS;i++) {                                        // set average_course to "0"
    average_course[i]=0;
  }

  units_speed = units & 15;
  units_dist = (units &= ~15);
  gps_speed = 0;
  fillDisplayLines3to5(1);

  // We need this assurance for fallback to fixed interval, if gps position is lost.
  // fixed beacon rate heigher than sb_max_interval does not make sense
  if (!fixed_beacon_enabled && gps_state && fix_beacon_interval < sb_max_interval)
    fix_beacon_interval = (sb_max_interval > 120000 ? sb_max_interval : 120000);

  if (runtime_reconfiguration) {
    setup_oled_timer_values();
    // fix exception: reference to free'd Tcall in OledHdr -> New copy of current Tcall
    OledHdr = String(Tcall);
    OledLine1 = "setup_phase2";
    OledLine2 = String("");
    OledLine3 = String("");
    OledLine4 = String("");
    OledLine5 = String("");
    writedisplaytext(OledHdr,OledLine1,OledLine2,OledLine3,OledLine4,OledLine5);
  } // else: in setup() during boot, we have several unpredictable delays. That's why it's not called here

  if (runtime_reconfiguration)
    digitalWrite(TXLED, TXLED_HIGH);
}

void setup_compile_flags_info()
{
#ifdef  HELTEC_V1
  strcpy(compile_flags, "Heltec-WiFi-v1 / HELTEC_V1");
#elif   HELTEC_V2
  strcpy(compile_flags, "Heltec-WiFi-v2 / HELTEC_V2");
#elif   HELTEC_WIRELESS_TRACKER
  strcpy(compile_flags, "heltec-wireless-tracker / HELTEC_WIRELESS_TRACKER_V3");
#elif   LORA32_1
  strcpy(compile_flags, "ttgo-lora32-v1 / LORA32_1");
#elif   LORA32_2
  strcpy(compile_flags, "ttgo-lora32-v2 / LORA32_2");
#elif   LORA32_21
  strcpy(compile_flags, "ttgo-lora32-v2_1 / LORA32_21");
#elif   T_BEAM_V0_7
  strcpy(compile_flags, "ttgo-t-beam-v0_7 / T_BEAM_V0_7");
#elif   T_BEAM_V1_0
  strcpy(compile_flags, "ttgo-t-beam-v1_0 / T_BEAM_V1_0");
#elif   T_BEAM_V1_2
  strcpy(compile_flags, "ttgo-t-beam-v1_2 / T_BEAM_V1_2");
#else
  strcpy(compile_flags, "IcannotExist / UNKNOW");
#endif

#ifdef  ENABLE_BLUETOOTH
  strcat(compile_flags, ",ENABLE_BLUETOOTH");
#endif
#ifdef  ENABLE_WIFI
  strcat(compile_flags, ",ENABLE_WIFI");
#endif
#ifdef  KISS_PROTOCOL
  strcat(compile_flags, ",KISS_PROTOCOL");
#endif
#ifdef  ENABLE_OLED
  strcat(compile_flags, ",ENABLE_OLED");
#endif
#ifdef  ENABLE_SYSLOG
  strcat(compile_flags, ",ENABLE_SYSLOG");
#endif

#ifdef  DEVELOPMENT_DEBUG
  strcat(compile_flags, ",DEVELOPMENT_DEBUG");
#endif
}


void do_shutdown(boolean update_display_text = true, const String& status_message_detail = "") {
  // enforce display wakeup (because we will not reach the timer function for setting it on)
  enableOled_now();

  if (update_display_text)
    writedisplaytext("((HALT))","","Powering","down","","");

  esp_task_wdt_reset();
  #ifdef ENABLE_WIFI
    do_send_status_message_about_shutdown_to_aprsis();
  #endif
  esp_task_wdt_reset();
  delay(2000);

  // send_status_message_about_shutdown_to_rf() writes also to display. Display HALT afterwards
  if (send_status_message_about_shutdown_to_rf) {
    String msg = String("B") + buildnr + String(",up:") + String((int ) (millis()/1000/60)) + String(" qrt");
    if (!status_message_detail.isEmpty())
      msg = msg + " " + status_message_detail;
    sendStatusPacket(msg);
  }

  #ifdef T_BEAM_V1_0
    axp.setChgLEDMode(AXP20X_LED_OFF);
    axp.shutdown();
  #elif T_BEAM_V1_2
    axp.setChargingLedMode(XPOWERS_CHG_LED_OFF);
    axp.shutdown();
  #else
    display.dim(true);
  #ifdef VEXT_CTRL
    digitalWrite(VEXT_CTRL, LOW);
  #endif
    esp_deep_sleep_start();
  #endif
}

// + SETUP --------------------------------------------------------------+//
void setup()
{

  // for diagnostics
  uint32_t t_setup_entered = millis();

  // https://iotespresso.com/how-to-disable-brownout-detector-in-esp32-in-arduino/
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  // initialize ESP32 Process WDT, 120s T/O
  esp_task_wdt_init(120, true);

  // Our BUILD_NUMBER. The define is not available in the WEBSERVER -> we need to assign a global variable
  //buildnr = BUILD_NUMBER;
  buildnr = VERS_XXSHORT_BN;
  setup_compile_flags_info();

  Serial.begin(115200);
  delay(2000);  // 2s delay to be safe that serial.print works
  Serial.println("System Start-Up");

  #ifdef HAS_SX127X
    SPI.begin(SPI_sck,SPI_miso,SPI_mosi,SPI_ss);    //DO2JMG Heltec Patch
  #elif HAS_SX126X
    SPI.begin(SPI_sck, SPI_miso, SPI_mosi);
    radio.begin(lora_freq);
    radio.setOutputPower(0);
  #endif

  // Start I2C. Useful if we have an OLED. But may be needed anyway.
  Wire.begin(I2C_SDA, I2C_SCL);

#ifdef HAS_TFT
  #ifdef VEXT_CTRL
    #if HELTEC_WIRELESS_TRACKER
      digitalWrite(VEXT_CTRL, HIGH);
    #endif
  #endif
  do_tft_init();
#else
  // Enable OLED as soon as possible, for better disgnostics
   if (!display.begin(SSD1306_SWITCHCAPVCC, SSD1306_ADDRESS)) {
     //for(;;);                                                             // Don't proceed, loop forever
     delay(30000);
     ESP.restart();
  }
  display_do_full_refresh = true;
#endif

  writedisplaytext("LoRa-APRS","by DL9SAU & DL3EL","Build:" + buildnr,"Hello!","For Factory Reset:","  press middle Button");
  Serial.println("LoRa-APRS by DL9SAU & DL3EL Build:" + buildnr);
  Serial.println("Hardware / compiled with features: ");
  Serial.print("  ");
  Serial.println(compile_flags);

  #ifdef BUZZER
    // framwork-arduinoespressif32 library now warns if frequency is too high:
    // ledc: requested frequency and duty resolution can not be achieved, try reducing freq_hz or duty_resolution. div_param=50
    // [E][esp32-hal-ledc.c:75] ledcSetup(): ledc setup failed!
    // Examples in documentation use 5000. If your buzzer does not work correctly, please find a correct value,
    // and report us (along with the info which CPU frequency you configured).
    //ledcSetup(0,1E5,12);
    ledcSetup(0,5000,12);
    ledcAttachPin(BUZZER,0);
    ledcWriteTone(0,0);  // turn off buzzer on start
  #endif

  #ifdef DIGI_PATH
    relay_path = DIGI_PATH;
  #else
    relay_path = "";
  #endif

  #ifdef FIXED_BEACON_EN
    fixed_beacon_enabled = true;
  #endif


  #ifdef ENABLE_PREFERENCES
    int clear_preferences = 0;
    if(digitalRead(BUTTON)==LOW){
      clear_preferences = 1;
    }

    preferences.begin("cfg", false);

    #ifdef ENABLE_WIFI
      init_wifi_STA_and_AP_settings();
    #endif

    // https://www.tutorialspoint.com/esp32_for_iot/esp32_for_iot_spiffs_storage.htm
    // Launch SPIFFS file system
    if (SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
      Serial.println("SPIFFS Mount Success");
      //debug:
      //listDir(SPIFFS, "/", 0);
      #ifdef ENABLE_WIFI
        // read wifi.cfg file, interprete the json and assign some of our global variables
        readFile(SPIFFS, "/wifi.cfg");
      #endif

      // current idea is, that if PREF_LORA_FREQ_PRESET_INIT is false, then this is a fresh reseted system.
      // if available preference.cfg values should beused, if not, takes default values
      if (!preferences.getBool(PREF_LORA_FREQ_PRESET_INIT)) {
        // reseted? try to use /preferences.cfg
        readFile(SPIFFS, "/preferences.cfg");
        #ifdef ENABLE_WIFI
          if (preferences.getString(PREF_WIFI_SSID, "").isEmpty()) {
            preferences.putString(PREF_WIFI_SSID, wifi_ModeSTA_SSID);
            preferences.putString(PREF_WIFI_PASSWORD, wifi_ModeSTA_PASS);
            preferences.putString(PREF_AP_PASSWORD, wifi_ModeAP_PASS);
            #if defined(ENABLE_SYSLOG)
              if (debug_verbose)
                syslog_log(LOG_DEBUG, String("FlashWrite preferences: setup()"));
            #endif
            Serial.println("WiFi: Updated remote SSID: " + wifi_ModeSTA_SSID);
            Serial.println("WiFi: Updated remote PW: ***");
          }
        #endif
      } else {
        Serial.println("Preferences: normal start, using preferences from flash");
      }
      SPIFFS.end();
    } else {
      Serial.println("SPIFFS Mount Failed");
    }

    // always call load_preferences_from_flash. It updates the _INIT values, and will do some value checks
    load_preferences_from_flash();

    if (clear_preferences){
      delay(1000);
      if(digitalRead(BUTTON)==LOW){
        clear_preferences = 2;
      }
    }

#endif // ENABLE_PEFERENCES

  pinMode(TXLED, OUTPUT);
  #if defined(T_BEAM_V1_0) || defined(T_BEAM_V1_2)
    pinMode(BUTTON, INPUT);
  #elif T_BEAM_V0_7
    pinMode(BUTTON, INPUT);
  #elif HELTEC_WIRELESS_TRACKER
    pinMode(BUTTON, INPUT);
    #ifdef HAS_TFT
      pinMode(TFT_BACKLIGHT, INPUT);
    #endif
  #else
    pinMode(BUTTON, INPUT_PULLUP);
  #endif
  digitalWrite(TXLED, TXLED_HIGH);                                               // turn blue LED on

  #ifdef T_BEAM_V1_0
    if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
      ;
    }
    axp.setLowTemp(0xFF);                                                 //SP6VWX Set low charging temperature
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);
    //axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);				// switch this on if you need it
    axp.setDCDC1Voltage(3300);
    // Enable ADC to measure battery current, USB voltage etc.
    axp.adc1Enable(0xfe, true);
    axp.adc2Enable(0x80, true);
    axp.setChgLEDMode(AXP20X_LED_OFF);
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);                          // oled do not turn off
  #elif T_BEAM_V1_2
    if (!axp.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
      ;
    }
    axp.disableTSPinMeasure();
    axp.setDC1Voltage(3300);
    axp.enableDC1();                                                        // oled do not turn off
    axp.setDC2Voltage(3300);
    axp.enableDC2();
    // Enable ADC to measure battery current, USB voltage etc.
    axp.enableGeneralAdcChannel();
    axp.enableTemperatureMeasure();
    axp.enableBattDetection();
    axp.enableSystemVoltageMeasure();
    axp.enableVbusVoltageMeasure();
    axp.enableBattVoltageMeasure();
    axp.setChargingLedMode(XPOWERS_CHG_LED_OFF);
    // set the charging voltage
    axp.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);
    // set the charging current
    axp.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);
    // Battery-friendly settings:
    // Set the precharge current
    axp.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_200MA);
    // Set the charging termination current
    axp.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);
  #elif T_BEAM_V0_7
    //adcAttachPin(35);
    //adcStart(35);
    //analogReadResolution(10);
    //analogSetAttenuation(ADC_6db);
    pinMode(BATTERY_PIN, INPUT);
    //adc1_config_width(ADC_WIDTH_BIT_12);
    //adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_11);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_6);
  #elif HELTEC_WIRELESS_TRACKER
    #ifdef BATTERY_PIN
      pinMode(BATTERY_PIN, INPUT);
    #endif
    #ifdef ADC_CTRL
      pinMode(ADC_CTRL, OUTPUT);
    #endif
    #ifdef VEXT_CTRL
      pinMode(VEXT_CTRL, OUTPUT); // this is for GPS and TFT screen on Wireless_Tracker and only for Oled in Heltec V3
    #endif
  #else
    pinMode(BATTERY_PIN, INPUT);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_6);
  #endif

  batt_read();
  writedisplaytext("LoRa-APRS","","Init:","ADC OK!","P: "+String(InpVolts, 2)+"V, BAT: "+String(BattVolts,2)+"V","");

  // can reduce cpu power consumtion up to 20 %
  if (adjust_cpuFreq_to > 0)
    setCpuFrequencyMhz(adjust_cpuFreq_to);

  set_callsign();
  writedisplaytext("LoRa-APRS","by DL9SAU & DL3EL","Build:" + buildnr,"Hello de " + Tcall,"For Factory Reset:","  press middle Button");
  delay(2000);

  #ifdef ENABLE_PREFERENCES
    if (clear_preferences == 2){
      //#if defined(T_BEAM_V1_0) || defined(T_BEAM_V1_2)
        if(digitalRead(BUTTON)==LOW){
          clear_preferences = 3;
          preferences.clear();
          preferences.end();
          writedisplaytext("LoRa-APRS","","Reset to /preferences.cfg","","if availabe","now booting");
          delay(2000);
          ESP.restart();
        } else {
          writedisplaytext("LoRa-APRS","","Factory reset","canceled","","");
          delay(2000);
        }
      //#endif
    }
  #endif

  delay(500);

  #ifdef HAS_SX127X
    if (!rf95.init()) {
      writedisplaytext("LoRa-APRS","","Init:","RF95 FAILED!",":-(","");
      Serial.println("Init: RF95 FAILED!");
      //for(;;); // Don't proceed, loop forever
      delay(30000);
      ESP.restart();
    }
    writedisplaytext("LoRa-APRS","","Init:","RF95 OK!","","");
  #elif HAS_SX126X
    // todo: how to verify?
    writedisplaytext("LoRa-APRS","","Init:","SX126X OK!","","");
  #endif

  setup_phase2_soft_reconfiguration(0);
  delay(500);

  // Avoid concurrent access of processes to lora our chip
#ifdef IF_SEMAS_WOULD_WORK
  sema_lora_chip = xSemaphoreCreateBinary();
  sema_handle_aprs_message_addressed_to_us = xSemaphoreCreateBinary();
  sema_is_call_blacklisted = xSemaphoreCreateBinary();
#else
  sema_lora_chip = false;
  sema_handle_aprs_message_addressed_to_us = false;
  sema_is_call_blacklisted = false;
#endif

  // new process: TNC
  #ifdef KISS_PROTOCOL
    xTaskCreatePinnedToCore(taskTNC, "taskTNC", 10000, nullptr, 1, nullptr, xPortGetCoreID());
  #endif

  // We could start process webServer here.
  // But:
  //  - webserver and bluetooth do not work in parallel on some devices.
  //  - webserver needs some variables to be set correctly if it starts up
  //    (and web client requests them).
  // -> We already finished variable stuff above, or do it right before end of setup().
  //    Now we are prepared to start the webserver process (if needed). First, we may start bluetooth.

#if !defined(KISS_PROTOCOL) || !defined(ENABLE_BLUETOOTH)
  enable_bluetooth = false;
#else
  // TTGO: webserver cunsumes abt 80mA. User may not start the webserver
  // if bt-client is connected. We'll also wait here for clients.
  // If enable_webserver on LORA32_21 is set to 2 (or aprsis connection is
  // configured in webserver mode 1), user likes the webserver always
  // to be started -> do not start bluetooth.

#if defined(ENABLE_WIFI) && defined(LORA32_21)
  if (enable_bluetooth && (enable_webserver > 1 || aprsis_enabled))
    enable_bluetooth = false;
#endif

  if (enable_bluetooth) {
    #if defined(BLUETOOTH_PIN)
      SerialBT.setPin(BLUETOOTH_PIN);
    #endif
    SerialBT.begin(String("TTGO LORA APRS ") + Tcall);
    writedisplaytext("LoRa-APRS","","Init:","BT OK!","","");

    #if defined(ENABLE_WIFI)
      if (enable_webserver == 1 && !aprsis_enabled) {
        writedisplaytext("LoRa-APRS","","Init:","Waiting for BT-client","","");
        // wait 60s until BT client connects
        uint32_t t_end = millis() + 60000;
        while (millis() < t_end) {
          if (SerialBT.hasClient())
            break;
          delay(100);
        }
        if (!SerialBT.hasClient()) {
          #if defined(LORA32_21)
            writedisplaytext("LoRa-APRS","","Init:","Waiting for BT-client","Disabling BT!","");
            SerialBT.end();
            serial_bt_client_is_connected = false;
            enable_bluetooth = false;
          #endif
        } else {
          writedisplaytext("LoRa-APRS","","Init:","Waiting for BT-clients","BT-client connected","Will NOT start WiFi!");
        }
        delay(1500);
      }
    #endif /* ENABLE_WIFI */
  }
#endif // KISS_PROTOCOL and ENABLE_BLUETOOTH


#ifdef ENABLE_WIFI
  if (enable_webserver) {
#if defined(KISS_PROTOCOL) && defined(ENABLE_BLUETOOTH)
    // if enable_webserver == 2 or (enable_webserver == 1 && (no serial-bt-client is connected OR aprs-is-connecion configured)
    if (enable_webserver > 1 || aprsis_enabled || !enable_bluetooth || !SerialBT.hasClient()) {
#else
    {
#endif /* KISS_PROTOCOL && ENABLE_BLUETOOTH */
      webServerCfg = {.callsign = Tcall};
      // new process: TNC
      xTaskCreate(taskWebServer, "taskWebServer", 12000, (void*)(&webServerCfg), 1, nullptr);
      webserverStarted = true;
      writedisplaytext("LoRa-APRS","","Init:","WiFi task started","   =:-)   ","");
#if defined(KISS_PROTOCOL) && defined(ENABLE_BLUETOOTH)
    } else {
      writedisplaytext("LoRa-APRS","","Init:","WiFi NOT started!","   =:-S   ","");
    }
#else
    }
#endif /* KISS_PROTOCOL && ENABLE_BLUETOOTH */
    delay(1500);
  }
#endif /* ENABLE_WIFI */


  setup_oled_timer_values();

  // init lastheard table
  for (int ii=0; ii < MAX_LH; ii++) {
    LH[ii].callsign = "";
  }

  esp_task_wdt_add(NULL); //add current thread to WDT watch
  esp_task_wdt_reset();

  writedisplaytext("LoRa-APRS","","Init:","FINISHED OK!","   =:-)   ","");
  Serial.println("Time used since start (-2000ms delay): " + String(millis()-t_setup_entered-2000) + "ms");

  fillDisplayLine1(2);
  fillDisplayLine2();
  displayInvalidGPS();

  digitalWrite(TXLED, TXLED_LOW);
}


int packet_is_valid (const char *frame_start) {
  const char *p = frame_start;
  if (!*p || !((*p >= 'A' && *p <= 'Z') || (*p >= '0' && *p <= '9')))
    return 0;
  for (p++; *p && *p != ':'; p++) {
    if (! ((*p >= 'A' && *p <= 'Z') || (*p >= '0' && *p <= '9') || *p == '-' || *p == '*' || *p == ',' || *p == '>') )
      return 0;
  }
  if (!*p || *p != ':')
    return 0;
  // do it again. Now we know the header is ok, which makes it parseable more easy, without race conditions (no need to check p[1] == 0 or so..)
  boolean src_call_end = 0;
  boolean to_call_end = 0;
  for (p = frame_start; *p && *p != ':'; p++) {
    if (*p == '>') { if (src_call_end || !isalnum(*(p-1))) return 0; src_call_end = true; } // '>' twice?
    else if (*p == ',') { if (!src_call_end || (to_call_end ? (!(isalnum(*(p-1)) || *(p-1) == '*')) : !(isalnum(*(p-1))))  || !isalnum(p[1])) return 0; to_call_end = true; } // -,call or ,- is not valid
    else if (*p == '-') {
      // not call-1-2 or call-- or -call or ..>-1 or ,-1 or --15
      if (!isalnum(*(p-1)) || !isdigit(p[1])) return 0;
      char c_after_ssid = p[2];
      if (isdigit(c_after_ssid)) {
        if (p[1] != '1' || c_after_ssid > '5') return 0; // max "-15".
        c_after_ssid = p[3];
      }
      if (!(c_after_ssid == '>' || c_after_ssid == '*' || c_after_ssid == ',' || c_after_ssid == ':')) return 0; // After ssid (pos 3) only '>', '*', ',', ':', are allowed.
    }
  }
  if (!to_call_end && isalnum(*(p-1)))
    to_call_end = true;
  if (!src_call_end || !to_call_end) return 0;
  return 1;
}


int is_call_blacklisted(const char *frame_start) {

  sema_lock_or_unlock__is_call_blacklisted(1);

  // src-call_validation
  const char *p_call = frame_start;
  char *header_end;
  char *p;
  int ret = 1;
  boolean ssid_present = false;
  char buf[13]; // room for ",DL1AAA-15*," + \0
  int i = 0;

  if (!p_call || !*p_call) goto end;
  // Special case: message from ham radio winlink system:
  if (!strncmp(p_call, "WLNK", 4)) {
    ;
  } else {
    // Call sign validation
    if (isdigit(p_call[1]) && isalpha(p_call[2])) {
      // left-shift g1abc -> _g1abc
      i = 1;
      p_call--; // warning, beyond start of pointer
    }
    for (; i <= 7; i++) {
      if (i == 7) goto end;
      else if (!p_call[i] || p_call[i] == '>' || p_call[i] == '-') {
        if (i < 4) goto end;
        break;
      } else if (i < 2 && !isalnum(p_call[i])) goto end;
      else if (i == 2 && !isdigit(p_call[i])) goto end;
      else if (i > 2 && !isalpha(p_call[i])) goto end;
    }
  }


  p = buf;
  *p++ = ',';
  for (i = 0; i < 9; i++) {
    if (!frame_start[i] || /* frame_start[i] == '>' || */ ! (frame_start[i] == '-' || isalnum(frame_start[i]))) {
      break;
    }
    if (frame_start[i] == '-')
      ssid_present = true;
    // callvalidation above prevents calls (excl. ssid) with len > 6.
    // this loop goes over all positions in DL1AAA-15 (9). But if someone modifies the assurance above, we'll have
    // a race condition here. If user input is DL1AAAAAA (9 bytes; no ssid present), and we add '-0' afterwards,
    // the result will be ",DL1AAAAAA-00," -> 13, plus \0. But buf is len 12.
    // If we are here at position i==6 (behind "DL1AAA"), that means at '-', and we did not found the ssid,
    // well' enforce a break here.
    if (i == 6 && !ssid_present)
      break;
    *p++ = frame_start[i];
  }
  if (!ssid_present) {
    // for being able to filter out DL1AAA but not DL1AAA-1. -> DL1AAA is DL1AAA-0 by AX.25 definition. Blacklist lists DL1AAA-0. DL1AAA means: filter all variants.
    *p++ = '-'; *p++ = '0';
  }
  *p++ = ',';
  *p = 0;


  // blacklist empty? we may leave here
  if (!*blacklist_calls || !strcmp(blacklist_calls, ",,")) {
    ret = 0;
    goto end;
  }

  // exact match?
  if (strstr(blacklist_calls, buf)) {
    ret = 2;
    goto end;
  }
  // filter call completely?
  if ((p = strchr(buf, '-'))) {
    *p++ = ','; *p++ = 0;
    if (strstr(blacklist_calls, buf)) {
      ret = 3;
      goto end;
    }
  }

  header_end = strchr(frame_start, ':');
  // check for blacklisted digi in path
  if (header_end && (p = strchr(frame_start, ','))) {
    for (;;) {
      char *q = strchr(p+1, ',');
      if (!q || q > header_end)
        q = header_end;
      // copy ",DL1AAA,.." to buf as ",DL1AAA"
      // but before: length check. len ",DL1AAA-15*," is 12; sizeof(buf) is 13 (due to \0); we copy until trailing ','.
      if ((q-p) > sizeof(buf)-1)
        break;
      strncpy(buf, p, q-p);
      buf[q-p] = 0;
      char *r = strchr(buf, '*');
      if (r)
        *r = 0;
      // after modifications above, is len(buf) still <= 10 (room for ',' + 9 (call-ssid)  == 10. and later  ',' and \0)?
      r = strchr(buf, '-');
      if (strlen(buf) > (r ? 10 : 7)) {
        // can't check non-conformal stuff. Unfortunately, aprs-is tier node names in the digi path like T2CSNGRAD (length of 9)
        // are non-conformal. That's why we return 0 here. Length for our exact-match-test below would be ",T2CSNGRAD-0,"+\0 == 14
        break;
      }
      // our ssid filter construct: -0 means search for call with ssid 0 zero.
      if (!r) {
        strcat(buf, "-0");
      }
      strcat(buf, ",");
      // exact match?
      if (strstr(blacklist_calls, buf)) {
        ret = 4;
        goto end;
      }
      // filter call completely?
      if ((r = strchr(buf, '-'))) {
        *r++ = ','; *r++ = 0;
        if (strstr(blacklist_calls, buf)) {
          ret = 5;
          goto end;
        }
      }
      if (q == header_end)
        break;
      p = q;
    }
  }
  ret = 0;

end:
  sema_lock_or_unlock__is_call_blacklisted(0);
  return ret;
}


// rf95.lastSNR() returns unsigned 8bit value, which we get as input
int bg_rf95snr_to_snr(uint8_t snr)
{
    // SNR values reported by rf95.lastSNR() are not plausible. See those two projects:
    // https://github.com/Lora-net/LoRaMac-node/issues/275
    // https://github.com/mayeranalytics/pySX127x/blob/master/SX127x/LoRa.py
    // We use an old BG_RF95 library from 2001. RadioHead/RH_RF95.cpp implements _lastSNR and _lastRssi correctly accordingg to the specs
    // Per page 111, SX1276/77/78/79 datasheet
    //return ( ( snr > 127 ) ? (snr - 256) : snr ) / 4.0;
    if (snr & 0x80) {
      return -(( ( ~snr + 1 ) & 0xFF ) >> 2);
    }
    return ( snr & 0x7F ) >> 2;
}


// bg_rf95rssi_to_rssi, called with arguments: 1. rf95.lastRssi() and 2. result of  bg_rf95snr_to_snr(rf95.lastSNR())
int bg_rf95rssi_to_rssi(int rf95_lastRssi, int _lastSNR)
{
  // We use an old BG_RF95 library from 2001. RadioHead/RH_RF95.cpp implements _lastSNR and _lastRssi correctly accordingg to the specs
  boolean _usingHFport = (lora_freq >= 779.0);

  // bg_rf95 library: _lastRssi = spiRead(BG_RF95_REG_1A_PKT_RSSI_VALUE) - 137. First, undo -137 operation
  int _lastRssi = rf95_lastRssi + 137;

  if (_lastSNR < 0)
    _lastRssi = _lastRssi + _lastSNR;
  else
    _lastRssi = (int)_lastRssi * 16 / 15;
  if (_usingHFport)
    _lastRssi -= 157;
  else
    _lastRssi -= 164;
  return _lastRssi;
}


char *encode_snr_rssi_in_path(int snr, int rssi)
{
  static char buf[7]; // length for "Q2373X" == 6 + 1 (\0) == 7
  *buf = 0;
  // SNR values reported by rf95.lastSNR() are not plausible. See those two projects:
  // https://github.com/Lora-net/LoRaMac-node/issues/275
  // https://github.com/mayeranalytics/pySX127x/blob/master/SX127x/LoRa.py
  // rf95snr_to_snr returns values in range -32 to 31. The lowest two bits are RFU
  if (snr > 99) snr = 99;			// snr will not go upper 31
  else if (snr < -99) snr = -99;		// snr will not go below -32
  if (rssi > 0) rssi = 0;			// rssi is always negative
  else if (rssi < -259) rssi = -259;		// rssi will not be below -174 anyway ;)
  // Make SNR >= 0 human readable:
  //   First position: SNR < 0: replace -1 by A1, -10 by B0, ...
  //   This way, we reduce "-10" to two letters: B0.
  // Make RSSI  > -100 RSSI human readable.
  //   First position: rssi < -99 -> 0. We'll use 91 for rssi -91 instead of J1 for better readibility.
  //   K means -10x, L means -11x, M means -12x, N means -13x, ...   (everone knows, 'N' is #13 in alphabet out of 26 chars ;)
  //   => With this, we reduce "-110" to two letters: L0.
  // Last position of call (6) is left empty for future use. As well as SSID 1-15. Could be used for BER, RX antenna gain, EIRP, ..
  // => This is a good compromise between efficiency and being able to quickly interprete snr and rssi
  sprintf(buf, "Q%c%01d%c%01d",
    ((snr >= 0 ? snr : -snr) / 10) + (snr >= 0 ? '0' : 'A'),
    (snr >= 0 ? snr : -snr) % 10,
    (-rssi / 10) + (rssi > -100 ? '0' : 'A'),
    (-rssi) % 10);

  return buf;
}


char *add_element_to_path(const char *data, const char *element)
{
  static char buf[LORA_MAX_MESSAGE_LEN+1];
  if (strlen(data) + 1 /* ',' */ + strlen(element) + 1 /* '*' */ > sizeof(buf)-1)
    return 0;
  char *p = strchr(data, '>');
  char *header_end = strchr(data, ':');
  if (header_end <= p)
    return 0;
  char *q = strchr(data, ',');
  if (q > header_end)
    q = 0;
  if (q && q < p)
    return 0;
  if (q) {
    int n_digis = 1;
    char *n = q;
    for (;;) {
      n = strchr(n+1, ',');
      if (!n || n > header_end)
        break;
      n_digis++;
    }
  if (n_digis > 7)
    return 0;
  }

  char *r = 0;
  if (q && (r = strchr(q+1, '*')) > 0 && r < header_end) {
    // behind last digipeated call
    for (;;) {
      char *rr = strchr(r+1, '*');
      if (!rr || r > header_end)
        break;
      r = rr;
    }
  }
  char *pos = header_end;
  if (q) {
    // Something like DL9SAU>APRS,NOGATE -> DL9SAU>APRS,MYCALL*,NOGATE DL9SAU>APRS,DL1AAA*,WIDE2-1 -> DL9SAU>APRS,DL1AAA,MYCALL*,WIDE2-1
    // r points to '*', r+1 points to ',' and we omit '*'; q points to ','
    pos = r ? r : q;
  } // else: Something like DL9SAU>APRS:... (no via path)
  snprintf(buf, pos-data +1, "%s", data);
  sprintf(buf + strlen(buf), ",%s*%s", element, pos + (*pos == '*' ? 1 : 0));
  return buf;
}

// append element to path, regardless if it will exceed max digipeaters. It's for snr encoding for aprs-is. We don't use
// add_element_to_path, because we will append at the last position, and do not change digipeated bit.
char *append_element_to_path(const char *data, const char *element) {
  static char buf[LORA_MAX_MESSAGE_LEN+10+1];
  if (strlen(data) + 1 /* ',' */ + strlen(element) > sizeof(buf)-1)
    return 0;
  char *p = strchr(data, '>');
  char *header_end = strchr(data, ':');
  if (header_end <= p)
    return 0;
  char *q = strchr(data, ',');
  if (q > header_end)
    q = 0;
  if (q && q < p)
    return 0;
  snprintf(buf, header_end-data +1, "%s", data);
  sprintf(buf + strlen(buf), ",%s%s", element, header_end);
  return buf;
}


#define AX_ADDR_LEN 9   // room for "DL9SAU-15" == 9
#define AX_DIGIS_MAX 8

struct axaddr {
  char addr[AX_ADDR_LEN+1];
  boolean repeated;
};

struct ax25_frame {
  struct axaddr src;
  struct axaddr dst;
  struct axaddr digis[AX_DIGIS_MAX];
  uint8_t n_digis;
  char *data;
};

struct ax25_frame *tnc_format_to_ax25_frame(const char *s)
{
  static char data[LORA_MAX_MESSAGE_LEN+1];
  static struct ax25_frame frame;
  char *p;
  char *q;
  char *next_digi = 0;

  memset(&frame, 0, sizeof(frame));
  *data = 0;

  if (strlen(s) > sizeof(data) -1)
    return 0;
  strcpy(data, s);

  p = strchr(data, ':');
  if (!p || strlen(p) > sizeof(data)-1)
    return 0;
  // skip leading ':'
  frame.data = p+1;
  *p = 0;

  p = data;
  if (!(q = strchr(p, '>')))
    return 0;
  *q = 0;
  if (!*p || strlen(p) > AX_ADDR_LEN)
    return 0;
  strcpy(frame.src.addr, p);

  p = q+1;
  if ((q = strchr(p, ','))) {
    next_digi = q+1;
    *q = 0;
  }
  if (!*p || strlen(p) > AX_ADDR_LEN)
    return 0;
  strcpy(frame.dst.addr, p);

  int digi;
  for (digi = 0; digi < AX_DIGIS_MAX && next_digi; digi++) {
    p = next_digi;
    if ((q = strchr(p, ','))) {
      next_digi = q+1;
      *q = 0;
    } else
      next_digi = 0;

    if ((q = strchr(p, '*'))) {
      frame.digis[digi].repeated = true;
      *q = 0;
    }
    if (!*p || strlen(p) > AX_ADDR_LEN)
      return 0;
    strcpy(frame.digis[digi].addr, p);
    // max digi cound reached, but frame had more digis in path
    if (digi == AX_DIGIS_MAX-1 && next_digi)
      return 0;
  }

  // sanity check
  for (digi = 0; digi < AX_DIGIS_MAX && *(frame.digis[digi].addr); digi++) {
    if (!frame.digis[digi].repeated && !strncmp(frame.digis[digi].addr, "WIDE", 4) && strlen(frame.digis[digi].addr) == 5)
        frame.digis[digi].repeated = true;
  }
  boolean last_repeated_found = false;
  for (digi = AX_DIGIS_MAX-1; digi >= 0; digi--) {
    if (!*(frame.digis[digi].addr))
      continue;
    // once: set n_digis
    if (!frame.n_digis)
      frame.n_digis = digi+1;
    if (!last_repeated_found && frame.digis[digi].repeated)
      last_repeated_found = true;
    else if (last_repeated_found && !frame.digis[digi].repeated)
      frame.digis[digi].repeated = true;
  }

  return &frame;
}

void handle_lora_frame_for_lora_digipeating(const char *received_frame, const char *snr_rssi)
{

  if (snr_rssi && !*snr_rssi)
    snr_rssi = 0;

  struct ax25_frame* frame = tnc_format_to_ax25_frame(received_frame);

  if (!frame)
    return;

  // no room left for adding our call in path during repeating
  if (frame->n_digis > AX_DIGIS_MAX)
    return;

  // aprs-message / query addressed to us? Format: ":DL9SAU-15:..."
  if (frame->data[0] == ':' && strlen(frame->data) > 10 && frame->data[10] == ':' &&
       !strncmp((frame->data)+1, Tcall.c_str(), Tcall.length()) && (Tcall.length() == 9 || frame->data[9] == ' '))
    return;

  // '>' and ':' found. Always in header. Sanity check: if ',' present, it must be > p. If r exists, must be > q. And header_end must be > than the others and always a message-body *(header_end + 1) != 0.

  // wide1-digi case
  if (lora_digipeating_mode == 2) {
    const char *p = strchr(received_frame, '>');
    const char *q = strchr(received_frame, ','); // digis, optional
    const char *r = strchr(received_frame, '*');
    const char *header_end = strchr(received_frame, ':');
    // we hear an packet, digipeated from another digipeater (same source call, digipeated flag '*' in header)
    // and have the original packet in our digipeating queue? -> clear queue.
    // If we are a WIDE2 digi, it may be desired that we digipeat him.
    // We'll throw that frame away if further down the new frame is worth digipeating. We don't build up Digipeating-TX-queues
    if (p && !strncmp(lora_TXBUFF_for_digipeating, received_frame, p-received_frame) && r && r > q && r < header_end && *(header_end+1)) {
      char *he_prev = strchr(lora_TXBUFF_for_digipeating, ':');
      if (he_prev && !strcmp(header_end, he_prev))
        *lora_TXBUFF_for_digipeating = 0;
    }
  }

  int i = 0;
#ifdef notdef
  // src-call_validation
  // not here anymore; now in is_call_blacklisted()
  char *p_call = frame->src.addr;
  if (isdigit(p_call[1]) && isalpha(p_call[2])) {
    // left-shift g1abc -> _g1abc
    i = 1;
    p_call--; // warning, beyond start of pointer
  }
  for (; i <= 7; i++) {
    if (i == 7) return;
    else if (!p_call[i] || p_call[i] == '-') {
      if (i < 4) return;
      break;
    } else if (i < 2 && !isalnum(p_call[i])) return;
    else if (i == 2 && !isdigit(p_call[i])) return;
    else if (i > 2 && !isalpha(p_call[i])) return;
  }
#endif

  // If DST-call-ssid-digipeating: rewrite before adding path, because WIDE in path and DST-SSID-digipeating are mutual exclusive
  char *q;
  if ((q = strchr(frame->dst.addr, '-'))) {
    if (frame->n_digis && frame->digis[0].repeated)
      return;
    if (frame->n_digis > AX_DIGIS_MAX-1)
      return;
    *q++ = 0;
    if (*q < '1' || *q > '7' || q[1] )
      return;
    for (i = 0; i < frame->n_digis; i++)
      if (!strncmp(frame->digis[i].addr, "WIDE", 4) || !strcmp(frame->digis[i].addr, Tcall.c_str()))
        return;
    // move digi path one right
    for (i = frame->n_digis-1; i >= 0; i--) {
      strcpy(frame->digis[i+1].addr, frame->digis[i].addr);
      frame->digis[i+1].repeated = frame->digis[i].repeated;
    }
    sprintf(frame->digis[0].addr, "WIDE%c-%c", *q, *q);
    frame->digis[0].repeated = false;
    frame->n_digis++;
  }

  // nothing to repeat:
  if (!frame->n_digis)
    return;

  // we do not digipeat our own packets
  if (!strcmp(frame->src.addr, Tcall.c_str()))
    return;

  int curr_not_repeated = 0;
  for (i = 0; i < frame->n_digis; i++) {
    if (!frame->digis[i].repeated) {
      curr_not_repeated = i;
      break;
    }
    // our call with digipeated marker in header?
    if (!strcmp(frame->digis[i].addr, Tcall.c_str()))
      return;
  }

  // our call in path?
  boolean add_our_call = true;
  int insert_our_data_before = -1;

  if (curr_not_repeated == 0 && lora_digipeating_myalias.length() > 0 && !strcmp(frame->digis[curr_not_repeated].addr, lora_digipeating_myalias.c_str())) {
    // Digipeat our alias, if in first position of the path
    // digi path too long for adding snr_rssi? skip adding snr_rssi
    if (snr_rssi && frame->n_digis == AX_DIGIS_MAX)
      snr_rssi = 0;
    frame->digis[curr_not_repeated].repeated = true;
    add_our_call = false;
    strcpy(frame->digis[curr_not_repeated].addr, Tcall.c_str());
    insert_our_data_before = curr_not_repeated;
    // we'll repeat our alias now. our job ends here
    goto add_our_data;
  }
  if (!strcmp(frame->digis[curr_not_repeated].addr, Tcall.c_str())) {
    // digi path too long for adding snr_rssi? skip adding snr_rssi
    if (snr_rssi && frame->n_digis == AX_DIGIS_MAX)
      snr_rssi = 0;
    frame->digis[curr_not_repeated].repeated = true;
    add_our_call = false;
    insert_our_data_before = curr_not_repeated;
    // if we are a digicall-only digi, our job ends here
    goto add_our_data;
  }

  if (lora_digipeating_mode < 2)
    return;

  // digi path too long for adding our call skip adding snr_rssi
  if (frame->n_digis == AX_DIGIS_MAX)
    return;
  // digi path too long for adding snr_rssi and our call? skip adding snr_rssi
  if (snr_rssi && frame->n_digis +1 == AX_DIGIS_MAX)
    snr_rssi = 0;

  if (!strcmp(frame->digis[curr_not_repeated].addr, "WIDE1-1")) {
    frame->digis[curr_not_repeated].addr[5] = 0;
    frame->digis[curr_not_repeated].repeated = 1;
    if (insert_our_data_before < 0)
      insert_our_data_before = curr_not_repeated;
    curr_not_repeated++;
    // If we are a WIDE2 digi, we'll also change WIDE2-n to WIDE2* further down
  }
  if (lora_digipeating_mode < 3)
    goto add_our_data;

  if (!strncmp(frame->digis[curr_not_repeated].addr, "WIDE", 4) && strlen(frame->digis[curr_not_repeated].addr) == 7) {
    if (frame->digis[curr_not_repeated].addr[4] < '2' || frame->digis[curr_not_repeated].addr[4] > '3')
      return;
    if (frame->digis[curr_not_repeated].addr[5] != '-')
      return;
    // bad syntax?
    if (frame->digis[curr_not_repeated].addr[6] < '1' || frame->digis[curr_not_repeated].addr[6] > frame->digis[curr_not_repeated].addr[4])
      return;
    // prevent abuse
    if (curr_not_repeated+1 < frame->n_digis && !strncmp(frame->digis[curr_not_repeated+1].addr, "WIDE", 4))
        return;
    // mark every WIDEn-m as WIDEn and repeated
    frame->digis[curr_not_repeated].addr[5] = 0;
    frame->digis[curr_not_repeated].repeated = 1;
    if (insert_our_data_before < 0)
      insert_our_data_before = curr_not_repeated;
  }

add_our_data:

  // insert_our_data still < 0? i.e. searches like that for WIDE2-1 in path did not lead to a result
  if (insert_our_data_before < 0)
    return;


  // Build txbuff:

  char buf[sizeof(lora_TXBUFF_for_digipeating)];

  sprintf(buf, "%s>%s", frame->src.addr, frame->dst.addr);
  char *digi_paths_start = buf + strlen(buf);

  for (i = 0; i < insert_our_data_before; i++)
    sprintf(buf + strlen(buf), ",%s", frame->digis[i].addr);

  if (snr_rssi)
    sprintf(buf + strlen(buf), ",%s", snr_rssi);

  if (add_our_call)
    sprintf(buf + strlen(buf), ",%s", Tcall.c_str());

  for (i = insert_our_data_before; i < frame->n_digis; i++) {
    sprintf(buf + strlen(buf), ",%s", frame->digis[i].addr);
    if (frame->digis[i].repeated) {
      i++;
      break;
    }
  }

  // we have something added after "SRC>DST"?
  if (strlen(buf) > (digi_paths_start-buf))
    strcat(buf, "*");
  for (; i < frame->n_digis; i++)
    sprintf(buf + strlen(buf), ",%s", frame->digis[i].addr);

  // length check:
  if (strlen(buf) + 1 /* : */ + strlen(frame->data) > sizeof(lora_TXBUFF_for_digipeating)-1)
    return;

  sprintf(lora_TXBUFF_for_digipeating, "%s:%s", buf, frame->data);
  time_lora_TXBUFF_for_digipeating_was_filled = millis();
}

char *s_min_nn(uint32_t min_nnnnn, int high_precision) {
  /* min_nnnnn: RawDegrees billionths is uint32_t by definition and is n'telth
   * degree (-> *= 6 -> nn.mmmmmm minutes) high_precision: 0: round at decimal
   * position 2. 1: round at decimal position 4. 2: return decimal position 3-4
   * as base91 encoded char.
   */

  static char buf[8];
  min_nnnnn = min_nnnnn * 0.006;

  if (high_precision == 3) {
    if ((min_nnnnn % 100) >= 50 && min_nnnnn < 6000000 - 50) {
      // round up. Avoid overflow (59.99999 should never become 60.0 or more)
      min_nnnnn = min_nnnnn + 50;
    }
  } else if (high_precision) {
    if ((min_nnnnn % 10) >= 5 && min_nnnnn < 6000000 - 5) {
      // round up. Avoid overflow (59.999999 should never become 60.0 or more)
      min_nnnnn = min_nnnnn + 5;
    }
  } else {
    if ((min_nnnnn % 1000) >= 500 && min_nnnnn < (6000000 - 500)) {
      // round up. Avoid overflow (59.9999 should never become 60.0 or more)
      min_nnnnn = min_nnnnn + 500;
    }
  }

  switch (high_precision) {
  case 0:
  case 1:
    sprintf(buf, "%02u.%02u", (unsigned int)((min_nnnnn / 100000) % 100), (unsigned int)((min_nnnnn / 1000) % 100));
    break;
  case 2:
    sprintf(buf, "%c", (char)((min_nnnnn % 1000) / 11) + 33);
    // Like to verify? type in python for i.e. RawDegrees billions 566688333: i =
    // 566688333; "%c" % (int(((i*.0006+0.5) % 100)/1.1) +33)
    break;
  case 3:
    sprintf(buf, "%02u.%03u", (unsigned int)((min_nnnnn / 100000) % 100), (unsigned int)((min_nnnnn / 100) % 1000));
  default:
    sprintf(buf, "%02u.%04u", (unsigned int)((min_nnnnn / 100000) % 100), (unsigned int)((min_nnnnn / 10) % 10000));
  }
  return buf;
}

String create_lat_aprs(const char *delimiter, RawDegrees lat, int precision) {
  char str[20];
  char n_s = 'N';
  if (lat.negative) {
    n_s = 'S';
  }
  if (delimiter && strlen(delimiter) > 1)
    delimiter = 0;
  // we like sprintf's float up-rounding.
  // but sprintf % may round to 60.00 -> 5360.00 (53° 60min is a wrong notation
  // ;)
  sprintf(str, "%02d%s%s%c", lat.deg, delimiter ? delimiter : "", s_min_nn(lat.billionths, precision > 2 ? precision : 0), n_s);
  return String(str);
}

String create_long_aprs(const char *delimiter, RawDegrees lng, int precision) {
  char str[20];
  char e_w = 'E';
  if (lng.negative) {
    e_w = 'W';
  }
  if (delimiter && strlen(delimiter) > 1)
    delimiter = 0;
  sprintf(str, "%03d%s%s%c", lng.deg, delimiter ? delimiter : "", s_min_nn(lng.billionths, precision > 2 ? precision : 0), e_w);
  return String(str);
}


void update_speed_from_gps() {
  if (gps_state) {
    switch (units_speed) {
    case UNITS_SPEED_MS:
      gps_speed = gps.speed.mps();
      break;
    case UNITS_SPEED_MPH:
      gps_speed = gps.speed.mph();
      break;
    case UNITS_SPEED_KN:
      gps_speed = gps.speed.knots();
      break;
    case UNITS_SPEED_KMH: // fall through
    default:
      gps_speed = gps.speed.kmph();
    }
  } else {
    gps_speed = 0;
  }
}

// usb-serial tnc emulator

int parse_cmd_arg(const String &inputBuf, char *cmd, int cmd_size, char *arg, int arg_size) {
  const char *p = inputBuf.c_str();
  char *q;

  if (!cmd || cmd_size < 1 || !arg || arg_size < 1)
    return 1;
  *cmd = *arg = 0;
  if (cmd_size < 2 || arg_size < 2)
  return 1;

  // copy comand. First, skip leading blanks
  while (*p && *p == ' ')
    p++;
  // copy command part (separator is ' ')
  for (q = cmd; *p && *p != ' ' && q-cmd < cmd_size-1; p++, q++) {
    if (*p >= 'A' && *p <= 'Z')
      *q = tolower(*p);
    else
      *q = *p;
  }
  *q = 0;

  // copy arg. First, skip leading blanks
  while (*p && *p == ' ')
    p++;
  // copy arg until end or ' '
  for (q = arg; *p && *p != ' ' && q-arg < arg_size-1; p++, q++) {
    if (*p >= 'A' && *p <= 'Z')
      *q = tolower(*p);
    else
      *q = *p;
  }
  *q = 0;

  return 0;
}


void handle_usb_serial_input(void) {
  if (!usb_serial_data_type || !Serial.available())
    return;

  static boolean cmd_mode = true;
  static String inputBuf;
  static boolean local_echo = true;
  static boolean usb_serial_data_type__had_traceing_enabled_before_entering_converse_mode = false;
  static boolean in_rx_kiss_frame = false;
  char c = Serial.read();
  boolean do_prompt = false;

  if (c == 0xC0 || in_rx_kiss_frame) {
    // was kiss frame
    inputBuf = "";
    if (c == 0xC0)
      in_rx_kiss_frame = !in_rx_kiss_frame;;
    return;
  }

  if (inputBuf.length() > 255) {
    // reboot? lol. No ;)
    inputBuf = "";
    Serial.println("*** Error: Line too long");
    inputBuf = "";
  }

  if (c == 0x03) {
    // user pressed ^C
    do_prompt = true;
    Serial.print("\r\n");
    if (!cmd_mode) {
      if (!usb_serial_data_type__had_traceing_enabled_before_entering_converse_mode)
        usb_serial_data_type &= ~2;
      cmd_mode = true;
    }
    inputBuf="";
  } else if (c == 0x0c) {
    // user pressed ^L
    if (local_echo) {
      Serial.print("\r");
      Serial.print(inputBuf);
    }
  } else if (c == 0x15) {
    // user pressed ^U (clear line)
    inputBuf="";
    Serial.print("\r");
  } else if (c == 0x7f || c == 0x08) {
    if (inputBuf.length() > 0)
      inputBuf.remove(inputBuf.length()-1);
    if (local_echo)
      Serial.print(c);
  } else if (c == '\n') {
#ifdef notdef // no, ignore
    if (local_echo)
      Serial.print(c);
#endif
  } else if (c == '\r') {
    if (local_echo)
      Serial.print("\r\n");

    if (cmd_mode) {

      // command mode

      char buf_cmd[256];
      char buf_arg[256];
      parse_cmd_arg(inputBuf, buf_cmd, sizeof(buf_cmd), buf_arg, sizeof(buf_arg));
      String cmd = String(buf_cmd);
      String arg = String(buf_arg);

      do_prompt = true;

      if (inputBuf != "") {

        boolean arg_bool = false;
        if (arg == "on") {
          arg_bool = true;
        } else if (arg == "off") {
          arg_bool = false;
        } else {
          // This one is for us developers ;)
          if (cmd == "debug") {
            if (arg != "") {
              debug_verbose = arg.toInt();
              Serial.printf("Debug Level now: %d (Arg: %s)\r\n", debug_verbose, arg.c_str());
            } else {
              Serial.printf("Debug Level: %d\r\n", debug_verbose);
            }
            Serial.print("cmd:");
            inputBuf = "";
            return;
          }
          //  some commands need an non-binary agument
          #ifdef ENABLE_PREFERENCES
            if (cmd == "save_preferences_cfg") {
              Serial.println("*** save_preferences_cfg:");
              refill_preferences_as_jsonData();
              if (!preferences_as_jsonData.isEmpty()) {
                int ret = save_to_file("TNC", "/preferences.cfg", preferences_as_jsonData);
                if (ret >= 0)
                  Serial.println("*** save_preferences_cfg: ok");
                else
                  Serial.printf("*** save_preferences_cfg: error %d\r\n", ret);
              } else {
                  Serial.println("*** save_preferences_cfg: BUG (empty)");
              }
              Serial.print("cmd:");
              inputBuf = "";
              return;
            } else if (cmd == "show_preferences") {
              Serial.println("*** show_preferences:");
              refill_preferences_as_jsonData();
              // local copy
              String s = String(preferences_as_jsonData);
              s.replace("\n", "\r\n");
              Serial.print(s);
              Serial.printf("***\r\n");
              Serial.print("cmd:");
              inputBuf = "";
              return;
            } else if (cmd == "preferences") {
              Serial.println("*** preferences: error: preferences command needs to be implemented ;)");
              #if defined(ENABLE_SYSLOG) && defined(ENABLE_WIFI)
                syslog_log(LOG_WARNING, String("usb-serial: preferences: user entered preferences command. Yet not implemented."));
              #endif
              Serial.print("cmd:");
              inputBuf = "";
              return;
            } else if (cmd == "show_wifi") {
              Serial.println("*** show_wifi:");
              fill_wifi_config_as_jsonData();
              // local copy
              String s = String(wifi_config_as_jsonData);
              s.replace("\n", "\r\n");
              Serial.print(s);
              Serial.printf("***\r\n");
              Serial.print("cmd:");
              inputBuf = "";
              return;
            } else if (cmd == "save_wifi_cfg") {
              Serial.println("*** save_preferences_cfg:");
              fill_wifi_config_as_jsonData();
              if (!wifi_config_as_jsonData.isEmpty()) {
                int ret = save_to_file("TNC", "/wifi.cfg", wifi_config_as_jsonData);
                if (ret >= 0)
                  Serial.println("*** save_wifi_cfg: ok");
                else
                  Serial.printf("*** save_wifi_cfg: error %d\r\n", ret);
              } else {
                  Serial.println("*** save_wifi_cfg: BUG (empty)");
              }
              Serial.print("cmd:");
              inputBuf = "";
              return;
            } else if (cmd == "dir") {
              if (SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
                listDir(SPIFFS, "/", 0);
                SPIFFS.end();
              } else {
                Serial.println("*** dir: SPIFFS Mount Failed");
              }
              Serial.printf("***\r\n");
              Serial.print("cmd:");
              inputBuf = "";
              return;
            }
          #endif
          if (arg != "") {
            Serial.println("*** " + cmd + ": error: not implemented, or argument not 'on' or 'off'");
            Serial.print("cmd:");
            inputBuf = "";
            return;
          }
        }

        if (arg == "" &&
            (cmd == "?" || cmd == "beacon" || cmd == "converse" || cmd == "display" || cmd == "reboot" || cmd == "shutdown" || cmd == "version")) {
          if (cmd == "beacon") {
            Serial.println("*** beacon: sending");
            manBeacon = 2;
          } else if (cmd == "converse") {
            do_prompt = false;
            Serial.println("*** converse: entering converse mode. Enabling LoRa RX packet trace. Hit ^C to leave");
            cmd_mode = false;
            // enable tnc trace
            usb_serial_data_type__had_traceing_enabled_before_entering_converse_mode = (usb_serial_data_type & 2) ? true: false;
            // We do not change preference setting
            usb_serial_data_type |= 2;
          } else if (cmd == "display" || inputBuf == "?") {
            Serial.println("*** display: I know the following commands:");
#ifdef ENABLE_WIFI
            Serial.println("  aprsis <on|off>");
            Serial.println("  beacon               (tx a beacon)");
#endif
            Serial.println("  display              (help)");
            Serial.println("  converse             (leave with ^C)");
            Serial.println("  echo <on|off>");
            Serial.println("  kiss on");
            Serial.println("  logging <on|off>");
#ifdef ENABLE_PREFERENCES
            Serial.println("  preferences          (needs to bei implemented)");
            Serial.println("  show_preferences     (shows preferences as json from flash)");
            Serial.println("  save_preferences_cfg (saves running config to /preferences.cfg in filesystem)");
            Serial.println("  show_wifi            (shows wifi settings as json from ram)");
            Serial.println("  save_wifi_cfg        (saves running wifi config to /wifi.cfg in filesystem)");
            Serial.println("  dir                  (lists SPIFFS directory)");
#endif
            Serial.println("  nmea <on|off>");
            Serial.println("  trace <on|off>");
            Serial.println("  reboot");
            Serial.println("  shutdown");
#ifdef ENABLE_WIFI
            Serial.println("  wifi <on|off>");
#endif
            Serial.println("  version");
            Serial.println("  ?                    (help)");
          } else if (cmd == "reboot") {
            enableOled_now();
            writedisplaytext("((REBOOT))","","rebooting","due to","cli command","");
#ifdef ENABLE_WIFI
            do_send_status_message_about_reboot_to_aprsis();
#endif
            Serial.println("*** reboot: rebooting!");
            #if defined(ENABLE_SYSLOG) && defined(ENABLE_WIFI)
              syslog_log(LOG_WARNING, String("usb-serial: reboot: user entered reboot command. Rebooting.."));
            #endif
            delay(500);
            ESP.restart();
          } else if (cmd == "shutdown") {
            // enableOled_now() is called by do_shutdown()
            Serial.println("*** shutdown: halting!");
            #if defined(ENABLE_SYSLOG) && defined(ENABLE_WIFI)
              syslog_log(LOG_WARNING, String("usb-serial: halting: user entered shutdown command. Shutdown.."));
            #endif
            do_shutdown();
          } else if (cmd == "version") {
            Serial.print("*** version: ");
            Serial.print(VERSION);
            Serial.print(". Hardware / compiled with features: ");
            Serial.println(compile_flags);
          }

        } else {

          if (cmd == "echo") {
            if (arg != "")
              local_echo = arg_bool;
            Serial.println("*** " + cmd + " is " + (local_echo ? "on" : "off"));
          } else if (cmd == "kiss") {
             if (arg != "" && arg_bool) {
               Serial.printf("KISS ON\r\n%c", 0xC0);
               Serial.flush();
               // point of no return for this function, until reboot
               // We do not change preference setting
               usb_serial_data_type = 0;
               inputBuf = "";
               return;
            } else {
              Serial.println("*** " + cmd + " is off");
            }
          } else if (cmd == "logging") {
            if (arg != "") {
              if (arg_bool) {
                usb_serial_data_type |= 1;
                usb_serial_data_type &= ~128;
              } else {
                // avoid going to kiss mode (!usb_serial_data_type)
                usb_serial_data_type &= ~1;
                if (!usb_serial_data_type)
                  usb_serial_data_type = 128;
              }
              #ifdef ENABLE_PREFERENCES
                preferences.putInt(PREF_DEV_USBSERIAL_DATA_TYPE, usb_serial_data_type);
                #if defined(ENABLE_SYSLOG)
                  if (debug_verbose)
                    syslog_log(LOG_DEBUG, String("FlashWrite preferences: handle_usb_serial_input() 1"));
                #endif
              #endif
            }
            Serial.println("*** " + cmd + " is " + ((usb_serial_data_type & 1) ? "on" : "off"));
          } else if (cmd == "trace") {
            if (arg != "") {
              if (arg_bool) {
                usb_serial_data_type |= 2;
                usb_serial_data_type &= ~128;
              } else {
                // avoid going to kiss mode (!usb_serial_data_type)
                usb_serial_data_type &= ~2;
                if (!usb_serial_data_type)
                  usb_serial_data_type = 128;
              }
              #ifdef ENABLE_PREFERENCES
                preferences.putInt(PREF_DEV_USBSERIAL_DATA_TYPE, usb_serial_data_type);
                #if defined(ENABLE_SYSLOG)
                  if (debug_verbose)
                    syslog_log(LOG_DEBUG, String("FlashWrite preferences: handle_usb_serial_input() 2"));
                #endif
              #endif
            }
            Serial.println("*** " + cmd + " is " + ((usb_serial_data_type & 2) ? "on" : "off"));
          } else if (cmd == "nmea") {
            if (arg != "") {
              if (arg_bool) {
                usb_serial_data_type |= 4;
                usb_serial_data_type &= ~128;
              } else {
                // avoid going to kiss mode (!usb_serial_data_type)
                usb_serial_data_type &= ~4;
                if (!usb_serial_data_type)
                  usb_serial_data_type = 128;
              }
              #ifdef ENABLE_PREFERENCES
                preferences.putInt(PREF_DEV_USBSERIAL_DATA_TYPE, usb_serial_data_type);
                #if defined(ENABLE_SYSLOG)
                  if (debug_verbose)
                    syslog_log(LOG_DEBUG, String("FlashWrite preferences: handle_usb_serial_input() 3"));
                #endif
              #endif
            }
            Serial.println("*** " + cmd + " is " + ((usb_serial_data_type & 4) ? "on" : "off"));
#ifdef ENABLE_WIFI
          } else if (cmd == "aprsis") {
            if (arg != "") {
              aprsis_enabled = arg_bool;
              preferences.putBool(PREF_APRSIS_EN, arg_bool);
              #if defined(ENABLE_SYSLOG)
                if (debug_verbose)
                  syslog_log(LOG_DEBUG, String("FlashWrite preferences: handle_usb_serial_input() 4"));
              #endif
            }
            Serial.println("*** " + cmd + " is " + (aprsis_enabled ? "on" : "off"));
          } else if (cmd == "wifi") {
            if (arg != "") {
              #ifdef ENABLE_PREFERENCES
                preferences.putInt(PREF_WIFI_ENABLE, arg_bool ? 1 : 0);
                #if defined(ENABLE_SYSLOG)
                  if (debug_verbose)
                    syslog_log(LOG_DEBUG, String("FlashWrite preferences: handle_usb_serial_input() 5"));
                #endif
              #endif
              if (arg_bool) {
                if (!webserverStarted) {
                  enable_webserver = 1;
                  #if defined(LORA32_21) && defined(ENABLE_BLUETOOTH)
                    // lora32_21 hardware bug: bt and wifi are mutual exclusive
                    SerialBT.end();
                    serial_bt_client_is_connected = false;
                    enable_bluetooth = false;
                    delay(100);
                  #endif
                  esp_task_wdt_reset();
                  webServerCfg = {.callsign = Tcall};
                  xTaskCreate(taskWebServer, "taskWebServer", 12000, (void*)(&webServerCfg), 1, nullptr);
                  webserverStarted = true;
                  writedisplaytext("LoRa-APRS","","TNC:","WiFi task started","long press button","to stop again");
                  esp_task_wdt_reset();
                  delay(1500);
                  esp_task_wdt_reset();
                }
              } else {
                enable_webserver = 0;
              }
            }
            Serial.println("*** " + cmd + " is " + (enable_webserver ? "on" : "off"));
#endif // ENABLE_WIFI
          } else {
            Serial.println("*** ?");
          }
        }
      }
      inputBuf = "";

    } else {

      // converse mode

      const char *p = inputBuf.c_str();
      const char *q, *r;

      // user pressed enter, without any character before`
      if (!*p)
        return;

      if ((*p && ((*p >= 'A' && *p <= 'Z') || (*p >= '0' && *p <= '9')))
          && (q = strchr(p, '>')) && isalnum(q[1]) && (r = strchr(p, ':')) && r > q && r[1] > ' ') {
        for ( ++p; *p && *p != ':'; p++) {
          if (! (*p == '-' || *p == '>' || *p == ',' || *p == '*' || (*p >= 'A' && *p <= 'Z') || (*p >= '0' && *p <= '9')))
            break;
        }
        if (p == r) {
          Serial.println("*** sending: '" + inputBuf + "'");
          stats_from_serial++;
          #ifdef KISS_PROTOCOL
            sendToTNC(inputBuf);
            esp_task_wdt_reset();
          #endif
          #if defined(ENABLE_WIFI)
            send_to_aprsis(inputBuf);
            esp_task_wdt_reset();
          #endif
          if (lora_tx_enabled) {
            int do_delay = 0;
            enableOled_now(); // enable OLED
            writedisplaytext("((KISSTX))","",inputBuf,"","","");
            if (tx_own_beacon_from_this_device_or_fromKiss__to_frequencies % 2) {
              loraSend(txPower, lora_freq, lora_speed, 0, inputBuf);  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
              stats_tx_freq_main++;
              do_delay = 600000 / lora_speed;
            }
            if (((tx_own_beacon_from_this_device_or_fromKiss__to_frequencies > 1 && lora_digipeating_mode > 1) || tx_own_beacon_from_this_device_or_fromKiss__to_frequencies == 5) && lora_freq_cross_digi > 1.0 && lora_freq_cross_digi != lora_freq) {
              loraSend(txPower_cross_digi, lora_freq_cross_digi, lora_speed_cross_digi, 0, inputBuf);  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
              stats_tx_freq_secondary++;
              if (!do_delay || lora_speed_cross_digi < lora_speed)
                do_delay = 600000 / lora_speed_cross_digi;
            }
            if (do_delay) {
              delay(do_delay);
              esp_task_wdt_reset();
            }
          } else {
            Serial.println("*** Warning: lora tx must be enabled! Not sending to RF");
          }
        } else {
          Serial.println("*** refusing to send bad user input: '" + inputBuf + "'");
        }
      } else {
        Serial.println("*** refusing to send bad user input: '" + inputBuf + "'");
      }
      inputBuf = "";

    }

  } else {

    // user entered a character

    if (local_echo)
      Serial.print(c);

    if (inputBuf.length() > 255) {
      Serial.println("\r\n*** Error: Line too long");
      inputBuf = "";
    } else {
      // add character to inputBuf
      inputBuf += String(c);
    }

  }

  if (do_prompt)
    Serial.print("cmd:");

  Serial.flush();
}

String handle_aprs_messsage_addressed_to_us(const char *received_frame) {
  char *header_end;
  const char *header_normal_or_third_party_start;
  char *header_normal_or_third_party_end;
  char *q;

  sema_lock_or_unlock__handle_aprs_message_addressed_to_us(1);

  String answer_message = "";
  if (!received_frame)
    goto end;
  header_end = strchr(received_frame, ':');
  if (!header_end)
    goto end;
  header_normal_or_third_party_start = received_frame;
  header_normal_or_third_party_end = header_end;

  // handle messages addressed to us. May be called after lora RF receiption, or when parsing an aprsis message
  if (header_end[1] == '}') {
    header_normal_or_third_party_start = header_end+2;
    header_normal_or_third_party_end = strchr(header_normal_or_third_party_start, ':');
    if (!header_normal_or_third_party_end)
      goto end;
  }

  if (strlen(header_normal_or_third_party_end+1) > 11 &&
      header_normal_or_third_party_end[1] == ':' &&
      header_normal_or_third_party_end[11] == ':') {
    // this is an aprs message. Is this message for us? Then we will not digipeat and we will not send it to aprsis
    // Example: WLNK-1>APWLK,TCPIP*,qAC,T2MCI::DL9SAU-12:You have 1 Winlink mail messages pending{2077

    if (is_call_blacklisted(header_normal_or_third_party_start))
      goto end;

    char msg_to[12];
    sprintf(msg_to, ":%-9.9s:", Tcall.c_str());
    if (!strncmp(header_normal_or_third_party_end+1, msg_to, 11)) {
      char msg_from[10];
      answer_message = "M"; // M is a placeholder for aprs-message-sent-to-us
      strncpy(msg_from, header_normal_or_third_party_start, 9);
      msg_from[9] = 0;
      if ((q = strchr(msg_from, '>')))
        *q = 0;
      enableOled_now(); // enable OLED
      freeze_display = true;
      String RXMessageSender = String(msg_from);
      String RXMessageTimeAndSender = RXMessageSender + ":";
      String RXMessage = String(header_normal_or_third_party_end + 11+1);
      String RXMessageTimeStr = "";
      struct tm timeinfo;
      if (getLocalTimeTheBetterWay(&timeinfo)) {
        char buf[12]; // Room for "03-22 06:08" + \0
        strftime(buf, sizeof(buf), "%m-%d %H:%M", &timeinfo);
        RXMessageTimeStr = String(buf);
        // Add time to front
        RXMessageTimeAndSender = RXMessageTimeStr + " " + RXMessageTimeAndSender;
      }
      writedisplaytext(" ((MSG))",RXMessageTimeAndSender,RXMessage,"","","");
      if (add_winlink_notification &&
          !strncmp(header_normal_or_third_party_start, "WLNK-1", 6) && header_normal_or_third_party_start[6] == '>' &&
          !strncmp(header_normal_or_third_party_end + 12, "You have ", 9) &&
              strstr(header_normal_or_third_party_end + 20, " Winlink mail messages pending")) {
        // Don't overwrite a friend's nessage with winlinkInfo
        LastRXMessageInfo |= 2;
        if (!(LastRXMessageInfo & 1)) {
          // Store message
          LastRXMessageSender = String(RXMessageSender);
          LastRXMessageTimeStr = String(RXMessageTimeStr);
          LastRXMessage = String(RXMessage);
          LastRXMessageChanged = millis();
        }
        winlink_notified = millis();
      } else {
        // Store message
        LastRXMessageInfo |= 1;
        LastRXMessageSender = String(RXMessageSender);
        LastRXMessageTimeStr = String(RXMessageTimeStr);
        LastRXMessage = String(RXMessage);
        LastRXMessageChanged = millis();
      }
#ifdef ENABLE_WIFI
      // TODO: add message to a new-to-implement web-received-list-for-aps-messages
#endif

      // Messages with message number require an ack. We will only ack if we have no bluetooth client like aprsdroid, who will ack by himself
      if ((q = strchr(header_normal_or_third_party_end + 11+1, '{'))
#ifdef ENABLE_BLUETOOTH
           && !serial_bt_client_is_connected
#endif
        ) {
        // Before sending the ack to RF and aprsis, we first have to do other things (like send to tnc). That's why we
        // store the message here as answer-String

        // Add trailing blanks for the message, due to aprs spec.
        for (int i = strlen(msg_from); i < 9; i++) {
          msg_from[i] = ' ';
        }
        msg_from[9] = 0;
        answer_message = Tcall + ">" + MY_APRS_DEST_IDENTIFYER + "::" + String(msg_from) + ":ack" + String(q+1);
      }
    }
  }

end:
  sema_lock_or_unlock__handle_aprs_message_addressed_to_us(0);
  return String(answer_message);
}

void aprspos2double(char *lh_lat_aprs, char *lh_lng_aprs, double &lh_lat, double &lh_lng) {
  // conversion due to DL9SAU

  char buf[9];
  double decimals;
  boolean south = false;
  boolean west = false;

  strncpy(buf, lh_lat_aprs, 8);

  if (buf[7] == 'S') south = true;

  buf[7] = 0; // null terminate at end of decimal minute
  decimals=atof(buf+2) /60.0;
  buf[2] = 0; // null terminate az begin of minute
  lh_lat = atoi(buf) + decimals;
  if (south) lh_lat *= -1;

  strncpy(buf, lh_lng_aprs, 9);
  if (buf[8] == 'W') west = true;
  buf[8] = 0; // null terminate at end of decimal minute
  decimals=atof(buf+3) /60.0;
  buf[3] = 0; // null terminate az begin of minute
  lh_lng = atoi(buf) + decimals;
  if (west) lh_lng *= -1;

}


// Store calls and their position in LH. Used for displaying distance and course
void fill_lh(const String &rxcall, const char *digipeatedflag, const char *p) {
  double lh_lat = 0;
  double lh_lng = 0;

  char lh_lat_aprs[9];
  char lh_lng_aprs[10];
  int pos_type = 0;
  boolean lh_position = false;

  if (p[0] == '!' || p[0] == '=') {
    pos_type = 1;
    p++;
  } else if (p[0] == '/' || p[0] == '@') {
    // Pos Type 2 detected
    pos_type = 2;
    p += 8;
  } else {
    // Pos Type 3 detected
    pos_type = 3;
  }

  if (pos_type == 1 || pos_type == 2) {
    lh_position = true;
    if (isdigit(*p)) {
      strncpy(lh_lat_aprs, p, 8);
      lh_lat_aprs[8] = 0;
      p += 9;
      strncpy(lh_lng_aprs, p, 9);
      lh_lng_aprs[9] = 0;
      lh_lat = 0;
      lh_lng = 0;
      aprspos2double(lh_lat_aprs, lh_lng_aprs, lh_lat, lh_lng);
    } else {
      // compressed position
      long n;
      p++; // skip symbol id
      n = 0;
      for (int i = 3; i >= 0; i--) {
        n = n + ( *p - 33 ) * pow(91, i);
        p++;
      }
      lh_lat = double ( 90 - n / 380926.0);
      n = 0;
      for (int i = 3; i >= 0; i--) {
        n = n + ( *p - 33 ) * pow(91, i);
        p++;
      }
      lh_lng = double ( -180 + n / 190463.0);
    }
  } else {
    lh_lat = 0;
    lh_lng = 0;
    lh_position = false;
  }

  if (lh_position) {
    int ii;
    for (ii=0; ii < (MAX_LH-1); ii++) {
      if (rxcall == LH[ii].callsign) {
        break;
      }
    }

    for (; ii > 0; ii--) {
      LH[ii] = LH[ii-1];
    }

    LH[0].callsign = String(rxcall);
    LH[0].time_received = millis()/1000;
    LH[0].lat = lh_lat;
    LH[0].lng = lh_lng;
    LH[0].direct = digipeatedflag ? false : true;
  }
}

void write_last_heard_calls_with_distance_and_course_to_display() {
  String lines[5];
  //char dist_and_course[9];
  char line[OLED_LINE_LEN_MAX+1]; // Display length 21 + \0
  char course[3];
  double courseTo;
  double distTo;
  fillDisplayLines3to5(1);
  for (int i=0; i < MAX_LH; i++) {
    if (LH[i].callsign != "") {
      courseTo = TinyGPSPlus::courseTo(aprsLatPresetDouble, aprsLonPresetDouble, LH[i].lat, LH[i].lng);
      if (courseTo < 22.5 || courseTo >= 337.5) sprintf(course, "N");
      else if (courseTo < 67.5) sprintf(course, "NE");
      else if (courseTo < 112.5) sprintf(course, "E");
      else if (courseTo < 157.5) sprintf(course, "SE");
      else if (courseTo < 202.5) sprintf(course, "S");
      else if (courseTo < 247.5) sprintf(course, "SW");
      else if (courseTo < 292.5) sprintf(course, "W");
      else /* if (courseTo < 337.5) */ sprintf(course, "NW");

      distTo = TinyGPSPlus::distanceBetween(aprsLatPresetDouble, aprsLonPresetDouble, LH[i].lat, LH[i].lng) / 1000.0f;

      // show distance accoring to user's preferred unit
      switch (units_speed) {
      case UNITS_SPEED_MPH:
        distTo /= 1.609344;
        break;
      case UNITS_SPEED_KN:
        distTo /= 1.852;
        break;
      }

      // space on display
      // 012345678901234567890
      // DB0ABC-10*10m 0001 SE
      if (distTo < 10.0) {
        //sprintf(dist_and_course, " %4.2f %-2s", distTo, course);
        sprintf(line, "%-9s%c%3s  %4.2f%2s", LH[i].callsign.c_str(), LH[i].direct ? ':' : '*', compute_time_since_received(millis()/1000.0L - LH[i].time_received).c_str(), distTo, course);
      } else if (distTo < 100.0) {
        //sprintf(dist_and_course, " %4.1f %-2s", distTo, course);
        sprintf(line, "%-9s%c%3s %4.1f %2s", LH[i].callsign.c_str(), LH[i].direct ? ':' : '*', compute_time_since_received(millis()/1000.0L - LH[i].time_received).c_str(), distTo, course);
      } else if (distTo < 999.49) {
        //sprintf(dist_and_course, "  %03.0f %-2s", distTo, course);
        sprintf(line, "%-9s%c%3s %03.0f  %2s", LH[i].callsign.c_str(), LH[i].direct ? ':' : '*', compute_time_since_received(millis()/1000.0L - LH[i].time_received).c_str(), distTo, course);
      } else {
        //sprintf(dist_and_course, " >999 %-2s", course);
        sprintf(line, "%-9s%c%3s >999 %2s", LH[i].callsign.c_str(), LH[i].direct ? ':' : '*', compute_time_since_received(millis()/1000.0L - LH[i].time_received).c_str(), course);
      }
      //lines[i] = LH[i].callsign + (LH[i].direct ? ":" : "*") + compute_time_since_received(millis()/1000.0L - LH[i].time_received) + dist_and_course;
      lines[i] = String(line);
    } else {
      lines[i] = "";
    }
  }
  writedisplaytext("((LH))",lines[0],lines[1],lines[2],lines[3],lines[4]);
}


// +---------------------------------------------------------------------+//
// + MAINLOOP -----------------------------------------------------------+//
// +---------------------------------------------------------------------+//
void loop()
{
  double curr_knots = 0.0;
  double curr_kmph = 0.0;
  double curr_hdop = 99.9;
  int curr_sats = 0;
  boolean digipeated_on_cross_freq;

  esp_task_wdt_reset();

  digipeated_on_cross_freq = false;

  if (reboot_interval && millis() > reboot_interval) {
    // enforce display wakeup (because we will not reach the timer function for setting it on)
    enableOled_now();
    writedisplaytext("((REBOOT))","","rebooting","due to","configured","interval");
#ifdef ENABLE_WIFI
    do_send_status_message_about_reboot_to_aprsis();
#endif
    #if defined(ENABLE_SYSLOG) && defined(ENABLE_WIFI)
      syslog_log(LOG_WARNING, String("reboot_timer: reboot: configured timer expired. Rebooting.."));
    #endif
    delay(2000);
    ESP.restart();
  }

  sendpacket_was_called_twice = false;

  timer_once_a_second();

#ifdef ENABLE_BLUETOOTH
  if (serial_bt_client_is_connected && add_winlink_notification == 1 && new_serial_bt_client_connected) {
    // trigger winlink notification
    time_last_status_packet_sent = 0L;
  }
#endif

  if (digitalRead(BUTTON) == LOW && button_key_up == true) {
    button_key_up = false;
    delay(50);
    Serial.println("Tracker: Button pressed...");

    button_down_count += 1;
    #ifdef DEVELOPMENT_DEBUG
      if (debug_verbose > 1) Serial.printf("Button pressed %dx (1)\r\n", button_down_count);
    #endif
    if (digitalRead(BUTTON) == LOW) {
      delay(300);
      button_time_delay = millis() + 1500;
      #ifdef DEVELOPMENT_DEBUG
        if (debug_verbose > 1) Serial.printf("Button has been pressed %dx (2)\r\n", button_down_count);
      #endif
      if (digitalRead(BUTTON) == HIGH) {
        if (shutdown_active && shutdown_countdown_timer_enable) {
          enableOled_now(); // turn ON OLED now
          shutdown_countdown_timer_enable = false;
          #if defined(T_BEAM_V1_0) || defined(T_BEAM_V1_2)
            writedisplaytext("((ABORT))","","Shutdown aborted:","middle Button","was pressed","");
          #else
            writedisplaytext("((ABORT))","","Shutdown aborted:","User Button","was pressed","");
          #endif
          button_key_up = true;
          button_down_count = 0;
        } else if (!(display_is_on && enabled_oled)) {
            #if defined(HELTEC_WIRELESS_TRACKER) && defined(VEXT_CTRL) && defined(HAS_TFT)
              if (gps_state && !gps_task_enabled) {
                // On heltec wireles tracker we need to switch on VEXT_CTL on again if it was off
                // GPS also wakes up, because it's controlled by the same pin
                digitalWrite(VEXT_CTRL, HIGH);
                do_poweron_gps(0);
                // Display was also turned off. Reinitialize..
                do_tft_init();
                fillDisplayLine1(5);
                fillDisplayLine2();
                fillDisplayLines3to5(1);
                display_is_on = false; // needed below (for state wake up: show main screeen, not ((LH)))
              }
            #endif
          write2display();
          enableOled_now(); // turn ON OLED now
          button_down_count = 0;
        } else {
          enableOled(); // rewind oled_timer
          freeze_display = true;
          #ifdef DEVELOPMENT_DEBUG
            if (debug_verbose > 1) Serial.printf("Button Count (Oled on): %d\r\n", button_down_count);
          #endif
          if (button_down_count == 1) {
            write_last_heard_calls_with_distance_and_course_to_display();
            // double click? -> Send beacon
            delay(500);
            if (digitalRead(BUTTON) == LOW) {
              goto send_beacon;
            }
          } else if (button_down_count == 2) {
            String RXMessageTimeAndSender = "";
            if (!LastRXMessageSender.isEmpty()) {
              String RXMessageTimeStr;
              if (LastRXMessageTimeStr.isEmpty()) {
                RXMessageTimeStr = compute_time_since_received(millis()/1000L - LastRXMessageChanged/1000L) + " ago";
              } else {
                RXMessageTimeStr = LastRXMessageTimeStr;
              }
              RXMessageTimeAndSender = RXMessageTimeStr + " " + LastRXMessageSender + ":";
            }
            writedisplaytext("((MSG))",RXMessageTimeAndSender,LastRXMessage,"","","");
            // Message has been displayed, we can remove the "new message-indicator"
            if (LastRXMessageInfo & 2) {
              // last received was a winlink notification and we just read that message
              winlink_notified = 0L;
            }
            LastRXMessageInfo = 0;
          } else if (button_down_count < 6) {
            int n = button_down_count-3;
            writedisplaytext("RX raw-" + String(n+1) + String(RX_RAW_PACKET_LIST[n].QRG ? ( " (" + String(RX_RAW_PACKET_LIST[n].QRG) + ")" ) : "") ,"",RX_RAW_PACKET_LIST[n].packet,"","","");
            time_to_refresh = millis() + showRXTime;
          } else if (button_down_count == 6) {
            //String stats = "RX" + String(stats_rx_freq_main) + "+" + String(stats_rx_freq_secondary) + " TX" + String(stats_tx_freq_main) + "+" + String(stats_tx_freq_main) + "_b" + String(stats_tx_beacon) + "_d" + String(stats_digipeated_rf2rf) + " I" + String(stats_from_aprsis) + "+" + String(stats_to_aprsis);
            String stats = "";
            if (stats_rx_freq_main || stats_rx_freq_secondary) {
              stats += "R:";
              if (stats_rx_freq_main)
                stats += String(stats_rx_freq_main);
              stats += "/";
              if (stats_rx_freq_secondary)
                stats += String(stats_rx_freq_secondary);
              stats += " ";
            }
            if (stats_tx_freq_main || stats_tx_freq_secondary) {
              stats += "T:";
              if (stats_tx_freq_main)
                stats += String(stats_tx_freq_main);
              stats += "/";
              if (stats_tx_freq_secondary)
                  stats += String(stats_tx_freq_secondary);
              if ((stats_tx_freq_main && stats_tx_beacon != stats_tx_freq_main) || (stats_tx_freq_secondary && stats_tx_beacon != stats_tx_freq_secondary))
                stats = stats + "b" + String(stats_tx_beacon);
              if ((stats_tx_freq_main && stats_tx_beacon != stats_digipeated_rf2rf) || (stats_tx_freq_secondary && stats_digipeated_rf2rf != stats_tx_freq_secondary))
                stats = stats + "d" + String(stats_digipeated_rf2rf);
              stats += " ";
            }
            if (stats_from_aprsis || stats_to_aprsis)
               stats = stats + "I:" + String(stats_from_aprsis) + "/" + String(stats_to_aprsis);
            writedisplaytext("((BN))","BuildNr:" + buildnr,"by DL9SAU & DL3EL",stats,"next press: tx bcn","or wait ...");
          } else if (button_down_count == 7) {
send_beacon:
            button_down_count = 0;
            if (lora_tx_enabled || aprsis_enabled || usb_serial_data_type == 0 || usb_serial_data_type & 2) {
              freeze_display = false;
              fillDisplayLines3to5(1);
              if (gps_state && gps_isValid) {
#ifdef ENABLE_WIFI
                writedisplaytext("((MAN TX))","SSID: " + oled_wifi_SSID_curr, "IP: " + oled_wifi_IP_curr, OledLine3, OledLine4, OledLine5);
#else
                fillDisplayLines3to5(0);
                writedisplaytext("((MAN TX))","","",OledLine3, OledLine4, OledLine5);
#endif
                sendpacket(SP_POS_GPS);
              } else {
#ifdef ENABLE_WIFI
                writedisplaytext("((FIX TX))","SSID: " + oled_wifi_SSID_curr, "IP: " + oled_wifi_IP_curr, OledLine3, OledLine4, OledLine5);
#else
                fillDisplayLines3to5(0);
                writedisplaytext("((FIX TX))","","",OledLine3, OledLine4, OledLine5);
#endif
                sendpacket(SP_POS_FIXED);
              }
              // reset timer for automatic fixed beacon after manual beacon
              next_fixed_beacon = millis() + fix_beacon_interval;
            }
          } else {
            #ifdef DEVELOPMENT_DEBUG
              if (debug_verbose > 1) Serial.printf("Button pressed check >= 6: %d, shouldn't come here\r\n", button_down_count);
            #endif
            button_down_count = 0;
          }
          button_key_up = true;
        }
        #ifdef DEVELOPMENT_DEBUG
          if (debug_verbose > 1) Serial.printf("Button still down %dx \r\n", button_down_count);
        #endif
      } else {
        #ifdef DEVELOPMENT_DEBUG
          if (debug_verbose > 1) Serial.printf("Button now up %dx \r\n", button_down_count);
        #endif
      }
    } else {
      #ifdef DEVELOPMENT_DEBUG
        if (button_down_count > 0) {
          Serial.printf("Button press was lost...(%d)\r\n", button_down_count);
        }
      #endif
    }
  }


  // time to wake up a sleeping GPS?
  if (gps_state && gps_may_sleep && (gps_may_sleep > 1 ||
          #ifdef T_BEAM_V1_0
            !axp.isVBUSPlug()
          #elif T_BEAM_V1_2
            !axp.isVbusInsertOnSource()
          #else
            1
          #endif
      )) {
    static uint32_t t_gps_powersave_operation__next_action = 0L;
    static uint8_t gps_wakeups = 3; // initial sleep will be 8min
    static uint32_t t_last_movement = 0L;
    boolean do_suspend_gps = false;
    boolean do_resume_gps = false;

    if (gps_task_enabled && gps_isValid && t_gps_fix_lost == 0L && t_gps_got_first_fix_after_last_loss > 0L) {
      static double lastStopAfterMoveDoubleLat = bestDoubleLat;
      static double lastStopAfterMoveDoubleLng = bestDoubleLng;
      float dist = TinyGPSPlus::distanceBetween(lastStopAfterMoveDoubleLat, lastStopAfterMoveDoubleLng, bestDoubleLat, bestDoubleLng);

      // GPS is valid
      // -> reset gps_wakups to 3 -> initial sleep after loss will be 8min
      gps_wakeups = 3;
      // make sure that the next_action timer is reset. This mitigates the case
      // when gps fix is lost and timer is -15min (which could lead to an immediate gps sleep condition
      // in the gps toggle part below)
      t_gps_powersave_operation__next_action = 0L;

      // Movement detection: poweroff gps after detected speed or distance >limit
      if (t_last_movement > 0L && millis() - t_last_movement > 5*60*1000L &&
           ( (dist < 185.2 && millis() - t_gps_got_first_fix_after_last_loss > 3*60*1000L) ||
             (curr_kmph < (sb_min_speed > 7.2 ? sb_min_speed/2 : 3.6) && millis() - t_gps_got_first_fix_after_last_loss > 1*60*1000L) ) ) {

        // t_gps_got_first_fix_after_last_loss > 1min also ensures that gps does not run for only a few seconds.
        // gps should sleep
        do_suspend_gps = true;
        // No-movement-case: starts with Ratio 4min sleep 5min awake, then
        // (if no fix) 8min sleep, 8.75min awake, or (if fix and still not moving) this ratio again
        gps_wakeups = 2;
      } else {
        if (t_last_movement == 0L || dist >= 185.2 || curr_kmph >= (sb_min_speed > 7.2 ? sb_min_speed/2 : 3.6)) {
          t_last_movement = millis();
          // store last position only on these cases: initial (t_last_movement == 0), or (dist or speed) < limit
          // So we always compare the distance between current lat/lon and last movement.
          // So even if we move slow, we can detect this (which would not be the case
          // if we would always store lat/lon)
          lastStopAfterMoveDoubleLat = bestDoubleLat;
          lastStopAfterMoveDoubleLng = bestDoubleLng;
        }
      }
    } else {
      // GPS fix lost. Go to normal cyclic gps toggle below
      t_last_movement = 0L;
    }

    if (!do_suspend_gps && t_gps_fix_lost > 0L) {
      if (gps_task_enabled && millis() - t_gps_fix_lost > 10*60*1000L) {
        // new powersave operation cycle, because we lost gps fix 10min ago
        do_suspend_gps = true;
      } else if (t_gps_powersave_operation__next_action > 0L && millis() > t_gps_powersave_operation__next_action) {
        // powersave operation on/off toggles
        if (gps_task_enabled) {
          do_suspend_gps = true;
        } else {
          do_resume_gps = true;
        }
      }
    }


    // Power saving capabilities with sleep-if-no-gps-fix:
    // ((8.75*2+10)+6.25*3+5.41*3+5*3) / (((8.75*2+10)+6.25*3+5.41*3+5*3) + (8*3+1*3+2*3+4*3)) = 0.63 % power
    if (do_suspend_gps) {
      do_poweroff_gps();
      // No fix? Sleep intervals 8min, 1min, 2min, 4min, 8min, 1min, 2min, ..
      t_gps_powersave_operation__next_action = millis() + (1 << (gps_wakeups % 4)) * 60*1000L;
    } else if (do_resume_gps) {
      do_poweron_gps(1);
      // Keep running first try: 8.75min, 6.25min, 5.41min, 5min, 8.75min, 6.25min, ..
      // No gps fix since 3 rounds? -> Give once (every 4 rounds) a higher grace time for getting a fix (10min)
      gps_wakeups++;
      //t_gps_powersave_operation__next_action = millis() + ((gps_wakeups % 12) ? (2*60*1000L / ((gps_wakeups % 4) + 1)) : 10*60*1000L);
      t_gps_powersave_operation__next_action = millis() + ((gps_wakeups % 12) ? ((5*60*1000L / ((gps_wakeups % 4) + 1))+3.75*60*1000L) : 10*60*1000L);
    }
  }


  // Time to adjust time?
  int8_t t_hour_adjust_next = -1;
  if (gps_state && gps.time.isValid() && gps.time.isUpdated() &&
       gps.time.second() < 59 &&
       (t_hour_adjust_next == -1 || t_hour_adjust_next == gps.time.hour()) &&
       gps.time.age() < 500) {
    struct tm t;
    timeval tv = { 0, 0 };;
    t.tm_year = gps.date.year()-1900;
    t.tm_mon = gps.date.month()-1;	// Month, 0 - jan
    t.tm_mday = gps.date.day();	// Day of the month
    t.tm_hour = gps.time.hour();
    t.tm_min =  gps.time.minute();
    t.tm_sec = gps.time.second();
    if ((tv.tv_sec = mktime(&t)) != (time_t ) -1 && settimeofday(&tv, NULL) != -1)
      t_hour_adjust_next = (t.tm_hour + 1) % 24;
  }

  // time is now taken from system time and updates once per second in display_blinker

  // LatShownP  = gg-mm.dd[N|S]
  // aprsLatPreset = ggmm.dd[N|S]
  boolean gps_isValid_oldState = gps_isValid;
  gps_isValid = false;

  // do this before the curr_xxx=-Assignments below, because they set the gps.speed.isUpdated() value to false
  if (gps_state && gps.speed.isValid() && gps.speed.isUpdated() && gps.speed.age() < 10000L) {
    update_speed_from_gps();
  } // else: assume we still move with last speed

  // refresh speed and hdop
  curr_kmph = (gps.speed.isValid() ? gps.speed.kmph() : 0.0);
  curr_knots = (gps.speed.isValid() ? gps.speed.knots() : 0.0);
  curr_hdop = (gps.hdop.isValid() ? gps.hdop.hdop() : 99.9);
  curr_sats = gps.satellites.value();

  if (gps_state && gps.location.isValid()) {
    // store best hdop in a speed dependent time range
    static uint32_t t_interval_start = millis();
    int do_update = 0;

    if (gps.location.age() < 10000L) {
      gps_isValid = true;
    }

    if (gps_isValid && gps.location.isUpdated()) {

      if (curr_hdop < bestHdop || millis() > t_interval_start + 5*60*1000L) {
        // remember location with best hdop in this time range
        bestRawLat = gps.location.rawLat();
        bestRawLng = gps.location.rawLng();
        bestDoubleLat = gps.location.lat();
        bestDoubleLng = gps.location.lng();
        bestHdop = curr_hdop;
        if (t_gps_got_first_fix_after_last_loss == 0L)
          t_gps_got_first_fix_after_last_loss = millis();
      }

      uint32_t t_elapsed = millis() - t_interval_start;

      //if (t_elapsed > 15000L && ((curr_hdop < 1.5 && curr_sats >= 5) || gps_isValid != gps_isValid_oldState || no_gps_position_since_boot)) {
      if (t_elapsed > 15000L && ((curr_hdop < 1.5 && curr_sats >= 4) || gps_isValid != gps_isValid_oldState || no_gps_position_since_boot)) {
        // Approach to avoid gps inaccuracy (we observed bad gps positions in a range of 30m, or more):
        // Resolution of GPS is +/- 3 to 5m. In 1s at 10m/s (= 36 km/h) we are in 'best' case still
        // in behalf the resolution of GPS.
        do_update = 1;
      } else if (t_elapsed > 400L && gps.speed.isValid() &&
            ( (curr_knots > 20.0 && curr_sats >= 4) ||
              (curr_hdop < 1.5 && curr_sats >= 5 && curr_kmph > 1.8 && curr_kmph <= 16.0) ) ) {
        // aprs resolution is 1sm/100 = 1852m/100 = 18.52m
        // Updating this every 200ms would be enough for getting this resolution at speed of 18.52*3.6*5 = 333.36km/h.
        do_update = 1;
      } else if (millis() >= next_fixed_beacon && curr_sats >= 5 &&
        TinyGPSPlus::distanceBetween(bestDoubleLat, bestDoubleLng, gps.location.lat(), gps.location.lng()) > 185.2)  {
        // This hack tries to avoid position jumps due to inaccurate gps measurement, but it updates if we moved 185.2m in 10min
        do_update = 1;
      }

      if (do_update) {
        t_gps_fix_lost = 0L;
        if (!no_gps_position_since_boot || (latlon_precision > 0 && bestHdop < 1.0 && gps.speed.age() < 2000 && curr_knots < 18.0)) {
          // DAO: heigher precision 1/1000 arc-minute, if not > 36 knots (valid gps measurered speed). Idea behind:
          // 18.52 m/s are 36kn. We need abt 1s-2s time for understanding the whole displayed line -> resolution of > 2 decimal points is not needed
          // If we consider gps age of < 2s, we use 18kt as limit
                storeLatLonPreset(create_lat_aprs("-", bestRawLat, (latlon_precision == 2 ? 4 : 3)),
                                create_long_aprs("-", bestRawLng, (latlon_precision == 2 ? 4 : 3)),
                            latlon_precision);
        } else {
          storeLatLonPreset(create_lat_aprs("-", bestRawLat, 2), create_long_aprs("-", bestRawLng, 2), 0);
        }
        // aprs compressed position has always a high precision (29.17 cm in latitude, 58.34 cm (or less) in longitude)
        store_compressed_position(bestDoubleLat, bestDoubleLng);

        aprsPresetShown = "";

        bestHdop = 99.9;
        t_interval_start = millis();
        no_gps_position_since_boot = false;
      }
    }
  }

  if (gps_isValid != gps_isValid_oldState) {
    // String functions are cpu consuming. We adust string aprsPresetShown only if we changed status.
    if (!gps_isValid) {
      t_gps_fix_lost = millis();
      t_gps_got_first_fix_after_last_loss = 0L;
      // update to the old values
      update_speed_from_gps();
      if (gps_state && gps.speed.age() < 2000 && curr_knots > 4.0 && curr_hdop < 1.5) {
        // if we stand still, we can keep high precision DAO. Only heigher precision's lat/lon values are comparable (due to rounding at next decimal)
        // Background: if we look at the _minutes_, i.e. 42.237N, it's rounded for aprs position to 42.24N.
        // DAO !W! or !w! needs the cut-off-string 42.23N for !W7x!.
        // Do this only on good hdop; with bad hdop, the gps springs in a wide circle.
        bestRawLat = gps.location.rawLat();
        bestRawLng = gps.location.rawLng();
        bestDoubleLat = gps.location.lat();
        bestDoubleLng = gps.location.lng();
        bestHdop = gps.hdop.hdop();
        if (aprsLatPresetDAO != create_lat_aprs("", bestRawLat, (latlon_precision == 2 ? 4 : 3)) || aprsLonPresetDAO != create_long_aprs("", bestRawLng, latlon_precision == 2 ? 4 : 3)) {
          storeLatLonPreset(create_lat_aprs("-", bestRawLat, 2), create_long_aprs("-", bestRawLng, 2), 0);
          store_compressed_position(bestDoubleLat, bestDoubleLng);
        }
      }
      aprsPresetShown = "p";
      displayInvalidGPS();
    } else {
      aprsPresetShown = "";
      // No GPS signal for a long time? Enforce tx:
      if ((millis() - lastPositionTX) > ((sb_max_interval - sb_min_interval) / 2))
        nextTX = sb_min_interval;
    }
  } else if (!gps_isValid) {
    // isValid change of previous run, and still invalid
    gps_speed = 0;
    //gps_speed_kmph_oled = 0;
  }


#ifdef ENABLE_WIFI
  // Show informations on WiFi Status, only once after state change
  if (wifi_connection_status_prev != wifi_connection_status) {
    enableOled(); // turn ON OLED temporary
    if (wifi_connection_status == WIFI_CONNECTED_TO_AP) {
      writedisplaytext("((WiFi))","WiFi Client Mode","SSID: " + oled_wifi_SSID_curr, "Pass: ********", "IP: " + oled_wifi_IP_curr, getSatAndBatInfo());
    } else if (wifi_connection_status == WIFI_SEARCHING_FOR_AP) {
      writedisplaytext("((WiFi))","WiFi Client Mode","SSID: " + oled_wifi_SSID_curr, "Not in sight!", "IP: none", getSatAndBatInfo());
    } else if (wifi_connection_status == WIFI_RUNNING_AS_AP) {
      writedisplaytext("((WiFi))","WiFi AP Mode","SSID: " + oled_wifi_SSID_curr, "Pass: " + oled_wifi_PASS_curr, "IP: " + oled_wifi_IP_curr, getSatAndBatInfo());
    } else {
      //writedisplaytext("((WiFi))","WiFi off","SSID: " + oled_wifi_SSID_curr, "Pass: " + oled_wifi_PASS_curr, "IP: " + oled_wifi_IP_curr, getSatAndBatInfo());
      writedisplaytext("((WiFi))","WiFi off","press key long","to enable","", getSatAndBatInfo());
    }
    wifi_connection_status_prev = wifi_connection_status;
    // initial fill of line2
    fillDisplayLine2();
  }
#endif

  if (manBeacon && (lora_tx_enabled || aprsis_enabled || usb_serial_data_type == 0 || usb_serial_data_type & 2)) {
    // Manually sending beacon from html page
    enableOled_now();
#ifdef ENABLE_WIFI
    writedisplaytext((manBeacon == 1 ? "((WEB TX))" : "((CLI TX))"),"SSID: " + oled_wifi_SSID_curr,"IP: " + oled_wifi_IP_curr, OledLine3, OledLine4, OledLine5);
#else
    fillDisplayLines3to5(0);
    writedisplaytext((manBeacon == 1 ? "((WEB TX))" : "((CLI TX))"),"","",OledLine3, OledLine4, OledLine5);
#endif
    sendpacket(SP_POS_GPS);
    manBeacon=0;
  }

  // Only wake up OLED when necessary, note that DIM is to turn OFF the backlight
  // avoid unnecessary display_dim_calls -> remember dim state
  if (display_is_on) {
    if (oled_timeout > 0 && millis() >= oled_timer) {
      // if enabled_oled is >0: oled_timer switch-of-time reached? -> dim the display
      // if enabled_oled is 0: if we booted, display is on and oled_timer is set. oled_timer switch-of-time reached? -> dim the display
      // -> condition is the same. We don't have to look if enabled_oled is true or false.
      display.dim(true);
      display_is_on = false;
      // mark state change
      oled_timer = 0L;
      button_down_count = 0;
      freeze_display = false;
    } else { // else: keep it on, esp. if oled_timeout is set to 0, regardles of oled_timer.
        if (freeze_display && millis() >= oled_timer + showRXTime) {
          // if oled_timeout is set to 0 return to main display after showRXTime
          freeze_display = false;
          button_down_count = 0;
          fillDisplayLine1(6);
          fillDisplayLine2();
          fillDisplayLines3to5(1);
        }
    }
  } else {
    // state change of oled timer? switch backlight on, if oled is enabled
    if (enabled_oled && oled_timer != 0L
          #if defined(HELTEC_WIRELESS_TRACKER) && defined(VEXT_CTRL) && defined(HAS_TFT)
            && (!gps_state || gps_task_enabled)
          #endif
          ) {
      display.dim(false);
      display_is_on = true;
    } // else: enabled_oled == false: never turn on. enabled_oled == true and oled_timer == 0L: recently turned off -> also no need to be turned on.
  }

  if (digitalRead(BUTTON)==LOW && button_key_up == false && millis() >= button_time_delay && button_t_lock == false) {
    // enable OLED
    enableOled_now();
    button_down_count = 0;
    freeze_display = false;
    //---------------
    button_t_lock = true;
    // re-enable webserver, if was set to off.
#ifdef ENABLE_WIFI
    if (!webserverStarted) {
      enable_webserver = 1;
      #if defined(LORA32_21) && defined(ENABLE_BLUETOOTH)
        // lora32_21 hardware bug: bt and wifi are mutual exclusive
        SerialBT.end();
        serial_bt_client_is_connected = false;
        enable_bluetooth = false;
        delay(100);
      #endif
      esp_task_wdt_reset();
      webServerCfg = {.callsign = Tcall};
      xTaskCreate(taskWebServer, "taskWebServer", 12000, (void*)(&webServerCfg), 1, nullptr);
      webserverStarted = true;
      writedisplaytext("LoRa-APRS","","Button:","WiFi task started","long press to ","stop again (->reboot)");
      esp_task_wdt_reset();
      delay(3000);
      esp_task_wdt_reset();
    } else {
      #ifdef HELTEC_WIRELESS_TRACKER
        writedisplaytext("LoRa-APRS","","Button:","Shutdown","","Release Button now!");
        delay(1000);
        do_shutdown(false);
      #else
        writedisplaytext("LoRa-APRS","","Button:","Rebooting,","for stoping WiFi.","Release Button now!");
        do_send_status_message_about_reboot_to_aprsis();
        delay(3000);
        ESP.restart();
      #endif
    }
#else
  #ifdef HELTEC_WIRELESS_TRACKER
    writedisplaytext("LoRa-APRS","","Button:","Shutdown","","Release Button now!");
    delay(1000);
    do_shutdown(false);
  #else
    writedisplaytext("LoRa-APRS","","Button:","Rebooting","","Release Button now!");
    #ifdef ENABLE_WIFI
      do_send_status_message_about_reboot_to_aprsis();
    #endif
    delay(3000);
    ESP.restart();
  #endif
#endif
  }

  if(digitalRead(BUTTON)==HIGH && !button_key_up){
    button_key_up = true;
    button_t_lock = false;
  }

  if (dont_send_own_position_packets) {
#ifdef KISS_PROTOCOL
    // reset to default state if kiss device is disconnected, silent or last position frame was seen long time ago (i.e. 1h).
    // TODO: also iterate through KISS tcp-devices in the parts below wih 'kiss_client_came_via_bluetooth'
    if (
#ifdef ENABLE_BLUETOOTH
         (kiss_client_came_via_bluetooth && !serial_bt_client_is_connected) ||
#endif
         (((time_last_own_position_via_kiss_received + sb_max_interval + 10*1000L) < millis()) &&
             time_last_own_position_via_kiss_received >= time_last_frame_via_kiss_received) ||
           // ^kiss client has not recently sent a position gain (sb_max_interval plus 10 seconds grace) and kiss client sent no other data
         ((time_last_frame_via_kiss_received + sb_max_interval * 2 + 10*1000L) < millis())) {
            // ^ kiss client sent no positions and stoped sending other data for 2*sb_max_interval (plus 10 seconds grace)
      if (!gps_state && gps_state_before_autochange)
        do_poweron_gps(1);
      gps_state = gps_state_before_autochange;
      if (gps_state_before_autochange)
        dont_send_own_position_packets = false;
      time_last_own_position_via_kiss_received = 0L;
      time_last_frame_via_kiss_received = 0L;
#ifdef ENABLE_BLUETOOTH
      if (kiss_client_came_via_bluetooth && !serial_bt_client_is_connected)
        kiss_client_came_via_bluetooth = false;
#endif
    }
#endif
  }

  // fixed beacon, or if smartbeaconing with lost gps fix (but had at least one gps fix).
  // smartbeaconing also ensures correct next_fixed_beacon time
  if ((lora_tx_enabled || aprsis_enabled || usb_serial_data_type == 0 || usb_serial_data_type & 2) && !dont_send_own_position_packets && millis() >= next_fixed_beacon &&
       (fixed_beacon_enabled ||
       ((!gps_state || !gps_isValid) && lastPositionTX && lastPositionTX + sb_max_interval < millis()) ) ) {
    enableOled_now(); // enable OLED
    fillDisplayLines3to5(0);
    writedisplaytext("((AUT TX))", "", "fixed", OledLine3, OledLine4, OledLine5);
    sendpacket(SP_POS_FIXED);
  }


#ifdef notdef // unfortunately. against their own documentation, winlink ignores status packets.
              // Status packet notification request would be a much better approch than in position comment.
              // Also testet: they inform only once a day. To one and not any other of your ssid's sending "winlink" in comment.
              // And they don't honor if you don't send acks. This would have been nice, because then at least one of
              // your other devices may get a notification. And because most lora-gatewas are rx-only, we requested for
              // notification, but our ack (if implemented), or by connected aprsdroid, does not go through. They should
              // assume that if no ack is received, no notification was received at the notified ssid.
              // Another example is: lora with aprsdroid, and 2m portable. lora device got notified, but aprsdroid is not
              // running -> no ack. -> Notification to the 2m portable.
  if ((time_last_status_packet_sent + 3600000L < millis()) || (time_last_status_packet_sent == 0L && millis() > 5*60*1000L)) {
    String msg = "";
    if (add_winlink_notification &&
         (add_winlink_notification == 2
#ifdef ENABLE_BLUETOOTH
             || serial_bt_client_is_connected
#endif
        ) ) {
       msg = "WLNK-1 qru?";
    }
    if (!msg.isEmpty()) {
      msg = msg + String(" B") + buildnr + String(",up:") + String((int ) (millis()/1000/60));
      sendStatusPacket(msg);
    } else {
      time_last_status_packet_sent = millis();
    }
  }
#endif

  // On T-Beam >= V1.0, we have the AXP chip which does good battery management.
  // And we can inform if USB power is pluggd in/off, and do a shutdown (if configured) on plug off.
  // On other devices it's a good ide to do use the esp32_depp_sleep function in order to prevent
  // battery damage. We "re-used" the usb_status variable from the previous T-Beam implementation
  // On a device with AXP, InpVolt > 4.3 indicates we run on external power; else: on battery.
  //   The AXP chip does his job and prevents deep discharge; we don't need here to handle this case.
  // On a device without AXP, InpVolt > 3.4 indicates our battery charge is still ok; else: enter deep sleep.
  #if defined(T_BEAM_V1_0) || defined(T_BEAM_V1_2)
    //if (InpVolts > 4.3) {  // No, my usb hub suddenly only made 4.1V
    if (InpVolts > 4.1) {
  #else
    if (InpVolts > 3.4) {
  #endif
      if (!usb_status_before) {
        enableOled_now(); // Turn Oled on as indicatior that external power is plugged on
        if (shutdown_active && shutdown_countdown_timer_enable) {
          shutdown_countdown_timer_enable = false;
          #if defined(T_BEAM_V1_0) || defined(T_BEAM_V1_2)
            writedisplaytext("((ABORT))","External Power","plugged in","Shutdown aborted","","");
          #else
            writedisplaytext("((ABORT))","External Power","Voltage ok","Shutdown aborted","","");
          #endif
          freeze_display = true;
        } else {
          #if defined(T_BEAM_V1_0) || defined(T_BEAM_V1_2)
            writedisplaytext("((POWER))","External Power","plugged in","","","");
          #else
            writedisplaytext("((POWER))","External Power","Voltage ok","","","");
          #endif
        }
        usb_status_before = true;
      }
    } else {
      if (usb_status_before) {
        enableOled_now(); // Turn Oled on as indicatior that external power is plugged off
        #if defined(T_BEAM_V1_0) || defined(T_BEAM_V1_2)
          writedisplaytext("((POWER))","","External Power","plugged off","Running on","batteries now");
        #else
          if (shutdown_active && !shutdown_countdown_timer_enable)
            writedisplaytext("((POWER))","","External Power","Voltage too low!","Enabling","shutdown timer");
           else
            writedisplaytext("((POWER))","","External Power","Voltage too low!","Use shutdown timer to","prevent LiPo damage");
        #endif
        freeze_display = true;
        if (shutdown_active && !shutdown_countdown_timer_enable) {
          shutdown_countdown_timer_enable = true;
          shutdown_countdown_timer = millis() + shutdown_delay_time;
        }
        usb_status_before = false;
      } else {
        if (shutdown_active && shutdown_countdown_timer_enable) {
          if (shutdown_countdown_timer - shutdown_delay_time + 15000L >= millis()  /* display for 15s after shutdown was activated */
                ||  millis() > shutdown_countdown_timer - 15000L                   /* display at least 15s before shutdown */
              ) {
            enableOled_now(); // Turn Oled on as indicatior that external power is plugged off
            if ((millis() >= shutdown_countdown_timer)) {
              // send_status_message_about_shutdown_to_rf() writes also to display. Display HALT afterwards
              String msg = "";
              if (send_status_message_about_shutdown_to_rf) {
                #if defined(T_BEAM_V1_0) || defined(T_BEAM_V1_2)
                  msg = "ext. power plugged off";
                #else
                  msg = "V low: " + String(InpVolts, 2) + "";
                #endif
              }
              #if defined(ENABLE_SYSLOG) && defined(ENABLE_WIFI)
                syslog_log(LOG_WARNING, String("shutdown_countdown_timer_enable: halting: timer expired after powerloss or undervoltage. Shutdown.."));
              #endif
              delay(1000);
              do_shutdown(true, msg);
            } else {
              #if defined(T_BEAM_V1_0) || defined(T_BEAM_V1_2)
                writedisplaytext("((POWER))","","External Power","plugged off","Shutdown in",String((shutdown_countdown_timer-millis())/1000) + String("s"));
              #else
                writedisplaytext("((POWER))","","External Power",String("too low:")+String(InpVolts, 2),"Shutdown in",String((shutdown_countdown_timer-millis())/1000) + String("s"));
              #endif
              freeze_display = true;
            }
          } else if (shutdown_countdown_timer - shutdown_delay_time + 15500L >= millis()) {
            freeze_display = false;
          }
        }
      }
    }

  #ifdef KISS_PROTOCOL
    String *TNC2DataFrame = nullptr;
    if (tncToSendQueue) {
      if (xQueueReceive(tncToSendQueue, &TNC2DataFrame, (1 / portTICK_PERIOD_MS)) == pdPASS) {
        #ifdef ENABLE_WIFI
          boolean was_own_position_packet = false;
        #endif
        time_last_frame_via_kiss_received = millis();
        const char *data = TNC2DataFrame->c_str();
        stats_from_kiss++;

        // Frame comes from same call as ours and is a position report?
        if (!strncmp(data, Tcall.c_str(), Tcall.length()) && data[Tcall.length()] == '>') {
          char *p = strchr(data, ':');
          p++;
          if (*p == '!' || *p == '=' || *p == '/' || *p == '@' || *p == '\'' || *p == '`' || *p == '[' || *p == '$') {
            time_last_own_position_via_kiss_received = time_last_frame_via_kiss_received;
            if (!acceptOwnPositionReportsViaKiss)
              goto out; // throw away this frame.
            #ifdef ENABLE_WIFI
              was_own_position_packet = true;
            #endif
            if (!dont_send_own_position_packets) {
              gps_state_before_autochange = gps_state;
              if (gps_allow_sleep_while_kiss && gps_state) {
                do_poweroff_gps();
                gps_state = false;
              }
              dont_send_own_position_packets = true;
              // TODO: there are also tcp kiss devices. Instead of 'kiss_client_came_via_bluetooth', we should mark it in a session struct where we can iterate through
#ifdef ENABLE_BLUETOOTH
              if (serial_bt_client_is_connected)
                kiss_client_came_via_bluetooth = true;
#endif
            }
          } else if (*p == ':') {
            time_last_own_text_message_via_kiss_received = millis();
          }

        } else {
            if (lora_digipeating_mode > 1) {
              const char *p = strchr(data, '>');
              const char *q = strchr(data, ',');
              const char *r = strchr(data, '*');
              const char *header_end = strchr(data, ':');
              // packet (sender not our call) came via kiss and we gate it to lora? If we are a configured as a digipeater,
              // and our gated packet will be repeated, then we have avoid repeating it again, by adding our call to the digi path.
              // Example: DL9SAU>APRS,WIDE1-1,WIDE2-1. If we gate it and a fill-in-digi hears it, if becomes DL9SAU>APRS,DL1AAA,WIDE1*,WIDE2-1.
              // If we are a wide-digi, we see that packet and don't find our callsign in the path, we would repeat it.
              // => We should add our callsign to path, with the following exceptions:
              //   1. if sender is our call (already checked above);
              //   2. if we are not a digipeater and user has a software like aprsdroid and sends packets with another ssid (-> even we are not in the
              // path, there's no risk, we will not repeat it - as said, we are not configured as a repeater).
              //   3. DEST-call-addressing with one hop ("DST-1")
              //   4. In case of WIDE1 or WIDE2-1 (== with one hop left).
              // We assure bufffer has enough room for our call ",CALL*"
              if (q > header_end) q = 0;
              if (r > header_end) r = 0;
              if (p && header_end > p) {
                boolean add_our_call = true;
                // Has the path only one hop or less left (WIDE1-1 or WIDEn or WiDEn* or WIDEn*,WIDEn-1 or WIDEn-1)?
                // Also check for digicall,WIDE1-1;  digicall*,WIDE1-1 is ok.   Pos ',' before wide is != q (q is first ',' in ..>APRS,DIGICALL,WIDE2-1)
                char *wide_hop = strstr(data, ",WIDE");
                if (wide_hop && wide_hop < header_end && wide_hop != q && *(wide_hop-1) != '*')
                  add_our_call = false;
                if (add_our_call) {
                  for (;;) {
                    if (!wide_hop || wide_hop > header_end) {
                      wide_hop = 0;
                      break;
                    }
                    // pos 5 is number, pos 6 is '-', pos 7 are left digis
                    if (wide_hop[6] == '-' && wide_hop[7] != '1')
                      break;
                    wide_hop = strstr(wide_hop+8, ",WIDE");
                  }
                }
                if (!wide_hop)
                  add_our_call = false;
                else {
                  // Is our callsign in the path? -> Fine. But only if repeated-bit is set.
                  char *our_call = strstr(p+1, Tcall.c_str());
                  if (our_call && our_call < header_end && *(our_call-1) == ',' && our_call[Tcall.length()] == '*')
                    add_our_call = false;
                }

                if (add_our_call) {
                  char *s = add_element_to_path(data, Tcall.c_str());
                  if (s) data = s;
                }
              }
            }
        }

        if (usb_serial_data_type & 2) {
          Serial.println(data);
          stats_to_serial++;
        }

#if defined(ENABLE_WIFI)
        if (!was_own_position_packet || tx_own_beacon_from_this_device_or_fromKiss__to_aprsis) {
          // No word "NOGATE" or "RFONLY" in header? -> may be sent to aprs-is. This is a quick pre-check;
          // send_to_aprsis() will check in detail
          char *header_end = strchr(data, ':');
          char *q;
          if (!(q = strstr(data, ",NOGATE")) || q > header_end) {
            if (!(q = strstr(data, ",RFONLY")) || q > header_end) {
              if (!(q = strstr(data, ",TCPIP")) || q > header_end) {
                if (!(q = strstr(data, ",TCPXX")) || q > header_end) {
                  send_to_aprsis(*TNC2DataFrame);
                }
              }
           }
          }
        }
#endif
        if (lora_tx_enabled) {
          enableOled_now(); // enable OLED
          writedisplaytext("((KISSTX))","",String(data),"","","");
          int do_delay = 0;
          if (tx_own_beacon_from_this_device_or_fromKiss__to_frequencies % 2) {
            loraSend(txPower, lora_freq, lora_speed, 0, String(data));  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
            stats_tx_freq_main++;
            do_delay = 600000 / lora_speed;
          } if (((tx_own_beacon_from_this_device_or_fromKiss__to_frequencies > 1 && lora_digipeating_mode > 1) || tx_own_beacon_from_this_device_or_fromKiss__to_frequencies == 5) && lora_freq_cross_digi > 1.0 && lora_freq_cross_digi != lora_freq) {
            loraSend(txPower_cross_digi, lora_freq_cross_digi, lora_speed_cross_digi, 0, String(data));  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
            stats_tx_freq_secondary++;
            if (!do_delay || lora_speed_cross_digi < lora_speed)
              do_delay = 600000 / lora_speed_cross_digi;
          }
          if (do_delay)
            delay(do_delay);
        }
out:
        delete TNC2DataFrame;
      }
    }
  #endif // KISS_PROTOCOL

  esp_task_wdt_reset();
  // lora chip is in mode RX
#ifdef HAS_SX126X
  // We need the sema lock later for non-blocking receive
  if (flag_lora_packet_available) {
#else
  static uint32_t last_receive_check = 0L;
  if (last_receive_check + 300 < millis()) {
    last_receive_check = millis();
#endif
    sema_lock_or_unlock__lora(1);
    esp_task_wdt_reset();
    #ifdef HAS_SX127X
      //if (rf95.waitAvailableTimeout(10)) {
      if (rf95.available()) {
        flag_lora_packet_available = true;
        esp_task_wdt_reset();
        // keep sema up
      } else {
        flag_lora_packet_available = false;
        sema_lock_or_unlock__lora(0);
      }
    #elif HAS_SX126X
      // no else. Elegant: radio.setDio1Action() sets flag_lora_packet_available via signal_new_packet_received() if a packet was received.
      // keep sema up
    #endif
  }
  if (flag_lora_packet_available) {
    // we still take the lock
    #ifdef ENABLE_LED_SIGNALING
      #ifdef T_BEAM_V1_0
        axp.setChgLEDMode(AXP20X_LED_LOW_LEVEL);
      #elif T_BEAM_V1_2
        axp.setChargingLedMode(XPOWERS_CHG_LED_BLINK_1HZ);
      #endif
    #endif
    #ifdef BUZZER
      int melody[] = {300, 50, 500, 100};
      buzzer(melody, sizeof(melody)/sizeof(int));
    #endif

    // we need to read the received packt, even if rx is set to disable. else rf95.waitAvailableTimeout() will always show, data is available
    //byte array
    #ifdef HAS_SX127X
      byte  lora_RXBUFF[LORA_MAX_MESSAGE_LEN];      //buffer for packet
      uint8_t loraReceivedLength = sizeof(lora_RXBUFF); // (implicit ) reset max length before receiving!
      boolean lora_rx_data_available = rf95.recvAPRS(lora_RXBUFF, &loraReceivedLength);
      int lastSNR = bg_rf95snr_to_snr(rf95.lastSNR());
      int lastRssi = bg_rf95rssi_to_rssi(rf95.lastRssi(), lastSNR);
    #elif HAS_SX126X
      String str_lora_RXBUFF;
      const char *lora_RXBUFF = 0;
      boolean lora_rx_data_available = false;
      size_t loraReceivedLength = 0;
      int state = radio.readData(str_lora_RXBUFF);
      if (state == RADIOLIB_ERR_NONE) {
        loraReceivedLength = str_lora_RXBUFF.length();
        // SX126X packets start with the PHY header containing our well-known address
        if (loraReceivedLength > 4 && str_lora_RXBUFF.startsWith("\x3c\xff\x01")) {
          lora_rx_data_available = true;
          lora_RXBUFF = str_lora_RXBUFF.c_str() +3;
          loraReceivedLength -=3;
        } // else: leave loraReceivedLength 0 and lora_rx_data_available false
      }
      int lastSNR = radio.getSNR();
      int lastRssi = radio.getRSSI();
      float freqError = radio.getFrequencyError();
    #endif
    flag_lora_packet_available = false;

    // release lock here. We read the data from the lora chip. And we may call later loraSend (which should not be blocked by ourself)
    sema_lock_or_unlock__lora(0);

    const char *rssi_for_path = encode_snr_rssi_in_path(lastSNR, lastRssi);

    // always needed (even if rx is disabled)
    if (lora_freq_rx_curr == lora_freq) {
      time_last_lora_frame_received_on_main_freq = millis();
      lora_packets_received_in_timeslot_on_main_freq++;
    } else {
      lora_packets_received_in_timeslot_on_secondary_freq++;
    }

    if (lora_rx_enabled && lora_rx_data_available) {
        String loraReceivedFrameString;               //data on buff is copied to this string. raw
        #if defined(ENABLE_SYSLOG)
          String loraReceivedFrameString_for_syslog;    //data on buff is copied to this string. Non-printable characters are shown as <0xnn>. Even valid EOL \r. Syslog is for analyzing.
        #endif
        String loraReceivedFrameString_for_weblist;   //data on buff is copied to this string. Bad characters like \n and \0 are replaced by ' ' (also valid \r - aprs does not use two line messages ;); \r, \n or \0 at the end of the string are removed.
        char *s = 0;
        for (int i=0; i < loraReceivedLength; i++) {
          loraReceivedFrameString += (char) lora_RXBUFF[i];
          #if defined(ENABLE_WIFI)   // || defined(ENABLE_SYSLOG)
            if (lora_RXBUFF[i] >= 0x20) {
              #if defined(ENABLE_SYSLOG)
                loraReceivedFrameString_for_syslog += (char) lora_RXBUFF[i];
              #endif
              loraReceivedFrameString_for_weblist += (char) lora_RXBUFF[i];
            } else {
              #if defined(ENABLE_SYSLOG)
                // In real world, we saw a packet with message text that ended with "\0". This confused the String function, esp. when logging
                char buf[7]; // room for "<0x01>" + \0 == 7
                sprintf(buf, "<0x%2.2x>", (unsigned char ) lora_RXBUFF[i]);
                loraReceivedFrameString_for_syslog += String(buf);
              #endif
              if (lora_RXBUFF[i] == '\r' || lora_RXBUFF[i] == '\n' || lora_RXBUFF[i] == '\t' || lora_RXBUFF[i] == '\0') {
                // replace \r, \n and \0 by ' '. aprs-unsupported multi-line-messages will become a one-liner. At the end of the stringm we'll to .trim() (remove trailing blanks)
                loraReceivedFrameString_for_weblist += String(" ");
              } /* else: skip */
            }
          #endif
        }

        loraReceivedFrameString_for_weblist.trim();

        const char *received_frame = loraReceivedFrameString.c_str();

        char *header_end = strchr(received_frame, ':');
        // valid packet?
        if (!header_end || !packet_is_valid(received_frame)) {
          goto invalid_packet;
        }

        int blacklisted = is_call_blacklisted(received_frame);
        // don't even automaticaly adapt CR for spammers
        if (blacklisted) {
          goto call_invalid_or_blacklisted;
        }

        if (lora_freq_rx_curr == lora_freq) {
          stats_rx_freq_main++;
        } else {
          stats_rx_freq_secondary++;
        }

        char *digipeatedflag = strchr(received_frame, '*');
        if (digipeatedflag && digipeatedflag > header_end)
          digipeatedflag = 0;

        String answer_message = handle_aprs_messsage_addressed_to_us(received_frame);
        boolean its_an_aprs_message_for_us = !answer_message.isEmpty();
        if (answer_message == "M") {
          // It was a message for us. It contained an answer message we have to send;
          // or a Placeholder "M" if no ack message is required -- in this case, clear the message
          answer_message = "";
        }

        uint8_t our_packet = 0; // 1: from us. 2: digipeated by us

        // not our own digipeated call?
        if (loraReceivedFrameString.startsWith(Tcall + '>'))
          our_packet = 1;
        if (((s = strstr(received_frame, (',' + Tcall + '*').c_str())) || (s = strstr(received_frame, (',' + Tcall + ',').c_str()))) && s < header_end) {
          // in path: exact call match and digipeated flag present and we have repeated it (pos behind start of our call?
          if (digipeatedflag && digipeatedflag > s)
            our_packet |= 2;
        }

        // CR adaption: because only for SF12 different CR levels have been defined, we unfortunately cannot deal with SF < 12.
        // In most cases, only useful for normal users, not for WIDE1 or WIDE2 digis. But there may exist good reasons; thus we don't enforce.
        if (lora_automatic_cr_adaption && lora_speed <= 300L) {
          // not our own digipeated call?
          if (! (our_packet & 1)) {
            lora_automaic_cr_adoption_rf_transmissions_heard_in_timeslot++;
            // was digipeated? -> there was another rf transmission
            if (digipeatedflag && !(our_packet & 2))
              lora_automaic_cr_adoption_rf_transmissions_heard_in_timeslot++;
          }
        }

        char *q;

        // User sends ",Q" in path? He likes at the first digi to add snr/rssi to path
        uint8_t user_demands_trace = ( ((q = strstr(received_frame, ",Q,")) || (q = strstr(received_frame, ",Q:"))) && q < header_end) ? 1 : 0;
        // User sends ",QQ"' in path? He likes all digis to add snr/rssi to path
        if (((q = strstr(received_frame, ",QQ,")) || (q = strstr(received_frame, ",QQ:"))) && q < header_end)
          user_demands_trace = 2;


        boolean do_not_gate = false;  // not to aprs-is and not cross-digipeat
        boolean do_not_repeat_to_secondary_freq = false;
        boolean do_not_repeat_on_main_freq = false;

        if (its_an_aprs_message_for_us) {
          do_not_gate = true;
          do_not_repeat_on_main_freq = true;
          do_not_repeat_to_secondary_freq = true;
        } else if (our_packet ||
              (header_end && header_end[1] == '}') ||
              ((q = strstr(received_frame, ",TCPIP")) && (q[6] == '*' || q[6] == ',' || q[6] == ':') && q < header_end) ||
              ((q = strstr(received_frame, ",TCPXXX")) && (q[6] == '*' || q[6] == ',' || q[6] == ':') && q < header_end) ) {
          // 3rd party traffic.
          // For main and secondary frequency, it's a filter for avoiding unnecessary traffic.
          // If we gate 3rd-party-traffic, we'd have to look there in the path for TCPIP, TCPXX, RFONLY, ..;
          // this is now implemented in send_to_aprsis(), where it's checked in detail if it already was there.
          do_not_repeat_on_main_freq = true;
          do_not_repeat_to_secondary_freq = true;
        } else if (header_end && header_end[1] == 'T') {
          // this is actually a filter to prevent telemetry flood on main frequency. You can disable this and recompile, if you really need this
          do_not_repeat_on_main_freq = true;
#ifdef notdef // for testing, these checks are disabled here, because send_to_aprsis() now should handle these cases properly
        } else if (((q = strstr(received_frame, ",RFONLY")) && (q[7] == '*' || q[7] != ',' || q[7] == ':')) && q < header_end) {
          do_not_gate = true;
        } else if (((q = strstr(received_frame, ",NOGATE")) && (q[7] == '*' || q[7] != ',' || q[7] == ':')) && q < header_end) {
          do_not_gate = true;
          do_not_repeat_to_secondary_freq = true;
#else
        } else if (((q = strstr(received_frame, ",NOGATE")) && (q[7] == '*' || q[7] != ',' || q[7] == ':')) && q < header_end) {
          do_not_repeat_to_secondary_freq = true;
#endif
        } else if (((q = strstr(received_frame, ",GATE")) && (q[5] == '*' || q[5] != ',' || q[5] == ':')) && q < header_end) {
          do_not_repeat_to_secondary_freq = true; // no ping pong to secondary freq
        }

     #if defined(ENABLE_WIFI)
        if (aprsis_enabled && !do_not_gate) {
          // No word "NOGATE" or "RFONLY" or "TCPIP" or "TCPXX" in header and not third_party-traffic=? -> may be sent to aprs-is
          s = 0;
          if (((lora_add_snr_rssi_to_path & FLAG_ADD_SNR_RSSI_FOR_APRSIS) || user_demands_trace > 1) ||
              (!digipeatedflag && ((lora_add_snr_rssi_to_path & FLAG_ADD_SNR_RSSI_FOR_APRSIS__ONLY_IF_HEARD_DIRECT) || user_demands_trace == 1)) )
            s = append_element_to_path(received_frame, rssi_for_path);
          send_to_aprsis(s ? String(s) : loraReceivedFrameString);
        }
      #endif

        // Third party traffic? - LH List is a LastPositionDistanceList, not a heard list. We'll ignore third-party-encoded packets.
        if (!our_packet && !(header_end[1] && header_end[1] == '}')) {
          struct ax25_frame* frame = tnc_format_to_ax25_frame(received_frame);
          if (frame) {
            fill_lh(String(frame->src.addr), digipeatedflag, frame->data);
          }
        }

    #ifdef SHOW_RX_PACKET                                                 // only show RX packets when activitated in config
        if (!its_an_aprs_message_for_us) {
           // ^ don't disturb the important display of the message content
          enableOled(); // enable OLED
          writedisplaytext("  ((RX))", String("#") +  String(lora_freq_rx_curr == lora_freq ? (String(stats_rx_freq_main) + " M") : (String(stats_rx_freq_secondary) + " S")) , loraReceivedFrameString, "", "", "");
          time_to_refresh = millis() + showRXTime;
        }

        // List of received packets, for display
        for (int ii=2; ii > 0; ii--) {
          RX_RAW_PACKET_LIST[ii].packet = RX_RAW_PACKET_LIST[ii-1].packet;
          RX_RAW_PACKET_LIST[ii].QRG = RX_RAW_PACKET_LIST[ii-1].QRG;
        }
        RX_RAW_PACKET_LIST[0].packet = String(loraReceivedFrameString);
        RX_RAW_PACKET_LIST[0].QRG = lora_freq_rx_curr == lora_freq ? 'M' : 'S';

        #ifdef ENABLE_WIFI
          sendToWebList(loraReceivedFrameString_for_weblist, lastRssi, lastSNR, lora_freq_rx_curr == lora_freq ? 'M' : 'S');
        #endif
    #endif
    #if defined(ENABLE_SYSLOG) && defined(ENABLE_WIFI) // unfortunately, on this plattform we only have IP if we have WIFI
        syslog_log(LOG_INFO, String("LoRa-RX: '") + loraReceivedFrameString_for_syslog + "', RSSI:" + String(lastRssi) + ", SNR: " + String(lastSNR));
    #endif
    #ifdef KISS_PROTOCOL
        s = 0;
        if (((lora_add_snr_rssi_to_path & FLAG_ADD_SNR_RSSI_FOR_KISS) || user_demands_trace > 1) ||
            (!digipeatedflag && ((lora_add_snr_rssi_to_path & FLAG_ADD_SNR_RSSI_FOR_KISS__ONLY_IF_HEARD_DIRECT) || user_demands_trace == 1)) )

          s = kiss_add_snr_rssi_to_path_at_position_without_digippeated_flag ? append_element_to_path(received_frame, rssi_for_path) : add_element_to_path(received_frame, rssi_for_path);
        sendToTNC(s ? String(s) : loraReceivedFrameString);
    #endif
        if (usb_serial_data_type & 2) {
          s = 0;
          if (((lora_add_snr_rssi_to_path & FLAG_ADD_SNR_RSSI_FOR_KISS) || user_demands_trace > 1) ||
              (!digipeatedflag && ((lora_add_snr_rssi_to_path & FLAG_ADD_SNR_RSSI_FOR_KISS__ONLY_IF_HEARD_DIRECT) || user_demands_trace == 1)) )


            s = kiss_add_snr_rssi_to_path_at_position_without_digippeated_flag ? append_element_to_path(received_frame, rssi_for_path) : add_element_to_path(received_frame, rssi_for_path);
          String s_tmp = s ? String(s) : String(loraReceivedFrameString);
          s_tmp.trim();
          Serial.println(s_tmp);
          stats_to_serial++;
        }

        // Are we configured as lora digi? Are we listening on the main frequency?
        if (lora_tx_enabled && lora_digipeating_mode > 0 &&
            (!do_not_repeat_on_main_freq || !do_not_repeat_to_secondary_freq) &&
            lora_freq_rx_curr == lora_freq) {
          uint32_t time_lora_TXBUFF_for_digipeating_was_filled_prev = time_lora_TXBUFF_for_digipeating_was_filled;
          if ( ((lora_add_snr_rssi_to_path & FLAG_ADD_SNR_RSSI_FOR_RF) || user_demands_trace > 1) ||
                (!digipeatedflag && ((lora_add_snr_rssi_to_path & FLAG_ADD_SNR_RSSI_FOR_RF__ONLY_IF_HEARD_DIRECT) || user_demands_trace == 1)) )
            handle_lora_frame_for_lora_digipeating(received_frame, rssi_for_path);
          else
            handle_lora_frame_for_lora_digipeating(received_frame, NULL);
          // new frame in digipeating queue? cross-digi freq enabled and freq set? Send without delay.
          if (!do_not_repeat_to_secondary_freq && *lora_TXBUFF_for_digipeating &&
              lora_cross_digipeating_mode > 0 && lora_freq_cross_digi > 1.0 && lora_freq_cross_digi != lora_freq &&
              time_lora_TXBUFF_for_digipeating_was_filled > time_lora_TXBUFF_for_digipeating_was_filled_prev) {
            stats_digipeated_rf2rf++;
            digipeated_on_cross_freq = true;
            enableOled_now(); // enable OLED
            writedisplaytext("(TX-Xdigi)", String("#") + String(stats_digipeated_rf2rf), String(lora_TXBUFF_for_digipeating), "", "", "");
            // word 'NOGATE' is not part of the header
            loraSend(txPower_cross_digi, lora_freq_cross_digi, lora_speed_cross_digi, 0, String(lora_TXBUFF_for_digipeating));  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
            stats_tx_freq_secondary++;
#ifdef KISS_PROTOCOL
            s = add_element_to_path(lora_TXBUFF_for_digipeating, "GATE");
            sendToTNC(s ? String(s) : lora_TXBUFF_for_digipeating);
#endif
          }
          if (do_not_repeat_on_main_freq) {
            *lora_TXBUFF_for_digipeating = 0;
          }
        }

        if (its_an_aprs_message_for_us && !answer_message.isEmpty()) {
          // Do not alter display, because it should not be interrupted in showing the received message
          if (usb_serial_data_type & 2) {
            Serial.println(answer_message);
            stats_to_serial++;
          }
#ifdef KISS_PROTOCOL
          sendToTNC(answer_message);
#endif
#if defined(ENABLE_WIFI)
          send_to_aprsis(answer_message);
#endif
          loraSend(lora_freq_rx_curr == lora_freq ? txPower : txPower_cross_digi, lora_freq_rx_curr, lora_freq_rx_curr == lora_freq ? lora_speed : lora_speed_cross_digi, 0, answer_message);
           if (lora_freq_rx_curr == lora_freq) {
             stats_tx_freq_main++;
           } else {
             stats_tx_freq_secondary++;
           }
        }

    } else {
        // rx disabled, or packet is still in receiption (-> channel busy). guess blind
        lora_automaic_cr_adoption_rf_transmissions_heard_in_timeslot++;
    }

call_invalid_or_blacklisted:
invalid_packet:
    #ifdef ENABLE_LED_SIGNALING
      #ifdef T_BEAM_V1_0
        axp.setChgLEDMode(AXP20X_LED_OFF);
      #elif T_BEAM_V1_2
        axp.setChargingLedMode(XPOWERS_CHG_LED_OFF);
      #else
        ; // make compiler happy
      #endif
    #else
      ; // make compiler happy
    #endif
  }

  if (lora_rx_enabled && rx_on_frequencies == 3 && lora_digipeating_mode < 2) {
    static uint8_t slot_table[9][10] = {
      { 0, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, // 1:9
      { 0, 1, 1, 1, 1, 0, 1, 1, 1, 1 }, // 2:8
      { 0, 1, 1, 0, 1, 1, 1, 0, 1, 1 }, // 3:7
      { 0, 1, 0, 1, 1, 0, 1, 0, 1, 1 }, // 4:6
      { 0, 1, 0, 1, 0, 1, 0, 1, 0, 1 }, // 5:5
      { 0, 1, 0, 0, 1, 0, 1, 0, 0, 1 }, // 6:4. From here it's inverse to 4:6, but time-slots left-shifted by one, so first position is alwas the main frequency
      { 0, 0, 1, 0, 0, 0, 1, 0, 0, 1 }, // 7:3
      { 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 }, // 8:2
      { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 }  // 9:1
    };
    static uint8_t *curr_slot_table = slot_table[4];
    static uint8_t *p_curr_slot_table = curr_slot_table;
    static uint32_t next_slot = millis() + 20*1000L;
    static uint32_t recompute_rx_freq_window = millis() + 10*60*1000L;
    if (millis() > recompute_rx_freq_window) {
      // recompute_rx_freq_window and align to n.000 seconds
      recompute_rx_freq_window = (millis() / 1000 + 10*60L) * 1000L;

      // reduce high rx counts because if ratio is < 1:10 or > 10:1 we still like to listen on at least one timeslot for packets
      if (lora_packets_received_in_timeslot_on_main_freq > 100) lora_packets_received_in_timeslot_on_main_freq = 100;
      if (lora_packets_received_in_timeslot_on_secondary_freq > 100) lora_packets_received_in_timeslot_on_secondary_freq = 100;
      // avoid division by zero
      uint8_t lora_packets_received_in_timeslot_on_both_freq = lora_packets_received_in_timeslot_on_main_freq + lora_packets_received_in_timeslot_on_secondary_freq;
      uint8_t ratio = (lora_packets_received_in_timeslot_on_both_freq ? (lora_packets_received_in_timeslot_on_main_freq * 10 / lora_packets_received_in_timeslot_on_both_freq) : 5);
      if (ratio > 9) ratio = 9;
      if (ratio < 1) ratio = 1;
      // choose slot table
      curr_slot_table = slot_table[ratio-1];
      p_curr_slot_table = curr_slot_table;
      next_slot = 0;

      lora_packets_received_in_timeslot_on_main_freq = lora_packets_received_in_timeslot_on_main_freq / 5;
      lora_packets_received_in_timeslot_on_secondary_freq = lora_packets_received_in_timeslot_on_secondary_freq / 5;
    }
    if (millis() > next_slot) {
      // next timeslot is in 20s (aligned to n.000s). If we'd do qsy more often (esp. in a 5:5 situation), chances increase that we loose too much packets due to qsy during receiption. 30 may be too long. 15 too short. 20s also alings good to our 10min window with 10 slots (20/60.0 * 10 * 3 aligns exactly to the 10min window)
      next_slot = (millis() / 1000 + 20L) * 1000L;
      // avoid calling rf95.setFrequency() and lora_set_speed() if previos *p_curr_slot_table was the same freq/speed
      if (*p_curr_slot_table != ((p_curr_slot_table > curr_slot_table) ? p_curr_slot_table[-1] : curr_slot_table[9])) {
        lora_freq_rx_curr = (*p_curr_slot_table) ? lora_freq_cross_digi : lora_freq;
        lora_speed_rx_curr = (*p_curr_slot_table) ? lora_speed_cross_digi : lora_speed;
        esp_task_wdt_reset();
        sema_lock_or_unlock__lora(1);
        esp_task_wdt_reset();
        lora_set_speed(lora_speed_rx_curr);
        #ifdef HAS_SX127X
          // Avoid packet in rx queue from secondary qrg being interpreted to come from main qrg
          if (lora_freq_rx_curr == lora_freq) {
            rf95.recvAPRS(0, 0);
            flag_lora_packet_available = false;
          }
          rf95.setFrequency(lora_freq_rx_curr);
        #elif HAS_SX126X
          // Avoid packet in rx queue from secondary qrg being interpreted to come from main qrg
          if (lora_freq_rx_curr == lora_freq && flag_lora_packet_available) {
            String tmp_buf;
            radio.readData(tmp_buf);
            flag_lora_packet_available = false;
          }
          radio.setFrequency(lora_freq_rx_curr);
          radio.startReceive();
        #endif
        sema_lock_or_unlock__lora(0);
      }
      // restart from beginning of current row?
      if ((p_curr_slot_table - curr_slot_table) >= 9)
        p_curr_slot_table = curr_slot_table;
      else
        p_curr_slot_table++;
    }
  }

  if (lora_automatic_cr_adaption && lora_speed <= 300L && millis() > (5*60*1000L) && (time_lora_automaic_cr_adoption_rx_measurement_window + 5*60*1000L) < millis()) {
    // recalculate automatic adapted CR
    // transmissions in 5 min window.
    // Approx seconds between frames: if transmission takes with CR4/5 3s and sleeps 3x so long, we see every 25s a transmission in a 5s window.
    // With CR4/6 we have 300 / (3*4) * 300.0/240.0 = 31s; wih CR4/7 we have 35s, and with 41s.
    // Slowly, step by step, incrase CR after frequency becomes quet
    uint32_t t_diff = millis() - time_last_lora_frame_received_on_main_freq;
    if (t_diff > (300*1000L / (3*4) * 300.0/180.0) && lora_speed <= 210)
      lora_speed = 180;
    else if (t_diff > (300*1000L / (3*4) * 300.0/210.0) && lora_speed <= 240)
      lora_speed = 210;
    else if (t_diff > (300*1000L / (3*4) * 300.0/240.0) && lora_speed <= 300)
      lora_speed = 240;
    else
      lora_speed = 300;
    lora_automaic_cr_adoption_rf_transmissions_heard_in_timeslot /= 2L;
    time_lora_automaic_cr_adoption_rx_measurement_window = millis() - (5*60*1000L/2L);
  }

  boolean display_was_updated = false;
  ulong tmp_t_since_last_sb_tx = millis() - lastPositionTX;
  static boolean reason_course_change = false;  // used as hint for sendpacket. static, because
                                                // we may not be sure (if we are not TXing now but in one of the next rounds)
                                                // when looking at nextTX == 0: was  the reason a course change

  // Send position, if not requested to do not ;) But enter this part if user likes our LA/LON/SPD/CRS to be displayed on his screen ('!gps_allow_sleep_while_kiss' caused 'gps_state false')
  if (!gps_state && (!dont_send_own_position_packets || !(lora_tx_enabled || aprsis_enabled)))
    goto behind_position_tx;

  // refresh speed and hdop
  curr_kmph = (gps.speed.isValid() ? gps.speed.kmph() : 0.0);
  curr_hdop = (gps.hdop.isValid() ?  gps.hdop.hdop() : 99.9);
  curr_sats = gps.satellites.value();

  average_speed[point_avg_speed] = ((curr_kmph < 1.8 || (curr_hdop < 1.5 && curr_sats >= 3 && curr_kmph <= 16.0) || (curr_hdop < 4.0 && curr_sats >= 3 && curr_kmph > 16.0)) ? curr_kmph : average_speed[(point_avg_speed-1) % 5]);   // calculate smart beaconing. Even a not-updated old speed is ok here.
  ++point_avg_speed;
  if (point_avg_speed > 4) {
    point_avg_speed = 0;
  }
  average_speed_final = (average_speed[0]+average_speed[1]+average_speed[2]+average_speed[3]+average_speed[4])/5;

  if ((millis()<sb_max_interval)&&(lastPositionTX == 0)) {
    nextTX = 0;
  } else if (gps.course.isValid() && gps.course.age() < 10000L) {
    //average_course[point_avg_course] = ((curr_kmph > 1.8 && ((curr_hdop < 1.5 && curr_sats >= 3 && curr_kmph <= 16.0) || (curr_hdop < 4.0 && curr_sats >= 3 && curr_kmph > 16.0))) ? gps.course.deg() : average_course[(point_avg_course-1) % ANGLE_AVGS]);   // calculate smart beaconing course
    average_course[point_avg_course] = ((average_speed_final > 1.8 && ((curr_hdop < 1.5 && curr_sats >= 3 && average_speed_final <= 16.0) || (curr_hdop < 4.0 && curr_sats >= 3 && average_speed_final > 16.0))) ? gps.course.deg() : average_course[(point_avg_course-1) % ANGLE_AVGS]);   // calculate smart beaconing course
    ++point_avg_course;

    if (point_avg_course>(ANGLE_AVGS-1)) {
      point_avg_course=0;
      avg_c_y = 0;
      avg_c_x = 0;
      for (int i=0;i<ANGLE_AVGS;i++) {
        avg_c_y += sin(average_course[i]/180*3.1415);
        avg_c_x += cos(average_course[i]/180*3.1415);
      }
      new_course = atan2f(avg_c_y,avg_c_x)*180/3.1415;
      if (new_course < 0) {
        new_course=360+new_course;
      }
      // too much false positives
      // Only in 30s interval and if we did not announce turn in last round.
      // Algo from Kenwood SB Docu. Considered values should be int.
      // algo is: (int ) ((int )sb_angle + 10 * turn_slope / (int ) mph). In km/h, we have (int ) ((int ) ab_angle + 16 *turn_slope / (int ) km/h)
      //if (nextTX && tmp_t_since_last_sb_tx > (sb_turn_time*1000L) && average_speed_final >= sb_min_speed) {
      if (nextTX && tmp_t_since_last_sb_tx > (sb_turn_time*1000L) && average_speed_final >= 1.8) {
        // cave: average_speed_final must not be 0 (division by zero). Becuse sb_min_speed may be configured as zero, check above is not enough
        int int_average_speed_final = (int ) average_speed_final;
        if (int_average_speed_final < 1) int_average_speed_final = 1;
        int sb_turn_threshold=min(120, (int ) ((int ) sb_angle + 16 * sb_turn_slope / int_average_speed_final));
        if ((old_course < sb_turn_threshold) && (new_course > (360-sb_turn_threshold))) {
          if (abs(new_course-old_course-360)>=sb_turn_threshold) {
            nextTX = 0;
          }
        } else {
          if ((old_course > (360-sb_turn_threshold)) && (new_course < sb_turn_threshold)) {
            if (abs(new_course-old_course+360)>=sb_turn_threshold) {
              nextTX = 0;
            }
          } else {
            if (abs(new_course-old_course)>=sb_turn_threshold) {
              nextTX = 0;
            }
          }
        }
        if (nextTX == 0)
          reason_course_change = true;
      }
      old_course = new_course;
    }
  }


  // No course change (indicator nextTX==0)? Recompute nextTX
  if (nextTX > sb_min_interval && gps.speed.isValid() && gps.speed.age() < 10000) {
#ifdef SB_ALGO_KENWOOD
    if (average_speed_final >= sb_max_speed) {
      nextTX = sb_min_interval;
    } else if (average_speed_final > sb_min_speed) {
      ulong newNextTX = (float ) sb_min_interval * sb_max_speed / average_speed_final;
      // implicit: if (newNextTX > sb_max_interval) newNextTX = sb_max_interval. nexTX is alway < sb_max_interval; we don't need to assure this again here
      // If we increased speed, we have a shorter nextTX. If we decreased, keep nextTX unchanged,
      // because we may have driven at high speed for most of the time until short of the end of the interval
      if (newNextTX < nextTX) {
        nextTX = newNextTX;
      }
    } // else: average_speed_final is <= sb_min_speed -> keep nextTX
#else
    // dl9sau: imho, too affine at high speed level
    //nextTX = (sb_max_interval-sb_min_interval)/(sb_max_speed-sb_min_speed)*(sb_max_speed-average_speed_final)+sb_min_interval;
    // next computation could become negative (if we are faster than sb_max_speed). fixes: either work with temporary signed long variable, or test if sb_max_speed < average_speed_final.
    nextTX = ((sb_max_speed > average_speed_final) ? ((sb_max_interval-sb_min_interval)/(sb_max_speed-sb_min_speed)*(sb_max_speed-average_speed_final)+sb_min_interval) : sb_min_interval);
    //if (nextTX < sb_min_interval) {nextTX=sb_min_interval;}   // already assured (  (sb_max_speed <= average_speed_final) -> nextTX=sb_min_interval)
    if (nextTX > sb_max_interval) {nextTX=sb_max_interval;}
    // now, nextTX is <= sb_max_interval
#endif
  }

  // If we just booted and gps is on but we still have no position, wait up to 10 minutes with entering smart-beaconing code,
  // for preventing our stored preset-position to be sent.
  if (gps_state && no_gps_position_since_boot && millis() < 10*60*1000L)
    goto behind_position_tx;

  // rate limit to 20s in SF12 CR4/5 aka lora_speed 300; 5s in lora_speed 1200 (SF9 CR4/7). -> 1200/lora_speed*5 seconds == 6000000 / lora_speed ms

  if (!fixed_beacon_enabled && !dont_send_own_position_packets && (lora_tx_enabled || aprsis_enabled) &&
      tmp_t_since_last_sb_tx >= (6000000L / lora_speed) && (lastPositionTX + nextTX) < millis()) {
    if (gps_isValid) {
      //enableOled(); // enable OLED
      // we need to send, display it and cancel skip through pages
      if (!freeze_display) {
        enableOled_now(); // enable OLED
        //writedisplaytext(" ((TX))","","LAT: "+LatShownP,"LON: "+LongShownP,"SPD: "+String(gps.speed.kmph(),1)+"  CRS: "+String(gps.course.deg(),1),getSatAndBatInfo());
        fillDisplayLine1(3);
        fillDisplayLine2();
        fillDisplayLines3to5(1);
        writedisplaytext("  ((TX))",String("#") + String(stats_tx_beacon+1),OledLine2,OledLine3,OledLine4,OledLine5);
      }
      sendpacket(SP_POS_GPS | (reason_course_change ? SP_ENFORCE_COURSE : 0 ));
      reason_course_change = false;
      // for fixed beacon (if we loose gps fix, we'll send our last position in fix_beacon_interval)
      // We just transmitted. We transmitted due to turn (nextTX == 1)? Also Don't TX again in next round, sendpacket() adjustet nextTX
    } else {
      if (millis() > time_to_refresh){
        displayInvalidGPS();
      }
    }
    display_was_updated = true;
  }

behind_position_tx:
  if (!display_was_updated) {
    if (!freeze_display) {
      if (millis() > time_to_refresh){
        if (gps_isValid) {
          OledHdr = Tcall;
          fillDisplayLine1(4);
          fillDisplayLine2();
          fillDisplayLines3to5(0);
          writedisplaytext(OledHdr,OledLine1,OledLine2,OledLine3,OledLine4,OledLine5);
        } else {
            displayInvalidGPS();
        }
      } else {
        // refresh  time -> check if necessary
        //fillDisplayLine1(5);
      }
    }
  }


  #if defined(ENABLE_TNC_SELF_TELEMETRY)
    if (enable_tel && nextTelemetryFrame < millis()){
      // Schedule the next telemetry frame
      nextTelemetryFrame = millis() + (tel_interval * 1000);
      sendTelemetryFrame();
    }
  #endif
  #ifdef KISS_PROTOCOL
    #ifdef KISS_DEBUG
      static auto last_debug_send_time = millis();
      if (millis() - last_debug_send_time > 1000*5) {
        last_debug_send_time = millis();
        String debug_message = "";
        #if defined(T_BEAM_V1_0) || defined(T_BEAM_V1_2)
          debug_message += "Bat V: " + String(axp.getBattVoltage());
          debug_message += ", ";
          #ifdef T_BEAM_V1_0
            debug_message += "Bat IN A: " + String(axp.getBattChargeCurrent());
            debug_message += ", ";
            debug_message += "Bat OUT A: " + String(axp.getBattDischargeCurrent());
            debug_message += ", ";
          #endif
          #ifdef T_BEAM_V1_0
            debug_message += "USB Plugged: " + String(axp.isVBUSPlug());
          #elif T_BEAM_V1_2
            debug_message += "USB Plugged: " + String(axp.isVbusInsertOnSource());
          #endif
          debug_message += ", ";
          debug_message += "USB V: " + String(axp.getVbusVoltage());
          debug_message += ", ";
          #ifdef T_BEAM_V1_0
            debug_message += "USB A: " + String(axp.getVbusCurrent());
            debug_message += ", ";
          #endif
          #ifdef T_BEAM_V1_0
            debug_message += "Temp C: " + String(axp.getTemp());
          #elif T_BEAM_V1_2
            debug_message += "Temp C: " + String(axp.getTemperature());
          #endif
        #else
          debug_message += "USB V: " + String(InpVolts);
        #endif

        Serial.print(encapsulateKISS(debug_message, CMD_HARDWARE));
        #ifdef ENABLE_BLUETOOTH
          if (enable_luetooth)
            SerialBT.print(encapsulateKISS(debug_message, CMD_HARDWARE));
        #endif
      }
    #endif
  #endif


  // Data for digipeating in queue?
  if (*lora_TXBUFF_for_digipeating) {
    boolean clear_lora_TXBUFF_for_digipeating = false;
    if (!lora_digipeating_mode || lora_cross_digipeating_mode > 1 || !lora_tx_enabled || !lora_rx_enabled) {
      clear_lora_TXBUFF_for_digipeating = true;
    } else {
      // Only digipeat if not too old (20s)
      if ((time_lora_TXBUFF_for_digipeating_was_filled + 20*1000L) < millis()) {
        clear_lora_TXBUFF_for_digipeating = true;
      }
    }
    if (!clear_lora_TXBUFF_for_digipeating) {
      // 3s grace time (plus up to 250ms random) for digipeating. 10s if we are a fill-in-digi (WIDE1);
      // -> If we are WIDE1 and another digipeater repeated it, we'll have deleted it from queue
      if (time_lora_TXBUFF_for_digipeating_was_filled + (lora_digipeating_mode == 2 ? 10 : 3 )*1000L > millis()) {
        if (!digipeated_on_cross_freq)
           stats_digipeated_rf2rf++;
        enableOled_now(); // enable OLED
        writedisplaytext("((TXdigi))", String("#") + String(stats_digipeated_rf2rf), String(lora_TXBUFF_for_digipeating), "", "", "");
        // if SF12: we degipeat in fastest mode CR4/5. -> if lora_speed < 300 tx in lora_speed_300.
        loraSend(txPower, lora_freq, (lora_speed < 300) ? 300 : lora_speed, 0, String(lora_TXBUFF_for_digipeating));  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
        stats_tx_freq_main++;
#ifdef KISS_PROTOCOL
        sendToTNC(String(lora_TXBUFF_for_digipeating));
#endif
        clear_lora_TXBUFF_for_digipeating = true;
      } // else: keep packet for next round in loop
    }
    if (clear_lora_TXBUFF_for_digipeating) {
      *lora_TXBUFF_for_digipeating = 0;
    }
  }

  handle_usb_serial_input();

#if defined(ENABLE_WIFI)
  if (tx_own_beacon_from_this_device_or_fromKiss__to_aprsis) {
    send_own_beacon_to_aprsis_cached_or_now("");
  }
#endif

  vTaskDelay(1);
}
