// Tracker for LoRA APRS
// from OE1ACM and OE3CJB redesigned by SQ9MDD
// KISS ans Bluetooth by SQ5RWU
// TTGO T-Beam v1.0 only
//
// licensed under CC BY-NC-SA
// Includes
//#include <TTGO_T-Beam_LoRa_APRS_config.h> // to config user parameters
#include <Arduino.h>
#include <SPI.h>
#include <BG_RF95.h>         // library from OE1ACM
#include <math.h>
#include <driver/adc.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SSD1306.h>
#include <splash.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>
#include <axp20x.h>
#include <esp_task_wdt.h>
#include "taskGPS.h"
#include "version.h"
#include "preference_storage.h"
#include "syslog_log.h"

#ifdef KISS_PROTOCOL
  #include "taskTNC.h"
#endif
#ifdef ENABLE_WIFI
  #include "taskWebServer.h"
  String wifi_info;                // saving wifi info (CLI|AP|dis) for Oled
#endif

// oled address
#define SSD1306_ADDRESS 0x3C

// SPI config
#define SPI_sck 5
#define SPI_miso 19
#define SPI_mosi 27
#define SPI_ss 18

// IO config
#ifdef T_BEAM_V1_0
  #define I2C_SDA 21
  #define I2C_SCL 22    
  #define BUTTON  38                //pin number for Button on TTGO T-Beam
  #define BUZZER 15                 // enter your buzzer pin gpio
  const byte TXLED  = 4;            //pin number for LED on TX Tracker
#elif T_BEAM_V0_7
  #define I2C_SDA 21
  #define I2C_SCL 22
  #define BUTTON  39                //pin number for Button on TTGO T-Beam
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
  #define BUZZER 13                 // enter your buzzer pin gpio
  const byte TXLED  = 4;            //pin number for LED on TX Tracker
#elif LORA32_2
  #define I2C_SDA 21
  #define I2C_SCL 22
  #define BUTTON 2                  //pin number for BUTTO
  #define BUZZER 13                 // enter your buzzer pin gpio
  const byte TXLED  = 4;            //pin number for LED on TX Tracker
#elif LORA32_1
  #define I2C_SDA 21
  #define I2C_SCL 22 
  #define BUTTON 2                  //pin number for BUTTO
  #define BUZZER 13                 // enter your buzzer pin gpio
  const byte TXLED  = 4;            //pin number for LED on TX Tracker
#elif HELTEC_V1
  #define I2C_SDA 4
  #define I2C_SCL 15 
  #define BUTTON 2                  //pin number for BUTTO
  #define BUZZER 13                 // enter your buzzer pin gpio
  const byte TXLED  = 4;            //pin number for LED on TX Tracker
#elif HELTEC_V2
  #define I2C_SDA 4
  #define I2C_SCL 15    
  #define BUTTON 2                  //pin number for BUTTO
  #define BUZZER 13                 // enter your buzzer pin gpio
  const byte TXLED  = 4;            //pin number for LED on TX Tracker
#endif

// Variables for LoRa settings
#ifdef LORA_SPEED_1200
  ulong lora_speed = 1200;
#else
  ulong lora_speed = 300;
#endif

#ifdef	TXFREQ
  double lora_freq = TXFREQ;
#else
  double lora_freq = 433.775;
#endif

ulong lora_speed_cross_digi = 1200;
double lora_freq_cross_digi = 433.900;

double lora_freq_rx_curr = lora_freq;
ulong lora_speed_rx_curr = lora_speed;

// Variables for WIFI APRS-IS connection. Requires ENABLE_WIFI
#ifdef ENABLE_WIFI
boolean aprsis_enabled = false;
String aprsis_host = "euro.aprs2.net";
uint16_t aprsis_port = 14580;
String aprsis_filter = "";
String aprsis_callsign = "";
String aprsis_password = "-1";
uint8_t aprsis_data_allow_inet_to_rf = 2;  // 0: disable. 1: gate to main qrg. 2: gate to secondary qrg. 3: gate to both frequencies
#endif

// Variables for APRS packaging
String Tcall;                       //your Call Sign for normal position reports
String aprsSymbolTable = APRS_SYMBOL_TABLE;
String aprsSymbol = APRS_SYMBOL;
String relay_path;
String aprsComment = MY_COMMENT;
String aprsLatPreset = LATITUDE_PRESET;
String aprsLonPreset = LONGITUDE_PRESET;
//String LatShownP = aprsLonPreset;
//String LongShownP = aprsLonPreset;
// "P" means original Preset, "u": Preset has been updated to the last valid position (DL3EL). "p" means, invalid gps, last known position used a temporary preset.
String aprsPresetShown = "P";
double lastTxdistance = 0;

#if defined(T_BEAM_V1_0) || defined(T_BEAM_V0_7)
  boolean gps_state = true;
#else
  boolean gps_state = false;
#endif
// as we now collect the gps_data at the beginning of loop(), also the speed ist from there, we should not query gps.speed.kmph directly later on
// the same show be done with course and alti (later)
// after successful data retrieval, gps_isValid becomes true (or turn false, if retrieval fails)
// all by DL3EL
boolean gps_isValid = false;
int gps_speed_kmph = 0;
int gps_speed_kmph_oled = 0;
char gps_time_s[20];// Room for len(01:02:03 04.05.2022) + 1 /* \0 */  -> 20

boolean key_up = true;
boolean t_lock = false;
boolean fixed_beacon_enabled = false;
boolean show_cmt = true;
// Telemetry sequence, current value
int tel_sequence;
// Telemetry path
String tel_path;

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
  int tel_mic = 0; // telemetry as "T#001"
#endif
#if defined(ENABLE_BLUETOOTH) && defined(KISS_PROTOCOL)
  boolean enable_bluetooth = true;
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
#endif
#ifdef ENABLE_OLED
  boolean enabled_oled = true;
#else
  boolean enabled_oled = false;
#endif

// Variables and Constants
String outString="";                      //The new Output String with GPS Conversion RAW

//Oled Display (DL3EL)
// to implement the blinking ticker, we have to save the content of the OLED. Also the preparation of output to OLED is done via these variables
// Lines in "normal" display (no rx or tx or keypress)
String OledHdr = "";      // Callsign
String OledLine1 = "";    // localtime + uptime
String OledLine2 = "";    // WebServer Info (CLI|AP|dis), next beacon (SB|FB), GPS Age (if > 2s)
String OledLine3 = "";    // Position
String OledLine4 = "";    // speed, course, altitude
String OledLine5 = "";    // sat info, batt info
int OLED_refresh = 1000;

#if defined(ENABLE_TNC_SELF_TELEMETRY) && defined(KISS_PROTOCOL)
  time_t nextTelemetryFrame;
#endif


//byte Variables
byte  lora_TXStart;          //start of packet data in TXbuff
byte  lora_TXEnd;            //end of packet data in TXbuff
byte  lora_FTXOK;            //flag, set to 1 if TX OK
byte  lora_TXPacketType;     //type number of packet to send
byte  lora_TXDestination;    //destination address of packet to send
byte  lora_TXSource;         //source address of packet received
byte  lora_FDeviceError;     //flag, set to 1 if RFM98 device error
byte  lora_TXPacketL;        //length of packet to send, includes source, destination and packet type.

unsigned long lastTX = 0L;
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

float average_speed[5] = {0,0,0,0,0}, average_speed_final=0;
float old_course = 0, new_course = 0;
int point_avg_speed = 0, point_avg_course = 0;

ulong nextTX=60000L;                  // preset time period between TX = 60000ms = 60secs = 1min

ulong time_to_refresh = 0;
ulong next_fixed_beacon = 75000L;    // first fixed beacon app. 125s after system start (DL3EL)
ulong fix_beacon_interval = FIX_BEACON_INTERVAL;
ulong showRXTime = SHOW_RX_TIME;
ulong time_delay = 0;
ulong shutdown_delay = 0;
ulong shutdown_delay_time = 10000;
ulong shutdown_countdown_timer = 0;
boolean shutdown_active =true;
boolean shutdown_countdown_timer_enable = false;
boolean shutdown_usb_status_bef = false;
uint32_t reboot_interval = 0L;

// Variables required to Power Save OLED
// With "Display dimmer enabled" it will turn OLED off after some time
// if the checkbox is disabled the display stays OFF
uint oled_timeout = SHOW_OLED_TIME; // OLED Timeout
bool tempOled = true; // Turn ON OLED at first startup
ulong oled_timer;

// Variable to manually send beacon from html page
bool manBeacon = false;

// Variable to show AP settings on OLED
int8_t WIFI_DISABLED = 0;
int8_t WIFI_SEARCHING_FOR_AP = 1;
int8_t WIFI_CONNECTED_TO_AP = 2;
int8_t WIFI_NOT_CONNECTED_TO_AP = 4;
int8_t WIFI_RUNNING_AS_AP = 8;
int8_t wifi_connection_status = WIFI_DISABLED;
int8_t wifi_connection_status_prev = -1;
String infoApName = "";
String infoApPass = "";
String infoIpAddr = "";

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


// may be configured
boolean rate_limit_message_text = true;		// ratelimit adding messate text (-> saves airtime)
boolean lora_automatic_cr_adaption = false;	// automatic CR adaption
						// We'll count users (except own digipeated packets), and if we're alone, CR4/8 doesn't disturb anyone.
						// If we have a high load on the channel, we'll decrease up to CR4/5.
						// You may set this to off if you are a fixed station / repeater / gateway
						// This may become set to true by default, after it proves it behaves good to our network
uint8_t lora_digipeating_mode = 1;		// Digipeating: 0: disabled (recommended if the device should not do repeating decisions, and even more, if you have attached a normal aprs digipeating software via kiss). 1: if own call addressed (recommended for users) 2: act as WIDE1 fill-in digi (recommended for standalone fill-in-digi). 3: act as a simple stupid WIDE2 digi
uint8_t lora_cross_digipeating_mode = 0;	// 0: disable cross freq digipeating. 1: send on both frequencies. 2: send only on cross frequency
#define FLAG_ADD_SNR_RSSI_FOR_RF     1
#define FLAG_ADD_SNR_RSSI_FOR_KISS   2
#define FLAG_ADD_SNR_RSSI_FOR_APRSIS 4
#define FLAG_ADD_SNR_RSSI_FOR_RF__ONLY_IF_HEARD_DIRECT     8
#define FLAG_ADD_SNR_RSSI_FOR_KISS__ONLY_IF_HEARD_DIRECT   16
#define FLAG_ADD_SNR_RSSI_FOR_APRSIS__ONLY_IF_HEARD_DIRECT 32
uint8_t lora_add_snr_rssi_to_path = (FLAG_ADD_SNR_RSSI_FOR_KISS | FLAG_ADD_SNR_RSSI_FOR_APRSIS__ONLY_IF_HEARD_DIRECT);	// Add snr+rssi to path. May become default, after it proves it behaves good to our network
boolean kiss_add_snr_rssi_to_path_at_position_without_digippeated_flag = 1; // Add snr+rssi at last digipeater, without digipeated flag, at last position in path. Set to 1, if you pass data to aprs-is. Set to 0 if you pass data to your favourite digipeater software. We need this hack because our rssi-encoded data should not be interpreted as "(last ==) direct heard station" in the aprs-is net.
int tx_own_beacon_from_this_device_or_fromKiss__to_frequencies = 1;	// TX own beacon generated from this device or our beacon from from-kiss on following frequencies. Only if lora_digipeating_mode > 1 (we are a WIDE1 or WIDE2 digi). 1: main freq. 2: cross_digi_freq. 3: both frequencies
boolean tx_own_beacon_from_this_device_or_fromKiss__to_aprsis = true;	// TX own beacon generated from this device or our beacon from from-kiss to aprs-is.
int rx_on_frequencies = 1;			// RX freq. Only if lora_digipeating_mode < 2 (we are a user) 1: main freq. 2: cross_digi_freq. 3: both frequencies

bool acceptOwnPositionReportsViaKiss = true;		// true: Switches off local beacons as long as a kiss device is sending positions with our local callsign. false: filters out position packets with own callsign coming from kiss (-> do not send to LoRa).
boolean gps_allow_sleep_while_kiss = true;		// user has a kiss device attached via kiss which sends positions with own call, we don't need our gps to be turned on -> We pause sending positions by ourself (neither fixed nor smart beaconing). Except: user has a display attached to this tracker, he'll will be able to see his position because our gps does not go to sleep (-> set this to false). Why sleep? Energy saving
boolean wifi_do_failback_to_mode_AP = true;		// Allow failback to mode AP after once connected successfully connected (after boot) to configured remote AP. Disable for igates, where you don't need your tracker to be a hotspot. You like to enable, if you use your tracker portable and it should automatically be wifi client to your home network, and be AP if you are outside.

#ifdef KISS_PROTOCOL
// do not configure
uint32_t time_last_own_position_via_kiss_received = 0L;	// kiss client sends position report with our call+ssid. Remember when.
uint32_t time_last_frame_via_kiss_received = 0L;	// kiss client sends aprs-text-messages with our call+ssid. Remember when.
boolean kiss_client_came_via_bluetooth = false;
#endif

uint16_t adjust_cpuFreq_to = 80;

// do not configure
boolean dont_send_own_position_packets = false;		// dynamicaly set if kiss device sends position. Maybe there are other usecases (-> kiss-independent)
boolean gps_state_before_autochange = false;		// remember gps state before autochange
uint32_t time_last_lora_frame_received_on_main_freq = 0L;
uint32_t time_last_own_text_message_via_kiss_received = 0L;
uint32_t time_lora_automaic_cr_adoption_rx_measurement_window = 0L;
uint16_t lora_automaic_cr_adoption_rf_transmissions_heard_in_timeslot = 0;
uint16_t lora_packets_received_in_timeslot_on_main_freq = 0;
uint16_t lora_packets_received_in_timeslot_on_secondary_freq = 0;
char lora_TXBUFF_for_digipeating[BG_RF95_MAX_MESSAGE_LEN+1] = "";		// buffer for digipeating
time_t time_lora_TXBUFF_for_digipeating_was_filled = 0L;
boolean sendpacket_was_called_twice = false;
// bits for sendpacket()
#define SP_POS_FIXED 1
#define SP_POS_GPS   2
#define SP_ENFORCE_COURSE 4
uint32_t t_last_smart_beacon_sent = 0L;

String MY_APRS_DEST_IDENTIFYER = "APLOX1";

#ifdef ENABLE_WIFI
  tWebServerCfg webServerCfg;
  String to_aprsis_data = "";
#endif

static const adc_atten_t atten = ADC_ATTEN_DB_6;
static const adc_unit_t unit = ADC_UNIT_1;
#ifdef T_BEAM_V1_0
  AXP20X_Class axp;
#endif


// Singleton instance of the radio driver
BG_RF95 rf95(18, 26);        // TTGO T-Beam has NSS @ Pin 18 and Interrupt IO @ Pin26


char blacklist_calls[256] = "";

// initialize OLED display
#define OLED_RESET 16         // not used
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);


#ifdef IF_SEMAS_WOULD_WORK
xSemaphoreHandle sema_lora_chip;
#else
volatile boolean sema_lora_chip = false;
#endif


// + FUNCTIONS-----------------------------------------------------------+//


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


void prepareAPRSFrame(uint8_t sp_flags){
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

  if (
#if defined(ENABLE_BLUETOOTH) && defined(KISS_PROTOCOL)
      SerialBT.hasClient() ||
#endif
      ((time_last_own_text_message_via_kiss_received + 24*60*60*1000L) > millis())
     )
    outString += "=";
  else
    outString += "!";

  if (!(sp_flags & SP_POS_FIXED) && gps_state && gps_isValid) {
    uint32_t aprs_lat, aprs_lon;
    String helper;
    char helper_base91[] = {"0000\0"};
    double Tlat=52.0000, Tlon=20.0000;
    double Tspeed=0, Tcourse=0;
    int i;
    long Talt;
    static uint8_t cnt = 0;
    boolean time_to_add_alt = false;

    Tlat=gps.location.lat();
    Tlon=gps.location.lng();
    Tcourse=gps.course.deg();
    Tspeed=gps.speed.knots();
    aprs_lat = 900000000 - Tlat * 10000000;
    aprs_lat = aprs_lat / 26 - aprs_lat / 2710 + aprs_lat / 15384615;
    aprs_lon = 900000000 + Tlon * 10000000 / 2;
    aprs_lon = aprs_lon / 26 - aprs_lon / 2710 + aprs_lon / 15384615;

    // altitude_ratio: 0%, 10%, 25%, 50%, 75%, 90%, 100%
    // course change on turn has a higher priority
    boolean altitude_isValid = (gps.altitude.isValid() && gps.altitude.age() < 10000);
    boolean cseSpd_isValid = (gps.speed.isValid() && gps.speed.age() < 10000 && gps.course.isValid() && gps.course.age() < 10000);

    if (altitude_isValid && altitude_ratio > 0) {
      if (altitude_ratio <= 50)
        time_to_add_alt = (cnt % (100 / altitude_ratio) == 0);
      else if (altitude_ratio < 100)
        time_to_add_alt = (cnt % (100 / (100-altitude_ratio)) != 0);
      else
        time_to_add_alt = true;
    }

    // Due to spec, /A...... in message text could be used with compressed and uncompressed positions.
    // But if you encode altitude in compressed position: course/speed (i.e. 090/012) may not be added to message text
    // -> if we need (or wish to have always) course/speed, compression is always for peed, and altitude is part of the message text

    boolean may_send_alt_compressed = true;
    if (always_send_cseSpd_AND_altitude || !altitude_ratio) {
                                           // ^altitude ratio == 0 means 'never altitude'
         // ^always_send_cseSpd_AND_altitude means, we have to go to the compressed-speed-section, regardle of the alt-ratio
         // because altitude could be added to compressed speed, but not: speed to compressed altitude.
         // We go there even for thes special case of altitude_ratio == 100.
      may_send_alt_compressed = false;
    } else if (!time_to_add_alt && altitude_ratio < 100) {
                                 // ^altitude ratio == 100 means 'always altitude', except if always_send_cseSpd_AND_altitude is configured (checked 3 lines aboce)
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

    outString += aprsSymbolTable;
    ax25_base91enc(helper_base91, 4, aprs_lat);
    for (i = 0; i < 4; i++) {
      outString += helper_base91[i];
    }
    ax25_base91enc(helper_base91, 4, aprs_lon);
    for (i = 0; i < 4; i++) {
      outString += helper_base91[i];
    }
    outString += aprsSymbol;


    if (may_send_alt_compressed) {

      if (altitude_isValid) {
        Talt = gps.altitude.feet();
        if (Talt < 0) Talt = 0;
        else if (Talt > 15270967) Talt = 15270967; /* 1.002** (90*91+90-1) */
        ax25_base91enc(helper_base91, 2, (uint32_t) (log1p(Talt) / 0.001998));
        /* ^ math.log1p(1.002-1) */
        outString += helper_base91[0];
        outString += helper_base91[1];
      } else {
        outString += "  ";
      }
      outString += "X";

    } else {

      if (cseSpd_isValid) {
        ax25_base91enc(helper_base91, 1, (uint32_t) Tcourse / 4);
        outString += helper_base91[0];
        if (Tspeed > 1018) Tspeed = 1018; /* 1.08**90 */
        ax25_base91enc(helper_base91, 1, (uint32_t) (log1p(Tspeed) / 0.07696));
        /* ^ math.log1p(1.08-1) */
        outString += helper_base91[0];
      } else {
        outString += "  ";
      }
      outString += "H";

      if (time_to_add_alt) {
        Talt = gps.altitude.feet();
        char buf[7];
        outString += "/A=";
        if (Talt > 999999) Talt=999999;
        else if (Talt < -99999) Talt=-99999;
        sprintf(buf, "%06ld", Talt);
        outString += buf;
      }
    }

    cnt++;

  } else {  //fixed position not compresed
    outString += aprsLatPreset;
    outString += aprsSymbolTable;
    outString += aprsLonPreset;
    outString += aprsSymbol;
  }

  if(show_cmt){
    static uint8_t comments_added = 0;
    static uint32_t time_comment_added = 0L;

    if (!rate_limit_message_text) {
      comments_added = 0;
    } else {
      uint32_t t_offset = (gps_state ? sb_max_interval : fix_beacon_interval);
      // send comment text not under 10min, and at least every hour
      if (t_offset < 600000)
        t_offset = 600000;
      else if (t_offset > 3600000)
        t_offset = 3600000;
      if ((time_comment_added + t_offset) < millis())
        comments_added = 0;
    }
    if ((comments_added++ % 10) == 0) {
      outString += aprsComment;
      time_comment_added = millis();
    }
  }

  if (showBattery && (BattVolts > 1.0 || InpVolts > 1.0)){
    outString += " Batt=";
    outString += String((BattVolts > 1.0 ? BattVolts : InpVolts), 2);
    outString += ("V");
  }

  #ifdef KISS_PROTOCOL
    sendToTNC(outString);
  #endif
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
}

#if defined(ENABLE_WIFI)
void send_to_aprsis(String s)
{
  to_aprsis_data = s;
  return;
}
#endif


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
    if (tx_own_beacon_from_this_device_or_fromKiss__to_frequencies % 2)
      loraSend(txPower, lora_freq, lora_speed, outString);  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
    if (tx_own_beacon_from_this_device_or_fromKiss__to_frequencies > 1 && lora_digipeating_mode > 1 && lora_freq_cross_digi > 1.0 && lora_freq_cross_digi != lora_freq)
      loraSend(txPower_cross_digi, lora_freq_cross_digi, lora_speed_cross_digi, outString);  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
  }
#if defined(ENABLE_WIFI)
  if (tx_own_beacon_from_this_device_or_fromKiss__to_aprsis)
    send_to_aprsis(outString);
#endif
  sendpacket_was_called_twice = true;
}


/**
 * Send message as APRS LoRa packet
 * @param lora_LTXPower
 * @param lora_FREQ
 * @param lora_SPEED
 * @param message
 */
void loraSend(byte lora_LTXPower, float lora_FREQ, ulong lora_SPEED, const String &message) {
  int n;

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
#ifdef IF_SEMAS_WOULD_WORK
  while (xSemaphoreTake(sema_lora_chip, 10) != pdTRUE)
    esp_task_wdt_reset();
#else
  for (n = 0; sema_lora_chip; n++) {
    delay(10);
    if (!(n % 100))
      esp_task_wdt_reset();
  }
  sema_lora_chip = true;
#endif

  randomSeed(millis());

  for (n = 0; n < 30; n++) {
    esp_task_wdt_reset();
    delay(wait_for_signal);
    if (rf95.SignalDetected()) {
      continue;
    }
    delay(100);
    if (!rf95.SignalDetected() && random(256) < 64) {
      break;
    }
  }

  esp_task_wdt_reset();
#ifdef T_BEAM_V1_0
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);                           // switch LoRa chip on
#endif

  //byte array
  byte  lora_TXBUFF[BG_RF95_MAX_MESSAGE_LEN];      //buffer for packet to send
  int messageSize = min(message.length(), sizeof(lora_TXBUFF) - 1);
  message.toCharArray((char*)lora_TXBUFF, messageSize + 1, 0);
  lora_set_speed(lora_SPEED);
  rf95.setFrequency(lora_FREQ);
  rf95.setTxPower(lora_LTXPower);

  #ifdef ENABLE_LED_SIGNALING
    digitalWrite(TXLED, LOW);
  #endif
  lastTX = millis();
  rf95.sendAPRS(lora_TXBUFF, messageSize);
  rf95.waitPacketSent();
  #ifdef ENABLE_LED_SIGNALING
    digitalWrite(TXLED, HIGH);
  #endif
  // cross-digipeating may have altered our RX-frequency. Revert frequency change needed for this transmission.
  if (lora_FREQ != lora_freq_rx_curr) {
    rf95.setFrequency(lora_freq_rx_curr);
    // flush cache. just to be sure, so that no cross-digi-qrg packet comes in the input-buffer of the main qrg.
    // With no buffer / length called, recvAPRS directly calls clearRxBuf()
    rf95.recvAPRS(0, 0);
  }
  if (lora_SPEED != lora_speed_rx_curr)
    lora_set_speed(lora_speed_rx_curr);
#ifdef T_BEAM_V1_0
  // if lora_rx is disabled, but  ONLY if lora_digipeating_mode == 0 AND no SerialBT.hasClient is connected,
  // we can savely go to sleep
  if (! (lora_rx_enabled || lora_digipeating_mode > 0 || SerialBT.hasClient()) )
    axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF);                           // switch LoRa chip off
#endif
  // release lock
#ifdef IF_SEMAS_WOULD_WORK
  xSemaphoreGive(sema_lora_chip);
#else
  sema_lora_chip = false;
#endif

}


void batt_read(){
#ifdef T_BEAM_V1_0
  BattVolts = axp.getBattVoltage()/1000;
  InpVolts = axp.getVbusVoltage()/1000;
#elif T_BEAM_V0_7
  BattVolts = (((float)analogRead(35) / 8192.0) * 2.0 * 3.3 * (1100.0 / 1000.0))+0.41;    // fixed thanks to Luca IU2FRL
  //BattVolts = adc1_get_raw(ADC1_CHANNEL_7)/1000;
#else
  BattVolts = analogRead(35)*7.221/4096;
#endif
}


void writedisplaytext(String HeaderTxt, String Line1, String Line2, String Line3, String Line4, String Line5) {
  batt_read();
#ifdef notdef
  if (InpVolts < 1.0) {
    if (BattVolts < 3.5 && BattVolts > 3.3){
      #ifdef T_BEAM_V1_0
        # ifdef ENABLE_LED_SIGNALING
          axp.setChgLEDMode(AXP20X_LED_BLINK_4HZ);
        #endif
      #endif
    } else if (BattVolts <= 3.3) {
      #ifdef T_BEAM_V1_0
        axp.setChgLEDMode(AXP20X_LED_OFF);
        //axp.shutdown(); <-we need fix this
        axp.shutdown();
      #endif
    }
  }
#endif
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
  if (!enabled_oled){                         // disable oled
    display.dim(true);
  }
  display.display();
  // if oled is continuosly on, refresh every second (DL3EL)
  if (oled_timeout == 0) {
    time_to_refresh = millis() + OLED_refresh;
  } else {
    time_to_refresh = millis() + showRXTime;
  }
  // save last display output for blinker
  OledHdr = HeaderTxt;
  OledLine1 = Line1;
  OledLine2 = Line2;
  OledLine3 = Line3;
  OledLine4 = Line4;
  OledLine5 = Line5;
}

// in refresh_display umbenannt werden
void display_refresh1s() {
  static uint32_t display_next_refresh = 0L;
  static struct tm timeinfo; 

  if (millis() < display_next_refresh)
    return;
  
  display_next_refresh = millis() + OLED_refresh;
// einmal Sekunde soll die Uhrzeit aktualisiert werden
  if (getLocalTime(&timeinfo)) {
    strftime(gps_time_s, sizeof(gps_time_s), "%H:%M:%S", &timeinfo);
  }
  fillDisplayLine1(); //update time & uptime
  
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

}


String getSpeedCourseAlti() {
  // muss noch umgebaut werden, so dass die Daten wie speed im loop() geholt werden (DL3EL)
  // TODO: Web configurable output in speed km/h and height m; or speed mph and height ft, or speed kn (sm/h) and height ft
  String sca = "";
  String dalt = "";

  if (gps_state == true && gps_isValid) {
    int  dalt_int = max(-99999, min(999999, (int ) gps.altitude.meters()));
    dalt = dalt + String(dalt_int) + "m ";
    sca = String(gps_speed_kmph) + "km/h " + String(gps.course.deg(), 1) + "\xF7 " + dalt;
  } else {
    sca = "--km/h --\xF7 --m";
  }
  return sca;
}


String getSatAndBatInfo() {
  String line5;

  if (gps_state == true)
    line5 = "S:" + String(gps.satellites.value()) + "/" + int(gps.hdop.hdop());
  else
    line5 = "S:-/-";
#ifdef T_BEAM_V1_0
  int b_out_c = (int ) axp.getBattDischargeCurrent();
  int b_in_c = (int ) axp.getBattChargeCurrent();
  if (b_out_c == 0 && b_in_c == 0) {
    line5 = line5 + " P:" + String(InpVolts, 2) + "V";
  } else {
#else
    {
#endif
      line5 = line5 + " B:" + String(BattVolts, 2) + "V";
    }
#ifdef T_BEAM_V1_0
    String charge = "";

    if (b_out_c > 0) {
      charge = "-" + String(b_out_c);
    } else {
      if (b_in_c > 0) {
        charge = String(b_in_c);
    } else {
        charge = "-" + String((int ) axp.getVbusCurrent());
      }
    }
    line5 = line5 + "/" + charge + "mA";
#endif
#if defined(ENABLE_BLUETOOTH) && defined(KISS_PROTOCOL)
    if (SerialBT.hasClient()) {
      line5 += " BT";
    }
#endif
    return line5;
}

void fillDisplayLine1() {
//  static String OledLine1s = "";
// OledLine1_time = gps_time_s;
//  OledLine1s = " Up:" + String(millis()/1000/60) + "m";

  static uint32_t old_time = 0L;
  uint32_t t = millis() / 1000;
  char s_uptime[9];  // room for 49d17:02 + \0 -> 9 bytes

  // > 49d 17h 2min? millis-overflow -> mark it
  if (t < old_time)
    old_time = ~0;

  if (old_time == ~0) {
    sprintf(s_uptime, ">49d");
  } else {
    int d = (t / 60 / 60 / 24);
    int h = ((t / 60 / 60) % 24);
    int m = ((t / 60) % 60);
    if (d)
      sprintf(s_uptime, "%dd%2.2d:%2.2d", d, h, m);
    else
      sprintf(s_uptime, "%2.2d:%2.2d", h, m);
    old_time = t;
  }
  OledLine1 = gps_time_s;
  OledLine1 = OledLine1 + " Up " + s_uptime;
}

void fillDisplayLine2() {

#ifdef	ENABLE_WIFI
        wifi_info = "WiFi";
        if (wifi_connection_status == WIFI_RUNNING_AS_AP) {
          wifi_info = (enable_webserver)? wifi_info + "-AP" : wifi_info + ":off";
        } else if (wifi_connection_status == WIFI_SEARCHING_FOR_AP) {
          wifi_info = (enable_webserver)? wifi_info + "-cli" : wifi_info + ":off";
        } else {
          wifi_info = (enable_webserver)? wifi_info + "-CLI" : wifi_info + ":off";
        }
#else        
        wifi_info = "";
#endif

  if (dont_send_own_position_packets || !lora_tx_enabled) {
    OledLine2 = wifi_info + " never TX";
  } else {
    if (gps_isValid) {
      if (fixed_beacon_enabled) {
        OledLine2 = wifi_info+ " FB:" +String((next_fixed_beacon-millis()) / 1000) + "s";
      } else {
        ulong nextSBtx = ((lastTX+nextTX)-millis())/1000;
        // do not send SB Info on very first tx, wrong value, cannot find the reason
        if (nextSBtx > 400000) {
          OledLine2 = wifi_info;
        } else {
          OledLine2 = wifi_info + " SB:" + String(nextSBtx) + "s";
        // next line will be used when debugging gps-output for smoothening km/h in oled
        //OledLine1s = OledLine1 + String(((lastTX+nextTX)-millis())/1000)+"s "+ String(lastTxdistance);
        }
      }
    } else {
      OledLine2 = wifi_info;
      }
    }
}

void fillDisplayLines3to5() {
  static uint32_t ratelimit_until = 0L;


  if (millis() < ratelimit_until)
    return;
  ratelimit_until = millis() + (oled_timeout == 0 ? 1000 : showRXTime);

  OledLine3 = aprsLatPreset + " " + aprsLonPreset + " " + aprsPresetShown;
  OledLine4 = getSpeedCourseAlti();
  OledLine5 = getSatAndBatInfo();
}


void displayInvalidGPS() {
  uint32_t gpsage;
  String gpsage_p = "";
  fillDisplayLine1();
  if (!gps_state){
    OledLine2 = wifi_info + " GPS dis";
  } else {
//  show GPS age (only if last retrieval was invalid)
    gpsage = gps.location.age()/1000;
    gpsage_p = String(gpsage) + "s" ;
    gpsage_p = (gpsage > 60)? String(gpsage/60) + "m" : gpsage_p;
    gpsage_p = (gpsage > 3600)? String(gpsage/3600) + "h" : gpsage_p;
    gpsage_p = (gpsage > 86400)? String(gpsage/86400) + "d" : gpsage_p;
    gpsage_p = (gpsage > 4000000)? " no GPS fix" : " GPS age:" + gpsage_p;
    OledLine2 = wifi_info + gpsage_p;
  }
  fillDisplayLines3to5();
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
void sendToWebList(const String& TNC2FormatedFrame, const int RSSI, const int SNR) {
  if (webListReceivedQueue){
    auto *receivedPacketData = new tReceivedPacketData();
    receivedPacketData->packet = new String();
    receivedPacketData->packet->concat(TNC2FormatedFrame);
    // strip EOL
    receivedPacketData->packet->trim();
    receivedPacketData->RSSI = RSSI;
    receivedPacketData->SNR = SNR;
    getLocalTime(&receivedPacketData->rxTime);

    if (xQueueSend(webListReceivedQueue, &receivedPacketData, (1000 / portTICK_PERIOD_MS)) != pdPASS){
      // remove buffer on error
      delete receivedPacketData->packet;
      delete receivedPacketData;
    }
  }
}
#endif

String prepareCallsign(const String& callsign){
  String tmpString = "";
  for (int i=0; i<callsign.length();++i){  // remove unneeded "spaces" from callsign field
    if (callsign.charAt(i) != ' ') {
      tmpString += callsign.charAt(i);
    }
  }
  return tmpString;
}

#if defined(ENABLE_TNC_SELF_TELEMETRY) && defined(KISS_PROTOCOL)
  void sendTelemetryFrame() {
    if(enable_tel == true){
      #ifdef T_BEAM_V1_0
        uint8_t b_volt = (axp.getBattVoltage() - 3000) / 5.1;
        uint8_t b_in_c = (axp.getBattChargeCurrent()) / 10;
        uint8_t b_out_c = (axp.getBattDischargeCurrent()) / 10;
        uint8_t ac_volt = (axp.getVbusVoltage() - 3000) / 28;
        uint8_t ac_c = (axp.getVbusCurrent()) / 10;
        // Pad telemetry message address to 9 characters
        char Tcall_message_char[9];
        sprintf_P(Tcall_message_char, "%-9s", Tcall.c_str());
        String Tcall_message = String(Tcall_message_char);
        // Flash the light when telemetry is being sent
        #ifdef ENABLE_LED_SIGNALING
          digitalWrite(TXLED, LOW);
        #endif

        // Determine sequence number (or 'MIC')
        String tel_sequence_str;
        if(tel_mic == 1){
          tel_sequence_str = "MIC";
        }else{
          // Get the current saved telemetry sequence
          tel_sequence = preferences.getUInt(PREF_TNC_SELF_TELEMETRY_SEQ, 0);
          // Pad to 3 digits
          char tel_sequence_char[3];
          sprintf_P(tel_sequence_char, "%03u", tel_sequence);
          tel_sequence_str = String(tel_sequence_char);
        }
        // Format telemetry path
        String tel_path_str;
        if(tel_path == ""){
          tel_path_str = tel_path;
        }else{
          tel_path_str = "," + tel_path;
        }

        String telemetryParamsNames = String(":") + Tcall_message + ":PARM.B Volt,B In,B Out,AC V,AC C";
        String telemetryUnitNames = String(":") + Tcall_message + ":UNIT.mV,mA,mA,mV,mA";
        String telemetryEquations = String(":") + Tcall_message + ":EQNS.0,5.1,3000,0,10,0,0,10,0,0,28,3000,0,10,0";
        //String telemetryData = String("T#") + tel_sequence_str + "," + String(b_volt) + "," + String(b_in_c) + "," + String(b_out_c) + "," + String(ac_volt) + "," + String(ac_c) + ",00000000";
        String telemetryData = String("T#") + tel_sequence_str + "," + String(b_volt) + "," + String(b_in_c) + "," + String(b_out_c) + "," + String(ac_volt) + "," + String(ac_c);
        String telemetryBase = "";
        telemetryBase += Tcall + ">" + MY_APRS_DEST_IDENTIFYER + tel_path_str + ":";
        Serial.print(telemetryBase);
        sendToTNC(telemetryBase + telemetryParamsNames);
        sendToTNC(telemetryBase + telemetryUnitNames);
        sendToTNC(telemetryBase + telemetryEquations);
        sendToTNC(telemetryBase + telemetryData);

        // Show when telemetry is being sent
        writedisplaytext("((TEL TX))","","","","","");

        // Flash the light when telemetry is being sent
        #ifdef ENABLE_LED_SIGNALING
          digitalWrite(TXLED, HIGH);
        #endif

        // Update the telemetry sequence number
        if(tel_sequence >= 999){
          tel_sequence = 0;
        }else{
          tel_sequence = tel_sequence + 1;
        }
        preferences.putUInt(PREF_TNC_SELF_TELEMETRY_SEQ, tel_sequence);
      #endif
    }
  }
#endif


// + SETUP --------------------------------------------------------------+//
void setup(){
#ifdef T_BEAM_V0_7 /*
  adcAttachPin(35);
  adcStart(35);
  analogReadResolution(10);
  analogSetAttenuation(ADC_6db); */
  pinMode(35, INPUT);
  //adc1_config_width(ADC_WIDTH_BIT_12);
  //adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_11);
#endif

  SPI.begin(SPI_sck,SPI_miso,SPI_mosi,SPI_ss);    //DO2JMG Heltec Patch
  Serial.begin(115200);

  #ifdef BUZZER
    ledcSetup(0,1E5,12);
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

// This section loads values from saved preferences,
// if available.
// https://randomnerdtutorials.com/esp32-save-data-permanently-preferences/

  #ifdef ENABLE_PREFERENCES
    int clear_preferences = 0;
    if(digitalRead(BUTTON)==LOW){
      clear_preferences = 1;
    }

    preferences.begin("cfg", false);

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
      preferences.putBool(PREF_WIFI_STA_ALLOW_FAILBACK_TO_MODE_AP_AFTER_ONCE_CONNECTED, wifi_do_failback_to_mode_AP);
    }
    wifi_do_failback_to_mode_AP = preferences.getBool(PREF_WIFI_STA_ALLOW_FAILBACK_TO_MODE_AP_AFTER_ONCE_CONNECTED);

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
#endif
    
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
      preferences.putString(PREF_APRS_SYMBOL_TABLE, APRS_SYMBOL_TABLE);
      aprsSymbolTable = preferences.getString(PREF_APRS_SYMBOL_TABLE);
    }

    aprsSymbol = preferences.getString(PREF_APRS_SYMBOL, "");
    if (aprsSymbol.isEmpty()){
      preferences.putString(PREF_APRS_SYMBOL, APRS_SYMBOL);
      aprsSymbol = preferences.getString(PREF_APRS_SYMBOL, APRS_SYMBOL);
    }

    if (!preferences.getBool(PREF_APRS_COMMENT_INIT)){
      preferences.putBool(PREF_APRS_COMMENT_INIT, true);
      preferences.putString(PREF_APRS_COMMENT, MY_COMMENT);
    }
    aprsComment = preferences.getString(PREF_APRS_COMMENT, "");

    if (!preferences.getBool(PREF_APRS_RELAY_PATH_INIT)){
      preferences.putBool(PREF_APRS_RELAY_PATH_INIT, true);
      preferences.putString(PREF_APRS_RELAY_PATH, DIGI_PATH);
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

    if (!preferences.getBool(PREF_APRS_LATITUDE_PRESET_INIT)){
      preferences.putBool(PREF_APRS_LATITUDE_PRESET_INIT, true);
      preferences.putString(PREF_APRS_LATITUDE_PRESET, LATITUDE_PRESET);
    }
    aprsLatPreset = preferences.getString(PREF_APRS_LATITUDE_PRESET, "");
    //LatShownP = aprsLonPreset;

    if (!preferences.getBool(PREF_APRS_LONGITUDE_PRESET_INIT)){
      preferences.putBool(PREF_APRS_LONGITUDE_PRESET_INIT, true);
      preferences.putString(PREF_APRS_LONGITUDE_PRESET, LONGITUDE_PRESET);
    }
    aprsLonPreset = preferences.getString(PREF_APRS_LONGITUDE_PRESET, "");
    //LongShownP = aprsLonPreset;

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

// + SMART BEACONING

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

    if (!preferences.getBool(PREF_DEV_SHOW_RX_TIME_INIT)){
      preferences.putBool(PREF_DEV_SHOW_RX_TIME_INIT, true);
      preferences.putInt(PREF_DEV_SHOW_RX_TIME, showRXTime/1000);
    }
    showRXTime = preferences.getInt(PREF_DEV_SHOW_RX_TIME) * 1000;

    // Read OLED RX Timer
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
    enable_bluetooth = preferences.getBool(PREF_DEV_BT_EN);

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

    if (clear_preferences){
      delay(1000);
      if(digitalRead(BUTTON)==LOW){
        clear_preferences = 2;
      }
    }

  #endif

  // We have stored the manual position strring in a higher precision (in case resolution more precise than 18.52m is required; i.e. for base-91 location encoding, or DAO extenstion).
  // Furthermore, 53-32.1234N is more readable in the Web-interface than 5232.1234N
  aprsLatPreset.toUpperCase(); aprsLatPreset.replace(",", "."); aprsLatPreset.trim();
  if (aprsLatPreset.length() == 11 && aprsLatPreset.indexOf('-') == 2 && aprsLatPreset.indexOf(' ') == -1 && (aprsLatPreset.endsWith("N") || aprsLatPreset.endsWith("S"))) {
    char buf[9];
    const char *p = aprsLatPreset.c_str();
    sprintf(buf, "%.2s%.5s%c", p, p+3, p[10]);
    aprsLatPreset = String(buf);
  }

  // 001-20.5000E is more readable in the Web-interface than 00120.5000E, and could not be mis-interpreted as 120.5 degrees east  (== 120 deg 30' 0" E)
  aprsLonPreset.toUpperCase(); aprsLonPreset.replace(",", "."); aprsLonPreset.trim();
  if (aprsLonPreset.length() == 12 && aprsLonPreset.indexOf('-') == 3 && aprsLonPreset.indexOf(' ') == -1 && (aprsLonPreset.endsWith("E") || aprsLonPreset.endsWith("W"))) {
    char buf[10];
    const char *p = aprsLonPreset.c_str();
    sprintf(buf, "%.3s%.5s%c", p, p+4, p[11]);
    aprsLonPreset = String(buf);
  }

  // enforce valid transmissions even on wrong configurations
  if (aprsSymbolTable.length() != 1)
    aprsSymbolTable = String("/");
  if (aprsSymbol.length() != 1)
    aprsSymbol = String("[");
  if (aprsLatPreset.length() != 8 || !(aprsLatPreset.endsWith("N") || aprsLatPreset.endsWith("S")) || aprsLatPreset.c_str()[4] != '.')
    aprsLatPreset = String("0000.00N");
  if (aprsLonPreset.length() != 9 || !(aprsLonPreset.endsWith("E") || aprsLonPreset.endsWith("W")) || aprsLonPreset.c_str()[5] != '.')
    aprsLonPreset = String("00000.00E");

  for (int i=0;i<ANGLE_AVGS;i++) {                                        // set average_course to "0"
    average_course[i]=0;
  }

  pinMode(TXLED, OUTPUT);
  #ifdef T_BEAM_V1_0
    pinMode(BUTTON, INPUT);
  #elif T_BEAM_V0_7
    pinMode(BUTTON, INPUT);
  #else
    pinMode(BUTTON, INPUT_PULLUP);
  #endif
  digitalWrite(TXLED, LOW);                                               // turn blue LED off

  Wire.begin(I2C_SDA, I2C_SCL);

  #ifdef T_BEAM_V1_0
    if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
    }
    axp.setLowTemp(0xFF);                                                 //SP6VWX Set low charging temperature
    if (lora_rx_enabled || lora_digipeating_mode > 0 || SerialBT.hasClient())
      axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);                            // switch LoRa chip on
    else
      axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF);                           // switch LoRa chip off
    if (gps_state){
      axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);                           // switch on GPS
    } else {
      axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);                           // switch off GPS
    }
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);
    //axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);				// switch this on if you need it
    axp.setDCDC1Voltage(3300);
    // Enable ADC to measure battery current, USB voltage etc.
    axp.adc1Enable(0xfe, true);
    axp.adc2Enable(0x80, true);
    axp.setChgLEDMode(AXP20X_LED_OFF);
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);                          // oled do not turn off
  #endif
  // can reduce cpu power consumtion up to 20 %
  if (adjust_cpuFreq_to > 0)
    setCpuFrequencyMhz(adjust_cpuFreq_to);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SSD1306_ADDRESS)) {
      for(;;);                                                             // Don't proceed, loop forever
  }

  #ifdef ENABLE_PREFERENCES
    if (clear_preferences == 2){
      String buildnr = BUILD_NUMBER;
      writedisplaytext("LoRa-APRS","by DL9SAU & DL3EL","Build:" + String(BUILD_NUMBER),"Factory reset","","");
      delay(2000);
      //#ifdef T_BEAM_V1_0
        if(digitalRead(BUTTON)==LOW){
          clear_preferences = 3;
          preferences.clear();
          preferences.end();
          writedisplaytext("LoRa-APRS","","Factory reset","Done!","","");
          delay(2000);
          ESP.restart();
        } else {
          writedisplaytext("LoRa-APRS","","Factory reset","Cancel","","");
          delay(2000);
        }
      //#endif
    }
  #endif
  writedisplaytext("LoRa-APRS","","Init:","Display OK!","","");

  Tcall = prepareCallsign(String(CALLSIGN));
  #ifdef ENABLE_PREFERENCES
    Tcall = preferences.getString(PREF_APRS_CALLSIGN, "");
    if (Tcall.isEmpty()){
      preferences.putString(PREF_APRS_CALLSIGN, String(CALLSIGN));
      Tcall = preferences.getString(PREF_APRS_CALLSIGN);
    }
  #endif

  if (!rf95.init()) {
    writedisplaytext("LoRa-APRS","","Init:","RF95 FAILED!",":-(","");
    for(;;); // Don't proceed, loop forever
  }

  if (sb_max_interval < nextTX){
    sb_max_interval=nextTX;
  }

  // we need this assurance for failback to fixed interval, if gps position is lost.
  // fixed beacon rate higher than sb_max_interval does not make sense
  if (!fixed_beacon_enabled && gps_state && fix_beacon_interval < sb_max_interval)
    fix_beacon_interval = sb_max_interval;

  writedisplaytext("LoRa-APRS","","Init:","RF95 OK!","","");
  writedisplaytext(Tcall,"","Init:","Waiting for GPS","","");
  xTaskCreate(taskGPS, "taskGPS", 5000, nullptr, 1, nullptr);
  writedisplaytext(Tcall,"","Init:","GPS Task Created!","","");
  #ifndef T_BEAM_V1_0
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_6);
  #endif
  batt_read();
  writedisplaytext("LoRa-APRS","","Init:","ADC OK!","BAT: "+String(BattVolts,2),"");

  // if we are fill-in or wide2 digi, we listen only on configured main frequency
  lora_speed_rx_curr = (rx_on_frequencies  != 2 || lora_digipeating_mode > 1) ? lora_speed : lora_speed_cross_digi;
  lora_set_speed(lora_speed_rx_curr);
  Serial.printf("LoRa Speed:\t%lu\n", lora_speed_rx_curr);

  lora_freq_rx_curr = (rx_on_frequencies  != 2 || lora_digipeating_mode > 1) ? lora_freq : lora_freq_cross_digi;
  rf95.setFrequency(lora_freq_rx_curr);
  Serial.printf("LoRa FREQ:\t%f\n", lora_freq_rx_curr);

  // we tx on main and/or secondary frequency. For tx, loraSend is called (and always has desired txpower as argument)
  rf95.setTxPower((lora_digipeating_mode < 2 || lora_cross_digipeating_mode < 1) ? txPower : txPower_cross_digi);
  delay(250);
  #ifdef KISS_PROTOCOL
    xTaskCreatePinnedToCore(taskTNC, "taskTNC", 10000, nullptr, 1, nullptr, xPortGetCoreID());
  #endif


#if defined(KISS_PROTOCOL) && defined(ENABLE_BLUETOOTH)
  // LORA32_21: bug in hardware. cannot run bluetooth and wifi concurrently.
  // We wait for a bt-client connecting, up to 60s. If none connected,
  // we start the webserver.
  // TTGO: webserver cunsumes abt 80mA. User may not start the webserver
  // if bt-client is connected. We'll also wait herefor clients.
  // If enable_webserver on LORA32_21 is set to 2, user
  // likes the webserver always to be started -> do not start bluetooth.
#if defined(ENABLE_WIFI)
#if defined(LORA32_21)
  if (enable_bluetooth && enable_webserver < 2) {
#else
  if (enable_bluetooth) {
#endif /* LORA32_21 */
#else
  if (enable_bluetooth) {
#endif /* ENABLE_WIFI */

#ifdef BLUETOOTH_PIN
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
  #endif
      } else {
        writedisplaytext("LoRa-APRS","","Init:","Waiting for BT-clients","BT-client connected","Will NOT start WiFi!");
      }
      delay(1500);
    }
#endif /* ENABLE_WIFI */
  }
#endif /* KISS_PROTOCOL && ENABLE_BLUETOOTH */

#ifdef ENABLE_WIFI
  if (enable_webserver) {
#if defined(KISS_PROTOCOL) && defined(ENABLE_BLUETOOTH)
    // if enabble_webserver == 2 or (enable_webserver == 1 && (no serial-bt-client is connected OR aprs-is-connecion configuried)
    if (enable_webserver > 1 || aprsis_enabled || !SerialBT.hasClient()) {
#else
    {
#endif /* KISS_PROTOCOL && ENABLE_BLUETOOTH */
      webServerCfg = {.callsign = Tcall};
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
#endif /* ENABLE_WIFI */
  }

  writedisplaytext("LoRa-APRS","","Init:","FINISHED OK!","   =:-)   ","");
//  writedisplaytext("","","","","","");
  time_to_refresh = millis() + showRXTime;
  fillDisplayLine1();
  fillDisplayLine2();
  displayInvalidGPS();
  digitalWrite(TXLED, HIGH);

  // Hold the OLED ON at first boot
  oled_timer=millis()+oled_timeout;

  esp_task_wdt_init(120, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  lastTxdistance  = 0;

#ifdef IF_SEMAS_WOULD_WORK
  sema_lora_chip = xSemaphoreCreateBinary();
#else
  sema_lora_chip = false;
#endif
}

void enableOled() {
  // This function enables OLED display after pressing a button
  tempOled = true;
  oled_timer = millis() + oled_timeout;
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
  // src-call_validation
  const char *p_call = frame_start;
  int i = 0;
  if (!p_call || !*p_call) return 1;
  if (isdigit(p_call[1]) && isalpha(p_call[2])) {
    // left-shift g1abc -> _g1abc
    i = 1;
    p_call--; // warning, beyond start of pointer
  }
  for (; i <= 7; i++) {
    if (i == 7) return 1;
    else if (!p_call[i] || p_call[i] == '>' || p_call[i] == '-') {
      if (i < 4) return 1;
      break;
    } else if (i < 2 && !isalnum(p_call[i])) return 1;
    else if (i == 2 && !isdigit(p_call[i])) return 1;
    else if (i > 2 && !isalpha(p_call[i])) return 1;
  }

  // list empty? we may leave here
  if (!*blacklist_calls || !strcmp(blacklist_calls, ",,"))
    return 0;

  boolean ssid_present = false;
  char buf[12]; // room for ",DL1AAA-15," + \0
  char *p = buf;
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
    // for being able to filter out DL1AAA but not DL1AAA-1. -> DL1AAA is DL1AAA-0 by AX.25 definition. Blacklist lists DL1AAA-0. DL1AAA means: filter all variants.:w
    *p++ = '-'; *p++ = '0';
  }
  *p++ = ',';
  *p = 0;

  // exact match?
  if (strstr(blacklist_calls, buf))
    return 2;
  // filter call completely?
  if ((p = strchr(buf, '-'))) {
    *p++ = ','; *p++ = 0;
    if (strstr(blacklist_calls, buf))
      return 3;
  }

  char *header_end = strchr(frame_start, ':');
  // check for blacklisted digi in path
  if (header_end && (p = strchr(frame_start, ','))) {
    for (;;) {
      char *q = strchr(p+1, ',');
      if (!q || q > header_end)
        q = header_end;
      // copy ",DL1AAA,.." to buf as ",DL1AAA"
      // but before: length check. len ",DL1AAA-15*," is 12; sizeof(buf) is 12 (due to \0); we copy until trailing ','.
      if ((q-p) > sizeof(buf)-1)
        break;
      strncpy(buf, p, q-p);
      buf[q-p] = 0;
      char *r = strchr(buf, '*');
      if (r)
        *r = 0;
      // our ssid filter construct: -0 means search for call with ssid 0 zero.
      if (!(r = strchr(buf, '-')))
        strcat(buf, "-0");
      // after modifications above, is len(buf) still < 10 (space for ',' and \0)?
      if (strlen(buf) > 10)
        return 0;
      strcat(buf, ",");
      // exact match?
      if (strstr(blacklist_calls, buf))
        return 4;
      // filter call completely?
      if ((r = strchr(buf, '-'))) {
        *r++ = ','; *r++ = 0;
        if (strstr(blacklist_calls, buf))
          return 5;
      }
      if (q == header_end)
        break;
      p = q;
    }
  }

  return 0;
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


int bg_rf95rssi_to_rssi(int rssi)
{
  // We use an old BG_RF95 library from 2001. RadioHead/RH_RF95.cpp implements _lastSNR and _lastRssi correctly accordingg to the specs
  int _lastSNR = bg_rf95snr_to_snr(rf95.lastSNR());
  boolean _usingHFport = (lora_freq >= 779.0);

  // bg_rf95 library: _lastRssi = spiRead(BG_RF95_REG_1A_PKT_RSSI_VALUE) - 137. First, undo -137 operation
  int _lastRssi = rf95.lastRssi() + 137;

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


char *encode_snr_rssi_in_path()
{
  static char buf[7]; // length for "Q2373X" == 6 + 1 (\0) == 7
  *buf = 0;
  // SNR values reported by rf95.lastSNR() are not plausible. See those two projects:
  // https://github.com/Lora-net/LoRaMac-node/issues/275
  // https://github.com/mayeranalytics/pySX127x/blob/master/SX127x/LoRa.py
  // rf95snr_to_snr returns values in range -32 to 31. The lowest two bits are RFU
  int snr = bg_rf95snr_to_snr(rf95.lastSNR());
  int rssi = bg_rf95rssi_to_rssi(rf95.lastRssi());
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
  static char buf[BG_RF95_MAX_MESSAGE_LEN+1];
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
  static char buf[BG_RF95_MAX_MESSAGE_LEN+10+1];
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
  static char data[BG_RF95_MAX_MESSAGE_LEN+1];
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
    if (p && strncmp(lora_TXBUFF_for_digipeating, received_frame, p-received_frame) && r && r > q && r < header_end && *(header_end+1))
      *lora_TXBUFF_for_digipeating = 0;
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
  {
    sprintf(buf + strlen(buf), ",%s*", Tcall.c_str());
  }

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

String create_lat_aprs(const char *delimiter, RawDegrees lat) {
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
  sprintf(str, "%02d%s%s%c", lat.deg, delimiter ? delimiter : "", s_min_nn(lat.billionths, 0), n_s);
  return String(str);
}

String create_long_aprs(const char *delimiter, RawDegrees lng) {
  char str[20];
  char e_w = 'E';
  if (lng.negative) {
    e_w = 'W';
  }
  if (delimiter && strlen(delimiter) > 1)
    delimiter = 0;
  sprintf(str, "%03d%s%s%c", lng.deg, delimiter ? delimiter : "", s_min_nn(lng.billionths, 0), e_w);
  return String(str);
}


// +---------------------------------------------------------------------+//
// + MAINLOOP -----------------------------------------------------------+//
// +---------------------------------------------------------------------+//
void loop() {

  esp_task_wdt_reset();

  if (reboot_interval && millis() > reboot_interval) {
    ESP.restart();
  }

  sendpacket_was_called_twice = false;

  // Ticker blinks upper left corner to indicate system is running
  // only when OLED is on
  if (tempOled) {
    display_refresh1s();
  }  

  if(digitalRead(BUTTON)==LOW && key_up == true){
    key_up = false;
    delay(50);
    if(digitalRead(BUTTON)==LOW){
      delay(300);
      time_delay = millis() + 1500;
      if(digitalRead(BUTTON)==HIGH){
        if (!tempOled && enabled_oled) {
          enableOled(); // turn ON OLED temporary
        } else {
          fillDisplayLines3to5();
          if (gps_state == true && gps_isValid) {
#ifdef ENABLE_WIFI
            writedisplaytext("((MAN TX))","SSID: " + infoApName,"IP: " + infoIpAddr, OledLine3, OledLine4, OledLine5);
#else
            writedisplaytext("((MAN TX))","","",OledLine3, OledLine4, OledLine5);
#endif
            sendpacket(SP_POS_GPS);
          } else {
#ifdef ENABLE_WIFI
            writedisplaytext("((FIX TX))","SSID: " + infoApName,"IP: " + infoIpAddr, OledLine3, OledLine4, OledLine5);
#else
            writedisplaytext("((FIX TX))","","",OledLine3, OledLine4, OledLine5);
#endif
            sendpacket(SP_POS_FIXED);
          }
          // reset timer for automatic fixed beacon after manual beacon
          next_fixed_beacon = millis() + fix_beacon_interval;
        }
        key_up = true;
      }
    }
  }

  if (gps.time.isValid() && gps.time.age() < 500) {
    if (gps.time.isUpdated()) {
      static int8_t t_hour_adjust_next = -1;
      // Time to adjust time?
      if (t_hour_adjust_next == -1 || t_hour_adjust_next == gps.time.hour()) {
        struct tm t;
        time_t t_of_day;
        t.tm_year = gps.date.year()-1900;
        t.tm_mon = gps.date.month()-1;	// Month, 0 - jan
        t.tm_mday = gps.date.day();	// Day of the month
        t.tm_hour = gps.time.hour();
        t.tm_min =  gps.time.minute();
        t.tm_sec = gps.time.second();
        t_of_day = mktime(&t);
        int epoch_time = t_of_day;
        timeval epoch = {epoch_time, 0};
        const timeval *tv = &epoch;
        settimeofday(tv, NULL);
        t_hour_adjust_next = (t.tm_hour + 1) % 24;
      }
    }
  }

// time is now taken from system time and updates once per second in display_blinker

  // LatShownP  = gg-mm.dd[N|S]
  // aprsLatPreset = ggmm.dd[N|S]
  boolean gps_isValid_oldState = gps_isValid;
  gps_isValid = false;
  if (gps_state && gps.hdop.hdop() < 8) {
    if (gps.speed.isValid() && gps.speed.age() < 10000) {
      if (gps.speed.isUpdated())
        gps_speed_kmph = gps.speed.kmph();
    } // else: assume we still move with last speed
  }
  if (gps.location.isValid() && gps.location.age() < 10000) {

    gps_isValid = true;

    if (gps.location.isUpdated()) {
      static uint32_t ratelimit_until = 0L;

      // to smoothen the speed, the distance between pos is calculated on gps retrieval. (DL3EL)
      //LatShownP = create_lat_aprs("-", gps.location.rawLat());
      //LongShownP = create_long_aprs("-", gps.location.rawLng());
      // save last valid position as new fixed location

      // aprs resolution is 1sm/100 = 1852m/100 = 18.52m
      // Updating this every 200ms is enough for getting this resolution at speed of 18.52*3.6*5 = 333.36km/h.
      // aprsLatPreset and aprsLonPreset are used for displaying. The transmission uses the true value.
      if (millis() >  ratelimit_until) {
        aprsLatPreset = create_lat_aprs("", gps.location.rawLat());
        aprsLonPreset = create_long_aprs("", gps.location.rawLng());
        aprsPresetShown = "";
        ratelimit_until = millis() + 200;
      }
    }

#ifdef notdef
    static uint32_t lastTxdistance_millis = 0;
    // code has no function, as currently gps_speed_kmph_oled is not used in getSpeedCourseAlti
    if ((millis()-lastTxdistance_millis) > 1000) {
      static double lastTxLat       = 0.0;
      static double lastTxLng       = 0.0;
      double currLat = gps.location.lat();
      double currLng = gps.location.lng();
      double dist = TinyGPSPlus::distanceBetween(currLat, currLng, lastTxLat, lastTxLng);
      // test code to smoothen km/h in oled, does not work, because of weird data from gps. Has to be looked at
      // get GPS Data, if valid and mark accordingly
      if (dist > 15 || (millis()-lastTxdistance_millis) > 3*60*10000) {
        lastTxLat = currLat;
        lastTxLng = currLng;
        gps_speed_kmph_oled = gps_speed_kmph;
        lastTxdistance_millis = millis();
      } else {
        gps_speed_kmph_oled = 0;
      }
    }
#endif
  }
  if (gps_isValid != gps_isValid_oldState) {
    // String functions are cpu consuming. We adust string aprsPresetShown only if we changed status.
    if (!gps_isValid) {
      // update to the old values
      gps_speed_kmph = gps.speed.kmph();
      aprsLatPreset = create_lat_aprs("", gps.location.rawLat());
      aprsLonPreset = create_long_aprs("", gps.location.rawLng());
      aprsPresetShown = "p";
    } else {
      aprsPresetShown = "";
    }
  } else if (!gps_isValid) {
    // isValid change of previous run, and still invalid
    gps_speed_kmph = 0;
    gps_speed_kmph_oled = 0;
  }


#ifdef	ENABLE_WIFI
  // Show informations on WiFi Status, only once after state change
  if (wifi_connection_status_prev != wifi_connection_status) {
    enableOled(); // turn ON OLED temporary
    if (wifi_connection_status == WIFI_CONNECTED_TO_AP) {
      writedisplaytext("((WiFi))","WiFi Client Mode","SSID: " + infoApName, "Pass: ********", "IP: " + infoIpAddr, getSatAndBatInfo());
    } else if (wifi_connection_status == WIFI_SEARCHING_FOR_AP) {
      writedisplaytext("((WiFi))","WiFi Client Mode","SSID: " + infoApName, "Not in sight!", "IP: none", getSatAndBatInfo());
    } else if (wifi_connection_status == WIFI_RUNNING_AS_AP) {
      writedisplaytext("((WiFi))","WiFi AP Mode","SSID: " + infoApName, "Pass: " + infoApPass, "IP: " + infoIpAddr, getSatAndBatInfo());
    } else {
//      writedisplaytext("((WiFi))","WiFi off","SSID: " + infoApName, "Pass: " + infoApPass, "IP: " + infoIpAddr, getSatAndBatInfo());
      writedisplaytext("((WiFi))","WiFi off","press key long","to enable","", getSatAndBatInfo());
    }
    wifi_connection_status_prev = wifi_connection_status;
//  initial fill of line2
    fillDisplayLine2();
  }
#endif

  if (manBeacon) {
    // Manually sending beacon from html page
    enableOled();
    fillDisplayLines3to5();
#ifdef	ENABLE_WIFI
    writedisplaytext("((WEB TX))","SSID: " + infoApName,"IP: " + infoIpAddr, OledLine3, OledLine4, OledLine5);
#else
    writedisplaytext("((WEB TX))","","",OledLine3, OledLine4, OledLine5);
#endif
    sendpacket(SP_POS_GPS);
    manBeacon=false;
  }
  // Only wake up OLED when necessary, note that DIM is to turn OFF the backlight
  if (enabled_oled) {
    if (oled_timeout > 0) {
      display.dim(!tempOled);
    } else {
      // If timeout is 0 keep OLED awake
      display.dim(false);
    }
  }

  if (tempOled && oled_timeout > 0 && millis() >= oled_timer) {
    tempOled = false; // After some time reset backlight
  }

  if (digitalRead(BUTTON)==LOW && key_up == false && millis() >= time_delay && t_lock == false) {
    // enable OLED
    enableOled();
    //---------------
    t_lock = true;
//  re-enable webserver, if was set to off.
#ifdef	ENABLE_WIFI
    if (!webserverStarted) {
      enable_webserver = 1;
#if defined(LORA32_21) && defined(ENABLE_BLUETOOTH)
      // lora32_21 hardware bug: btt and wifi are mutual exclusive
      SerialBT.end();
      delay(100);
#endif
      webServerCfg = {.callsign = Tcall};
      xTaskCreate(taskWebServer, "taskWebServer", 12000, (void*)(&webServerCfg), 1, nullptr);
      webserverStarted = true;
      writedisplaytext("LoRa-APRS","","Init:","WiFi task started","long press to ","stop again");
      delay(1500);
    } else {
      writedisplaytext("LoRa-APRS","","Reboot:","to stop WiFi","   =:-)   ","");
      ESP.restart();
    }  
#endif
  }

  if(digitalRead(BUTTON)==HIGH && !key_up){
    key_up = true;
    t_lock = false;
  }

  if (dont_send_own_position_packets) {
#ifdef KISS_PROTOCOL
    // reset to default state if kiss device is disconnected, silent or last position frame was seen long time ago (i.e. 1h).
    // TODO: also iterate through KISS tcp-devices in the parts below wih 'kiss_client_came_via_bluetooth'
    if (
#ifdef ENABLE_BLUETOOTH
         (kiss_client_came_via_bluetooth && !SerialBT.hasClient()) ||
#endif
         (((time_last_own_position_via_kiss_received + sb_max_interval + 10*1000L) < millis()) &&
             time_last_own_position_via_kiss_received >= time_last_frame_via_kiss_received) ||
           // ^kiss client has not recently sent a position gain (sb_max_interval plus 10 seconds grace) and kiss client sent no other data
	 ((time_last_frame_via_kiss_received + sb_max_interval * 2 + 10*1000L) < millis())) {
            // ^ kiss client sent no positions and stoped sending other data for 2*sb_max_interval (plus 10 seconds grace)
#ifdef T_BEAM_V1_0
      if (!gps_state && gps_state_before_autochange)
        axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
#endif
      gps_state = gps_state_before_autochange;
      dont_send_own_position_packets = false;
      time_last_own_position_via_kiss_received = 0L;
      time_last_frame_via_kiss_received = 0L;
#ifdef ENABLE_BLUETOOTH
      if (kiss_client_came_via_bluetooth && !SerialBT.hasClient())
        kiss_client_came_via_bluetooth = false;
#endif
    }
#endif
  }

  // fixed beacon, or if smartbeaconing with lost gps fix (but had at least one gps fix).
  // smartbeaconing also ensures correct next_fixed_beacon time
  if (!dont_send_own_position_packets && millis() >= next_fixed_beacon &&
         (fixed_beacon_enabled ||
           (t_last_smart_beacon_sent && (!gps_state || !gps_isValid)) ) ) {
      enableOled(); // enable OLED
      next_fixed_beacon = millis() + fix_beacon_interval;
      fillDisplayLines3to5();
      writedisplaytext("((AUT TX))", "", "fixed", OledLine3, OledLine4, OledLine5);
      sendpacket(SP_POS_FIXED);
  }

  #ifdef T_BEAM_V1_0
    if(shutdown_active){
      if(InpVolts> 4){
        shutdown_usb_status_bef = true;
        shutdown_countdown_timer_enable = false;
      }

      if(InpVolts < 4 && shutdown_usb_status_bef == true){
        shutdown_usb_status_bef = false;
        shutdown_countdown_timer_enable = true;
        shutdown_countdown_timer = millis() + shutdown_delay_time;
      }

      if(shutdown_countdown_timer_enable){
        if(millis() >= shutdown_countdown_timer){
          axp.setChgLEDMode(AXP20X_LED_OFF);
          axp.shutdown();
        }
      }
    }
  #endif

  #ifdef KISS_PROTOCOL
    String *TNC2DataFrame = nullptr;
    if (tncToSendQueue) {
      if (xQueueReceive(tncToSendQueue, &TNC2DataFrame, (1 / portTICK_PERIOD_MS)) == pdPASS) {
        boolean was_own_position_packet = false;
        time_last_frame_via_kiss_received = millis();
        const char *data = TNC2DataFrame->c_str();

	// Frame comes from same call as ours and is a position report?
        if (!strncmp(data, Tcall.c_str(), Tcall.length()) && data[Tcall.length()] == '>') {
          char *p = strchr(data, ':');
          p++;
          if (*p == '!' || *p == '=' || *p == '/' || *p == '@' || *p == '\'' || *p == '`' || *p == '[' || *p == '$') {
            time_last_own_position_via_kiss_received = time_last_frame_via_kiss_received;
            if (!acceptOwnPositionReportsViaKiss)
              goto out; // throw away this frame.
            was_own_position_packet = true;
            if (!dont_send_own_position_packets) {
              gps_state_before_autochange = gps_state;
              if (gps_allow_sleep_while_kiss) {
#ifdef T_BEAM_V1_0
                if (gps_state)
                  axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);                           // switch off GPS
#endif
                gps_state = false;
              }
              dont_send_own_position_packets = true;
              // TODO: there are also tcp kiss devices. Instead of 'kiss_client_came_via_bluetooth', we should mark it in a session struct where we can iterate through
#ifdef ENABLE_BLUETOOTH
              if (SerialBT.hasClient())
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

#if defined(ENABLE_WIFI)
        if (!was_own_position_packet || tx_own_beacon_from_this_device_or_fromKiss__to_aprsis) {
          // No word "NOGATE" or "RFONLY" in header? -> may be sent to aprs-is
          char *q = strstr(data, ",NOGATE");
          if (!q || q > strchr(data, ':')) {
            q = strstr(data, ",RFONLY");
            if (!q || q > strchr(data, ':')) {
              send_to_aprsis(*TNC2DataFrame);
            }
          }
        }
#endif

        if (lora_tx_enabled) {
          if (tx_own_beacon_from_this_device_or_fromKiss__to_frequencies % 2)
            loraSend(txPower, lora_freq, lora_speed, String(data));  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
          if (tx_own_beacon_from_this_device_or_fromKiss__to_frequencies > 1 && lora_digipeating_mode > 1 && lora_freq_cross_digi > 1.0 && lora_freq_cross_digi != lora_freq)
            loraSend(txPower_cross_digi, lora_freq_cross_digi, lora_speed_cross_digi, String(data));  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
          enableOled(); // enable OLED
          writedisplaytext("((KISSTX))","","","","","");
          time_to_refresh = millis() + showRXTime;
        }

out:
        delete TNC2DataFrame;
      }
    }
  #endif // KISS_PROTOCOL

  // sema lock for lora chip operations
#ifdef IF_SEMAS_WOULD_WORK
  if (xSemaphoreTake(sema_lora_chip, 100) == pdTRUE) {
#else
  if (!sema_lora_chip) {
   sema_lora_chip = true;
#endif
   if (rf95.waitAvailableTimeout(100)) {
    #ifdef T_BEAM_V1_0
      #ifdef ENABLE_LED_SIGNALING
        axp.setChgLEDMode(AXP20X_LED_LOW_LEVEL);
      #endif
    #endif
    #ifdef BUZZER
      int melody[] = {300, 50, 500, 100};
      buzzer(melody, sizeof(melody)/sizeof(int));
    #endif

    // we need to read the received packt, even if rx is set to disable. else rf95.waitAvailableTimeout(100) will always show, data is available
    //byte array
    byte  lora_RXBUFF[BG_RF95_MAX_MESSAGE_LEN];      //buffer for packet to send
    uint8_t loraReceivedLength = sizeof(lora_RXBUFF); // (implicit ) reset max length before receiving!
    boolean lora_rx_data_available = rf95.recvAPRS(lora_RXBUFF, &loraReceivedLength);
    // release lock here. We read the data from the lora chip. And we may call later loraSend (which should not be blocked by ourself)
#ifdef IF_SEMAS_WOULD_WORK
    xSemaphoreGive(sema_lora_chip);
#else
    sema_lora_chip = false;
#endif

    const char *rssi_for_path = encode_snr_rssi_in_path();

    // always needed (even if rx is disabled)
    if (lora_freq_rx_curr == lora_freq) {
      time_last_lora_frame_received_on_main_freq = millis();
      lora_packets_received_in_timeslot_on_main_freq++;
    } else {
      lora_packets_received_in_timeslot_on_secondary_freq++;
    }

    if (lora_rx_enabled && lora_rx_data_available) {
        String loraReceivedFrameString;               //data on buff is copied to this string. raw
        String loraReceivedFrameString_for_syslog;    //data on buff is copied to this string. Non-printable characters are shown as <0xnn>. Even valid EOL \r. Syslog is for analyzing.
        String loraReceivedFrameString_for_weblist;   //data on buff is copied to this string. Bad characters like \n and \0 are replaced by ' ' (also valid \r - aprs does not use two line messages ;); \r, \n or \0 at the end of the string are removed.
        char *s = 0;

        for (int i=0 ; i < loraReceivedLength ; i++) {
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

        // valid packet?
        if (!packet_is_valid(received_frame)) {
          goto invalid_packet;
        }

        int blacklisted = is_call_blacklisted(received_frame);
        // don't even automaticaly adapt CR for spammers
        if (blacklisted) {
          goto call_invalid_or_blacklisted;
        }

        //int rssi = rf95.lastSNR();
        //Serial.println(rssi);

        char *header_end = strchr(received_frame, ':');
        char *digipeatedflag = strchr(received_frame, '*');
        if (digipeatedflag && digipeatedflag > header_end)
          digipeatedflag = 0;

        char *q;

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

        // User sends ",Q" in path? He likes at the first digi to add snr/rssi to path
        uint8_t user_demands_trace = ( ((q = strstr(received_frame, ",Q,")) || (q = strstr(received_frame, ",Q:"))) && q < header_end) ? 1 : 0;
        // User sends ",QQ"' in path? He likes all digis to add snr/rssi to path
        if (((q = strstr(received_frame, ",QQ,")) || (q = strstr(received_frame, ",QQ:"))) && q < header_end)
          user_demands_trace = 2;

 #if defined(ENABLE_WIFI)
        if (aprsis_enabled && !our_packet && !blacklisted) {
          // No word "NOGATE" or "RFONLY" in header? -> may be sent to aprs-is
          q = strstr(received_frame, ",NOGATE");
          if (!q || q > header_end) {
            q = strstr(received_frame, ",RFONLY");
            if (!q || q > header_end) {
              s = 0;
              if (((lora_add_snr_rssi_to_path & FLAG_ADD_SNR_RSSI_FOR_APRSIS) || user_demands_trace > 1) ||
                  (!digipeatedflag && ((lora_add_snr_rssi_to_path & FLAG_ADD_SNR_RSSI_FOR_APRSIS__ONLY_IF_HEARD_DIRECT) || user_demands_trace == 1)) )
                s = append_element_to_path(received_frame, rssi_for_path);
                send_to_aprsis(s ? String(s) : loraReceivedFrameString);
            }
          }
        }
#endif

    #ifdef SHOW_RX_PACKET                                                 // only show RX packets when activitated in config
        enableOled(); // enable OLED
        writedisplaytext("  ((RX))", "", loraReceivedFrameString, "", "", "");
        time_to_refresh = millis() + showRXTime;
        #ifdef ENABLE_WIFI
          sendToWebList(loraReceivedFrameString_for_weblist, bg_rf95rssi_to_rssi(rf95.lastRssi()), bg_rf95snr_to_snr(rf95.lastSNR()));
        #endif
    #endif
    #if defined(ENABLE_SYSLOG) && defined(ENABLE_WIFI) // unfortunately, on this plattform we only have IP if we have WIFI
        syslog_log(LOG_INFO, String("Received LoRa: '") + loraReceivedFrameString_for_syslog + "', RSSI:" + bg_rf95rssi_to_rssi(rf95.lastRssi()) + ", SNR: " + bg_rf95snr_to_snr(rf95.lastSNR()));
    #endif
    #ifdef KISS_PROTOCOL
	s = 0;
        if (((lora_add_snr_rssi_to_path & FLAG_ADD_SNR_RSSI_FOR_KISS) || user_demands_trace > 1) ||
            (!digipeatedflag && ((lora_add_snr_rssi_to_path & FLAG_ADD_SNR_RSSI_FOR_KISS__ONLY_IF_HEARD_DIRECT) || user_demands_trace == 1)) )

          s = kiss_add_snr_rssi_to_path_at_position_without_digippeated_flag ? append_element_to_path(received_frame, rssi_for_path) : add_element_to_path(received_frame, rssi_for_path);
        sendToTNC(s ? String(s) : loraReceivedFrameString);
    #endif

        // Are we configured as lora digi? Are we listening on the main frequency?
        if (lora_tx_enabled && lora_digipeating_mode > 0 && !our_packet && !blacklisted && lora_freq_rx_curr == lora_freq) {
          uint32_t time_lora_TXBUFF_for_digipeating_was_filled_prev = time_lora_TXBUFF_for_digipeating_was_filled;
          if (((lora_add_snr_rssi_to_path & FLAG_ADD_SNR_RSSI_FOR_RF) || user_demands_trace > 1) ||
	        (!digipeatedflag && ((lora_add_snr_rssi_to_path & FLAG_ADD_SNR_RSSI_FOR_RF__ONLY_IF_HEARD_DIRECT) || user_demands_trace == 1)) )
            handle_lora_frame_for_lora_digipeating(received_frame, rssi_for_path);
          else
            handle_lora_frame_for_lora_digipeating(received_frame, NULL);
          // new frame in digipeating queue? cross-digi freq enabled and freq set? Send without delay.
          if (*lora_TXBUFF_for_digipeating && lora_cross_digipeating_mode > 0 && lora_freq_cross_digi > 1.0 && lora_freq_cross_digi != lora_freq && time_lora_TXBUFF_for_digipeating_was_filled > time_lora_TXBUFF_for_digipeating_was_filled_prev) {
            // word 'NOGATE' part of the header? Don't gate it
            q = strstr(lora_TXBUFF_for_digipeating, ",NOGATE");
            if (!q || q > strchr(lora_TXBUFF_for_digipeating, ':')) {
              loraSend(txPower_cross_digi, lora_freq_cross_digi, lora_speed_cross_digi, String(lora_TXBUFF_for_digipeating));  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
              enableOled(); // enable OLED
              writedisplaytext("  ((TX cross-digi))", "", String(lora_TXBUFF_for_digipeating), "", "", "");
              time_to_refresh = millis() + showRXTime;
#ifdef KISS_PROTOCOL
              s = add_element_to_path(lora_TXBUFF_for_digipeating, "GATE");
              sendToTNC(s ? String(s) : lora_TXBUFF_for_digipeating);
#endif
            }
          }
        }
    } else {
      // rx disabled. guess blind
     lora_automaic_cr_adoption_rf_transmissions_heard_in_timeslot++;
    }
call_invalid_or_blacklisted:
invalid_packet:
    #ifdef T_BEAM_V1_0
      #ifdef ENABLE_LED_SIGNALING
        axp.setChgLEDMode(AXP20X_LED_OFF);
      #endif
    #else
      ; // make compiler happy
    #endif
   } else {
#ifdef IF_SEMAS_WOULD_WORK
    xSemaphoreGive(sema_lora_chip);
#else
    sema_lora_chip = false;
#endif
   }
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
        rf95.setFrequency(lora_freq_rx_curr);
        lora_speed_rx_curr = (*p_curr_slot_table) ? lora_speed_cross_digi : lora_speed;
        lora_set_speed(lora_speed_rx_curr);
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
  // Send position, if not requested to do not ;) But enter this part if user likes our LA/LON/SPD/CRS to be displayed on his screen ('!gps_allow_sleep_while_kiss' caused 'gps_state false')
  if (!gps_state && (!dont_send_own_position_packets || !lora_tx_enabled))
    goto behind_position_tx;


  average_speed[point_avg_speed] = gps.speed.kmph();   // calculate smart beaconing. Even a not-updated old speed is ok here.
  ++point_avg_speed;
  if (point_avg_speed>4) {
    point_avg_speed=0;
  }

  average_speed_final = (average_speed[0]+average_speed[1]+average_speed[2]+average_speed[3]+average_speed[4])/5;
  if (gps.course.isValid() && gps.course.age() < 10000) {
    average_course[point_avg_course] = gps.course.deg();   // calculate smart beaconing course
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
      // algo is: (int ) ((int )sb_angle + 10 * turn_slope / (int ) mph). In km/h, we have (int ) ((int ) ab_angle + 16 *turn_slope / (int ) mph)
      if (nextTX > 1 && (millis()-lastTX) > (sb_turn_time*1000L) && average_speed_final >= sb_min_speed) {
        // cave: average_speed_final must not be 0 (division by zero). Becuse sb_min_speed may be configured as zero, check above is not enough
        int int_average_speed_final = (int ) average_speed_final;
        if (int_average_speed_final < 1) int_average_speed_final = 1;
        int sb_turn_threshold=min(120, (int ) ((int ) sb_angle + 16 * sb_turn_slope / int_average_speed_final));
        if ((old_course < sb_turn_threshold) && (new_course > (360-sb_turn_threshold))) {
          if (abs(new_course-old_course-360)>=sb_turn_threshold) {
            nextTX = 1;
          }
        } else {
          if ((old_course > (360-sb_turn_threshold)) && (new_course < sb_turn_threshold)) {
            if (abs(new_course-old_course+360)>=sb_turn_threshold) {
              nextTX = 1;
            }
          } else {
            if (abs(new_course-old_course)>=sb_turn_threshold) {
              nextTX = 1;
            }
          }
        }
      }
      old_course = new_course;
    }
  }

  if ((millis()<sb_max_interval)&&(lastTX == 0)) {
    nextTX = 0;
  }

  // No course change (indicator nextTX==1)? Recompute nextTX
  if (nextTX > 1 && millis()-lastTX >= sb_min_interval) {
#ifdef SB_ALGO_KENWOOD
    if (average_speed_final < sb_min_speed)
      nextTX = sb_max_interval;
    else if (average_speed_final > sb_max_speed)
      nextTX = sb_min_interval;
    else {
      nextTX = sb_min_interval * sb_max_speed / average_speed_final;
      if (nextTX > sb_max_interval)
        nextTX = sb_max_interval;
    }
#else
    // dl9sau: imho, too affine at high speed level
    //nextTX = (sb_max_interval-sb_min_interval)/(sb_max_speed-sb_min_speed)*(sb_max_speed-average_speed_final)+sb_min_interval;
    // next computation could become negative (if we are faster than sb_max_speed). fixes: either work with temporary signed long variable, or test if sb_max_speed < average_speed_final.
    nextTX = ((sb_max_speed > average_speed_final) ? ((sb_max_interval-sb_min_interval)/(sb_max_speed-sb_min_speed)*(sb_max_speed-average_speed_final)+sb_min_interval) : sb_min_interval);
    //if (nextTX < sb_min_interval) {nextTX=sb_min_interval;}   // already assured (  (sb_max_speed <= average_speed_final) -> nextTX=sb_min_interval)
    if (nextTX > sb_max_interval) {nextTX=sb_max_interval;}
 #endif
    // now, nextTX is <= sb_min_interval
  }

  // rate limit to 20s in SF12 CR4/5 aka lora_speed 300; 5s in lora_speed 1200 (SF9 CR4/7). -> 1200/lora_speed*5 seconds == 6000000 / lora_speed ms
  // If special case nextTX <= 1: we already enforced rate-limiting (see course computation)
  if (!fixed_beacon_enabled && !dont_send_own_position_packets && lora_tx_enabled && (lastTX+nextTX) < millis() && (nextTX <= 1 || (millis()-lastTX) >= (6000000L / lora_speed ))) {
    if (gps_isValid) {
      enableOled(); // enable OLED
      //writedisplaytext(" ((TX))","","LAT: "+LatShownP,"LON: "+LongShownP,"SPD: "+String(gps.speed.kmph(),1)+"  CRS: "+String(gps.course.deg(),1),getSatAndBatInfo());
      fillDisplayLine1();
      fillDisplayLine2();
      fillDisplayLines3to5();
      writedisplaytext("  ((TX))","",OledLine2,OledLine3,OledLine4,OledLine5);
      sendpacket(SP_POS_GPS | (nextTX == 1 ? SP_ENFORCE_COURSE : 0));
      // for fixed beacon (if we loose gps fix, we'll send our last position in fix_beacon_interval)
      next_fixed_beacon = millis() + fix_beacon_interval;
      t_last_smart_beacon_sent = millis();
      // We just transmitted. We transmitted due to turn? Don't TX again in next round:
      if (nextTX < sb_min_interval) nextTX = sb_min_interval;
    } else {
      if (millis() > time_to_refresh){
        displayInvalidGPS();
      }
    }
    display_was_updated = true;
  }

behind_position_tx:

  if (!display_was_updated) {
    if (millis() > time_to_refresh){
      if (gps_isValid) {
        OledHdr = Tcall;
        fillDisplayLine1();
        fillDisplayLine2();
        fillDisplayLines3to5();
        writedisplaytext(OledHdr,OledLine1,OledLine2,OledLine3,OledLine4,OledLine5);
      } else {
        displayInvalidGPS();
      }
    } else {
// refresh  time 
      fillDisplayLine1();
    }
  }


  #if defined(ENABLE_TNC_SELF_TELEMETRY) && defined(KISS_PROTOCOL)
    if (nextTelemetryFrame < millis()){
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
        #ifdef T_BEAM_V1_0
          debug_message += "Bat V: " + String(axp.getBattVoltage());
          debug_message += ", ";
          debug_message += "Bat IN A: " + String(axp.getBattChargeCurrent());
          debug_message += ", ";
          debug_message += "Bat OUT A: " + String(axp.getBattDischargeCurrent());
          debug_message += ", ";
          debug_message += "USB Plugged: " + String(axp.isVBUSPlug());
          debug_message += ", ";
          debug_message += "USB V: " + String(axp.getVbusVoltage());
          debug_message += ", ";
          debug_message += "USB A: " + String(axp.getVbusCurrent());
          debug_message += ", ";
          debug_message += "Temp C: " + String(axp.getTemp());
        #else
          debug_message += "Bat V: " + String(BattVolts);
        #endif

        Serial.print(encapsulateKISS(debug_message, CMD_HARDWARE));
        #ifdef ENABLE_BLUETOOTH
          SerialBT.print(encapsulateKISS(debug_message, CMD_HARDWARE));
        #endif
      }
    #endif
  #endif


  // Data for digipeating in queue?
  if (lora_tx_enabled && lora_rx_enabled && lora_digipeating_mode && *lora_TXBUFF_for_digipeating) {
    // 5s grace time (plus up to 250ms random) for digipeating. 10s if we are a fill-in digi
    if ((time_lora_TXBUFF_for_digipeating_was_filled + 5*lora_digipeating_mode*1000L + (millis() % 250)) < millis()) {
      if (lora_cross_digipeating_mode < 2 && (time_lora_TXBUFF_for_digipeating_was_filled + 2* 5*lora_digipeating_mode*1000L) > millis()) {
        // if SF12: we degipeat in fastest mode CR4/5. -> if lora_speed < 300 tx in lora_speed_300.
        loraSend(txPower, lora_freq, (lora_speed < 300) ? 300 : lora_speed, String(lora_TXBUFF_for_digipeating));  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
        writedisplaytext("  ((TX digi))", "", String(lora_TXBUFF_for_digipeating), "", "", "");
#ifdef KISS_PROTOCOL
        sendToTNC(String(lora_TXBUFF_for_digipeating));
#endif
      } // else: too late. skip TX
      *lora_TXBUFF_for_digipeating = 0;
    }
  }

  vTaskDelay(1);
}
