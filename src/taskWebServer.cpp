#include <list>
#include "taskWebServer.h"
#include "preference_storage.h"
#include "syslog_log.h"
//#include "PSRAMJsonDocument.h"
#include <time.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include "taskGPS.h"

#ifdef ENABLE_WIFI

extern int debug_verbose;

/**
 * @see board_build.embed_txtfiles in platformio.ini
 */
extern const char web_index_html[] asm("_binary_data_embed_index_html_out_start");
extern const char web_index_html_end[] asm("_binary_data_embed_index_html_out_end");
extern const char web_style_css[] asm("_binary_data_embed_style_css_out_start");
extern const char web_style_css_end[] asm("_binary_data_embed_style_css_out_end");
extern const char web_js_js[] asm("_binary_data_embed_js_js_out_start");
extern const char web_js_js_end[] asm("_binary_data_embed_js_js_out_end");

bool getLocalTimeTheBetterWay(struct tm * info);

// Variable needed to send beacon from html page
extern uint8_t manBeacon;

// KISS-friendly serial logging
extern void do_serial_println(const String &);

// Variable to show AP status
extern uint8_t WIFI_DISABLED;
extern uint8_t WIFI_SEARCHING_FOR_AP;
extern uint8_t WIFI_CONNECTED_TO_AP;
extern uint8_t WIFI_NOT_CONNECTED_TO_AP;
extern int8_t WIFI_RUNNING_AS_AP;
extern int8_t wifi_connection_status;

extern String wifi_ModeAP_SSID;
extern String wifi_ModeAP_PASS;
String wifi_ModeAP_PASS_default = "xxxxxxxxxx";
// AP Array, currently max 9 APs possible (plus 1 with SSID Self-AP with password)
#define MAX_AP_CNT 10                  // max number of possible APs
extern struct AccessPoint APs[MAX_AP_CNT];
extern int apcnt;

// for displaying wifi status
extern String oled_wifi_SSID_curr;
extern String oled_wifi_PASS_curr;
extern String oled_wifi_IP_curr;

extern int8_t wifi_txpwr_mode_AP;
extern int8_t wifi_txpwr_mode_STA;
String dnsHostname;

extern bool tncServer_enabled;
extern bool gpsServer_enabled;
extern bool gps_state;


extern boolean wifi_do_fallback_to_mode_AP;
extern String buildnr;

// For APRS-IS connection
extern String to_aprsis_data;
extern boolean aprsis_enabled;
extern String aprsis_host;
extern uint16_t aprsis_port;
extern String aprsis_own_filters_in;
extern boolean aprsis_own_filter_in_is_whitelist;
extern String aprsis_own_filters_words_in;
extern String aprsis_filter;
extern String aprsis_callsign;
extern String aprsis_password;
extern uint8_t aprsis_data_allow_inet_to_rf;
extern String MY_APRS_DEST_IDENTIFYER;
extern int lora_digipeating_mode;
extern int tx_own_beacon_from_this_device_or_fromKiss__to_frequencies;

extern uint8_t usb_serial_data_type;
extern double lora_freq_rx_curr;
extern ulong lora_speed_rx_curr;
extern boolean lora_tx_enabled;
extern boolean lora_rx_enabled;

extern void sema_lock_or_unlock__lora(int);

extern String aprsLatPreset;
extern String aprsLonPreset;
extern String aprsLatPresetNiceNotation;
extern String aprsLonPresetNiceNotation;
extern String aprsPresetShown;
extern int oled_line3and4_format;
// last line of display, Sat & Batt Info
extern String OledLine1;
extern String OledLine2;
extern String OledLine3;
extern String OledLine4;
extern String OledLine5;

extern char compile_flags[];

extern volatile boolean flag_lora_packet_available;

extern boolean enable_bluetooth;

QueueHandle_t webListReceivedQueue = nullptr;
std::list <tReceivedPacketData*> receivedPackets;
const int MAX_RECEIVED_LIST_SIZE = 50;

// last i/o to aprsis-connection (needed for anti-idle timer)
uint32_t t_aprsis_lastRXorTX = 0L;

extern void do_shutdown(boolean, const String &);

// needed for aprsis igate functionality
String aprsis_status = "Disconnected";
// aprsis 3rd party traffic encoding
String generate_third_party_packet(String, String);
void do_send_status_message_about_reboot_to_aprsis();
extern String handle_aprs_messsage_addressed_to_us(const char *);
void do_send_status_message_about_shutdown_to_aprsis();
#ifdef KISS_PROTOCOL
extern void sendToTNC(const String &);
#endif
extern uint8_t txPower;
extern double lora_freq;
extern ulong lora_speed;
extern uint8_t txPower_cross_digi;
extern ulong lora_speed_cross_digi;
extern double lora_freq_cross_digi;
extern void loraSend(byte, float, ulong, uint8_t , const String &);
extern void load_preferences_from_flash(void);
extern void setup_phase2_soft_reconfiguration(boolean);
extern int is_call_blacklisted(const char *);
extern void lora_set_speed(ulong lora_speed);

extern int save_to_file(const String &, const char *, const String &);

IPAddress IP_NULL(0,0,0,0);
IPAddress IP_SUBNET_NULL(0,0,0,0);
IPAddress IP_GATEWAY_NULL(0,0,0,0);


// APRS-IS status message (about connection reset or our rboot)
extern boolean send_status_message_to_aprsis;
extern boolean send_status_message_about_shutdown_to_rf;
String aprsis_time_last_successful_connect = "";
int aprsis_connect_tries = 0;
extern char gps_time_s[];
String aprs_callsign;

// statistics

extern uint32_t stats_rx_freq_main;
extern uint32_t stats_rx_freq_secondary;
extern uint32_t stats_tx_freq_main;
extern uint32_t stats_tx_freq_secondary;
extern uint32_t stats_tx_beacon;
extern uint32_t stats_tx_status;
extern uint32_t stats_tx_telemetry;
extern uint32_t stats_digipeated_rf2rf;
extern uint32_t stats_from_aprsis;
extern uint32_t stats_to_aprsis;
extern uint32_t stats_from_serial;
extern uint32_t stats_to_serial;
extern uint32_t stats_from_kiss;
extern uint32_t stats_to_kiss;

// keep config stored in this global variable
extern String preferences_as_jsonData;
extern String wifi_config_as_jsonData;

WebServer server(80);
#ifdef KISS_PROTOCOL
  WiFiServer tncServer(NETWORK_TNC_PORT);
#endif
WiFiServer gpsServer(NETWORK_GPS_PORT);

// APRSIS connection
WiFiClient aprsis_client;

#ifdef ENABLE_SYSLOG
  // A UDP instance to let us send and receive packets over UDP
  WiFiUDP udpClient;

  // Create a new empty syslog instance
  Syslog syslog(udpClient, SYSLOG_PROTO_IETF);
#endif

#ifdef T_BEAM_V1_2
#define XPOWERS_CHIP_AXP2101
  #include <XPowersLib.h>
  extern XPowersAXP2101 axp;
#elif T_BEAM_V1_0
  #include <axp20x.h>
  extern AXP20X_Class axp;
#endif


void send_queue_to_aprsis();

void sendCacheHeader() { server.sendHeader("Cache-Control", "max-age=3600"); }
void sendGzipHeader() { server.sendHeader("Content-Encoding", "gzip"); }


String htmlFreetextEscape(const char *s) {
    const char *p = s;
    String s_out;

    for (; *p; p++) {
      char buf[7] = ""; // room for "<0x01>" + \0 == 7
      switch (*p) {
      case '\"':
        strcpy(buf, "&quot;");
        break;
      case '<':
        strcpy(buf, "&lt;");
        break;
      case '>':
        strcpy(buf, "&gt;");
        break;
      case '&':
        strcpy(buf, "&amp;");
        break;
      case ' ':
        strcpy(buf, "&nbsp;");
        break;
      default:
        if (*p < 0x20 || *p == '\x7f') {
          sprintf(buf, "<0x%2.2x>", *p);
        } else {
          sprintf(buf, "%c", *p);
        }
      }
      s_out += String(buf);
    }
    return s_out;
}

String jsonEscape(const char *s){
    const char *p = s;
    String s_out;

    for (; *p; p++) {
      char buf[3] = ""; // room for 2x'\' + \0 == 7
      switch (*p) {
      case '\\':
        strcpy(buf, "\\\\");
        break;
      case '\"':
        strcpy(buf, "\\\"");
        break;
      default:
        sprintf(buf, "%c", *p);
      }
      s_out += String(buf);
    }
    return s_out;
}

String jsonLineFromPreferenceString(const char *preferenceName, bool last=false){
  return String("\"") + preferenceName + "\":\"" + jsonEscape(preferences.getString(preferenceName, "").c_str()) + (last ?  + R"(")" :  + R"(",)");
}
String jsonLineFromPreferenceBool(const char *preferenceName, bool last=false){
  return String("\"") + preferenceName + "\":" + (preferences.getBool(preferenceName) ? "true" : "false") + (last ?  + R"()" :  + R"(,)");
}
String jsonLineFromPreferenceInt(const char *preferenceName, bool last=false){
  return String("\"") + preferenceName + "\":" + (preferences.getInt(preferenceName)) + (last ?  + R"()" :  + R"(,)");
}
String jsonLineFromPreferenceDouble(const char *preferenceName, bool last=false){
    return String("\"") + preferenceName + "\":" + String(preferences.getDouble(preferenceName),4) + (last ?  + R"()" :  + R"(,)");
}
String jsonLineFromString(const char *name, const char *value, bool last=false){
  return String("\"") + name + "\":\"" + jsonEscape(value) + "\"" + (last ?  + R"()" :  + R"(,)");
}
String jsonLineFromInt(const char *name, const int value, bool last=false){
  return String("\"") + name + "\":" + String(value) + (last ?  + R"()" :  + R"(,)");
}
String jsonLineFromDouble(const char *name, const double value, bool last=false){
  return String("\"") + name + "\":" + String(value, 4) + (last ?  + R"()" :  + R"(,)");
}

void handle_NotFound(){
  sendCacheHeader();
  server.send(404, "text/plain", "Not found");
}

void handle_Index() {
  sendGzipHeader();
  server.send_P(200, "text/html", web_index_html, web_index_html_end - web_index_html);
}

void handle_Style() {
  sendCacheHeader();
  sendGzipHeader();
  server.send_P(200, "text/css", web_style_css, web_style_css_end - web_style_css);
}

void handle_Js() {
  sendCacheHeader();
  sendGzipHeader();
  server.send_P(200, "text/javascript", web_js_js, web_js_js_end-web_js_js);
}

void handle_ScanWifi() {
  #if defined(ENABLE_SYSLOG)
    syslog_log(LOG_INFO, String("WebServer: handle_ScanWifi()"));
  #endif
  do_serial_println("WebServer: WebServer: handle_ScanWifi()");
  String listResponse = R"(<label for="networks_found_list">Networks found:</label><select class="u-full-width" id="networks_found_list">)";
  int n = WiFi.scanNetworks();
  listResponse += "<option value=\"\">Select Network</option>";

  for (int i = 0; i < n; ++i) {
    listResponse += "<option value=\""+WiFi.SSID(i)+"\">" + WiFi.SSID(i) + "</option>";
  }
  listResponse += "</select>";
  server.send(200,"text/html", listResponse);
}

void handle_SaveWifiCfg() {
  String s = "";
  #if defined(ENABLE_SYSLOG)
    syslog_log(LOG_INFO, String("WebServer: handle_SaveWifiCfg()"));
  #endif
  do_serial_println("WebServer: WebServer: handle_WifiCfg()");

  if (!server.hasArg(PREF_WIFI_SSID) || !server.hasArg(PREF_AP_PASSWORD)){
    server.send(500, "text/plain", "Invalid request, make sure all fields are set");
    return;
  }

  // Update SSID
  preferences.putString(PREF_WIFI_SSID, server.arg(PREF_WIFI_SSID));
  do_serial_println("WiFi: Updated remote SSID: " + server.arg(PREF_WIFI_SSID));

  s = server.arg(PREF_WIFI_PASSWORD);
  if (s.length() == 0 && server.arg(PREF_WIFI_SSID) == "") {
    // user has deleted both, WIFI_SSID and WIFI_PASS
    preferences.putString(PREF_WIFI_PASSWORD, "");
    do_serial_println("WiFi: Deconfigured your remote AP (SSID and password cleared)");
  } else if (s != "*") {
    if (s != "" /* empty password is "open unencrypted wifi" */ &&
        (s.length() < 8 || s.length() > 63) ) {
      preferences.putString(PREF_WIFI_SSID, server.arg(PREF_WIFI_SSID));
      server.send(403, "text/plain", "WiFi Password must be minimum 8 characters, and max 63: leave empty if you need to connect to an open (unencrypted) AP");
      return;
    } else {
      // Update WiFi password
      preferences.putString(PREF_WIFI_PASSWORD, s);
      do_serial_println("WiFi: Updated remote PASS: " + s);
    }
  }
  if (server.hasArg(PREF_WIFI_TXPWR_MODE_STA)) {
    // Web chooser min, low, mid, high, max
    // We'll use "min", "low", "mid", "high", "max" -> 2dBm (1.5mW) -> 8, 11dBm (12mW) -> 44, 15dBm (32mW) -> 60, 18dBm (63mW) ->72, 20dBm (100mW) ->80
    int8_t choosed = server.arg(PREF_WIFI_TXPWR_MODE_STA).toInt();
    if (choosed < 0) choosed = 8;
    else if (choosed > 84) choosed = 84;
    preferences.putInt(PREF_WIFI_TXPWR_MODE_STA, choosed);
  }

  // Mode AP:
  s = server.arg(PREF_AP_PASSWORD);
  if (s.length() == 0) {
    // user has deleted both, WIFI_SSID and WIFI_PASS. Show ownly once
    if (preferences.getString(PREF_AP_PASSWORD) != "") {
      preferences.putString(PREF_AP_PASSWORD, "");
      do_serial_println("WiFi: your AP password is now factory default: " + wifi_ModeAP_PASS_default);
    }
  } else if (s != "*") {
    if (s.length() < 8 || s.length() > 63) {
      server.send(403, "text/plain", "AP Password must be minimum 8 characters, and max 63.");
      return;
    } else {
      // Update AP password
      preferences.putString(PREF_AP_PASSWORD, s);
      do_serial_println("WiFi: Updated local AP PASS: " + s);
    }
  }
  preferences.putBool(PREF_WIFI_STA_ALLOW_FAILBACK_TO_MODE_AP_AFTER_ONCE_CONNECTED,  server.hasArg(PREF_WIFI_STA_ALLOW_FAILBACK_TO_MODE_AP_AFTER_ONCE_CONNECTED));
  if (server.hasArg(PREF_WIFI_TXPWR_MODE_AP)) {
    // Web chooser min, low, mid, high, max
    // We'll use "min", "low", "mid", "high", "max" -> 2dBm (1.5mW) -> 8, 11dBm (12mW) -> 44, 15dBm (32mW) -> 60, 18dBm (63mW) ->72, 20dBm (100mW) ->80
    int8_t choosed = server.arg(PREF_WIFI_TXPWR_MODE_AP).toInt();
    if (choosed < 0) choosed = 8;
    else if (choosed > 84) choosed = 84;
    preferences.putInt(PREF_WIFI_TXPWR_MODE_AP, choosed);
  }

  if (server.hasArg(PREF_WIFI_ENABLE))
    preferences.putInt(PREF_WIFI_ENABLE, server.arg(PREF_WIFI_ENABLE).toInt());

  preferences.putBool(PREF_TNCSERVER_ENABLE, server.hasArg(PREF_TNCSERVER_ENABLE));
  preferences.putBool(PREF_GPSSERVER_ENABLE, server.hasArg(PREF_GPSSERVER_ENABLE));
  s = "";
  if (server.hasArg(PREF_NTP_SERVER)) {
    s = server.arg(PREF_NTP_SERVER);
    s.trim();
    preferences.putString(PREF_NTP_SERVER, s);
  }
  if (server.hasArg(PREF_SYSLOG_SERVER)) {
    s = server.arg(PREF_SYSLOG_SERVER);
    s.trim();
    preferences.putString(PREF_SYSLOG_SERVER, s);
  }

  // runtime reconfiguration with changed settings
  load_preferences_from_flash();
  setup_phase2_soft_reconfiguration(1);

  server.sendHeader("Location", "/");
  server.send(302,"text/html", "");
}

void handle_Reboot() {
  #if defined(ENABLE_SYSLOG)
    syslog_log(LOG_WARNING, String("WebServer: Reboot Request -> Rebooting..."));
  #endif
  Serial.println("WebServer: Reboot Request -> Rebooting...");
  do_send_status_message_about_reboot_to_aprsis();
  server.sendHeader("Location", "/");
  server.send(302,"text/html", "");
  server.close();
  ESP.restart();
}

void handle_Beacon() {
  #if defined(ENABLE_SYSLOG)
    syslog_log(LOG_INFO, String("WebServer: Button manual beacon pressed."));
  #endif
  do_serial_println("WebServer: WebServer: Button manual beacon pressed.");
  server.sendHeader("Location", "/");
  server.send(302,"text/html", "");
  manBeacon=1;
}

void handle_Shutdown() {
  #if defined(ENABLE_SYSLOG)
    syslog_log(LOG_WARNING, String("WebServer: Shutdown Request -> Shutdown..."));
  #endif
  Serial.println("WebServer: Shutdown Request -> Shutdown...");
  server.send(200,"text/html", "Shutdown");
  do_shutdown(true, "");
}

void handle_Restore() {
  #if defined(ENABLE_SYSLOG)
    syslog_log(LOG_WARNING, String("WebServer: Reset Reqeust -> Reseting preferences and rebooting..."));
  #endif
  Serial.println("WebServer: Reset Requet -> Reseting preferences and rebooting...");
  do_send_status_message_about_reboot_to_aprsis();
  server.sendHeader("Location", "/");
  server.send(302,"text/html", "");
  preferences.clear();
  preferences.end();
  ESP.restart();
}


void refill_preferences_as_jsonData()
{
  String s_tmp;
  String s = "{";
  s = s + "\n  " + String("\"") + PREF_WIFI_PASSWORD + "\": \"" + jsonEscape((preferences.getString(PREF_WIFI_PASSWORD, "").isEmpty() ? "" : "*")) + R"(",)";
  s = s + "\n  " +  String("\"") + PREF_AP_PASSWORD + "\": \"" + jsonEscape((preferences.getString(PREF_AP_PASSWORD, "").isEmpty() ? "" : "*")) + R"(",)";
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_WIFI_ENABLE);
  s = s + "\n  " +  jsonLineFromPreferenceString(PREF_WIFI_SSID);
  s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_WIFI_STA_ALLOW_FAILBACK_TO_MODE_AP_AFTER_ONCE_CONNECTED);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_WIFI_TXPWR_MODE_AP);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_WIFI_TXPWR_MODE_STA);
  s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_TNCSERVER_ENABLE);
  s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_GPSSERVER_ENABLE);
  s = s + "\n  " +  jsonLineFromPreferenceString(PREF_NTP_SERVER);
  s = s + "\n  " +  jsonLineFromPreferenceString(PREF_SYSLOG_SERVER);
  s = s + "\n  " +  jsonLineFromPreferenceDouble(PREF_LORA_FREQ_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_LORA_SPEED_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_LORA_RX_ENABLE);
  s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_LORA_TX_ENABLE);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_LORA_TX_POWER);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_LORA_TX_PREAMBLE_LEN);
  s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_LORA_AUTOMATIC_CR_ADAPTION_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_LORA_ADD_SNR_RSSI_TO_PATH_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_LORA_ADD_SNR_RSSI_TO_PATH_END_AT_KISS_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_APRS_DIGIPEATING_MODE_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceString(PREF_APRS_DIGIPEATING_MYALIAS);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_APRS_CROSS_DIGIPEATING_MODE_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_LORA_TX_BEACON_AND_KISS_TO_FREQUENCIES_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_LORA_TX_BEACON_AND_KISS_TO_APRSIS_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_LORA_TX_STATUSMESSAGE_TO_APRSIS_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceDouble(PREF_LORA_FREQ_CROSSDIGI_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_LORA_SPEED_CROSSDIGI_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_LORA_TX_POWER_CROSSDIGI_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_LORA_RX_ON_FREQUENCIES_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceString(PREF_APRS_CALLSIGN);
  s = s + "\n  " +  jsonLineFromPreferenceString(PREF_APRS_RELAY_PATH);
  s = s + "\n  " +  jsonLineFromPreferenceString(PREF_APRS_SYMBOL_TABLE);
  s = s + "\n  " +  jsonLineFromPreferenceString(PREF_APRS_SYMBOL);
  s = s + "\n  " +  jsonLineFromPreferenceString(PREF_APRS_OBJECT_NAME);
  s = s + "\n  " +  jsonLineFromPreferenceString(PREF_APRS_COMMENT);
  s = s + "\n  " +  jsonLineFromPreferenceString(PREF_APRS_LATITUDE_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceString(PREF_APRS_LONGITUDE_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_APRS_POSITION_AMBIGUITY);
  s = s + "\n  " +  jsonLineFromPreferenceString(PREF_APRS_SENDER_BLACKLIST);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_APRS_SB_MIN_INTERVAL_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_APRS_SB_MAX_INTERVAL_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_APRS_SB_MIN_SPEED_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_APRS_SB_MAX_SPEED_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceDouble(PREF_APRS_SB_ANGLE_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_APRS_SB_TURN_SLOPE_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_APRS_SB_TURN_TIME_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_APRS_SHOW_BATTERY);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_APRS_STATUS_WINLINK_NOTIFICATION);
  s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_APRS_STATUS_SHUTDOWN_NOTIFICATION);
  s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_APRS_FIXED_BEACON_PRESET);
  //s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_APRS_SHOW_ALTITUDE);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_APRS_ALTITUDE_RATIO);
  s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_APRS_ALWAYS_SEND_CSE_SPEED_AND_ALTITUDE);
  s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_APRS_GPS_EN);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_GPS_POWERSAVE);
  s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_ACCEPT_OWN_POSITION_REPORTS_VIA_KISS);
  s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_GPS_ALLOW_SLEEP_WHILE_KISS);
  s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_ENABLE_TNC_SELF_TELEMETRY);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_TNC_SELF_TELEMETRY_INTERVAL);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_TNC_SELF_TELEMETRY_MIC);
  s = s + "\n  " +  jsonLineFromPreferenceString(PREF_TNC_SELF_TELEMETRY_PATH);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_TNC_SELF_TELEMETRY_ALLOW_RF);
  s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_DEV_OL_EN);
  s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_APRS_SHOW_CMT);
  s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_APRS_COMMENT_RATELIMIT_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_DEV_BT_EN);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_DEV_USBSERIAL_DATA_TYPE);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_DEV_SHOW_RX_TIME);
  s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_DEV_AUTO_SHUT);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_DEV_AUTO_SHUT_PRESET);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_DEV_REBOOT_INTERVAL);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_DEV_SHOW_OLED_TIME);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_DEV_CPU_FREQ);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_DEV_UNITS);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_DEV_OLED_L3_L4_FORMAT);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_DEV_OLED_LOCATOR);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_DEV_OLED_LOCATOR_AMBIGUITY);
  s = s + "\n  " +  jsonLineFromPreferenceBool(PREF_APRSIS_EN);
  s = s + "\n  " +  jsonLineFromPreferenceString(PREF_APRSIS_SERVER_NAME);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_APRSIS_SERVER_PORT);
  s = s + "\n  " +  jsonLineFromPreferenceString(PREF_APRSIS_FILTER);
  s = s + "\n  " +  jsonLineFromPreferenceString(PREF_APRSIS_FILTER_LOCAL_INCOMING);
  s = s + "\n  " +  jsonLineFromPreferenceString(PREF_APRSIS_FILTER_LOCAL_WORDS_INCOMING);
  s = s + "\n  " +  jsonLineFromPreferenceString(PREF_APRSIS_CALLSIGN);
  s = s + "\n  " +  jsonLineFromPreferenceString(PREF_APRSIS_PASSWORD);
  s = s + "\n  " +  jsonLineFromPreferenceInt(PREF_APRSIS_ALLOW_INET_TO_RF);
  s = s + "\n  " +  jsonLineFromDouble("lora_freq_rx_curr", lora_freq_rx_curr);
  s = s + "\n  " +  jsonLineFromString("aprsis_status", aprsis_status.c_str());
  s = s + "\n  " +  jsonLineFromInt("FreeHeap", ESP.getFreeHeap());
  s = s + "\n  " +  jsonLineFromInt("HeapSize", ESP.getHeapSize());
  s = s + "\n  " +  jsonLineFromInt("FreeSketchSpace", ESP.getFreeSketchSpace());
  s = s + "\n  " +  jsonLineFromInt("PSRAMSize", ESP.getPsramSize());
  s = s + "\n  " +  jsonLineFromInt("PSRAMFree", ESP.getFreePsram());
  if (oled_line3and4_format == 0) {
    s_tmp = aprsLatPreset + " " + aprsLonPreset + " [" + (aprsPresetShown == "" ? "GPS" : aprsPresetShown) + "]";
  } else {
    s_tmp = aprsLatPresetNiceNotation + " " + aprsLonPresetNiceNotation + " [" + (aprsPresetShown == "" ? "GPS" : aprsPresetShown) + "]";
  }
  s = s + "\n  " +  jsonLineFromString("curPos", s_tmp.c_str());
  s = s + "\n  " +  jsonLineFromInt("UptimeMinutes", millis()/1000/60);
  s = s + "\n  " +  jsonLineFromString("OledLine1", OledLine1.c_str());
  s = s + "\n  " +  jsonLineFromString("OledLine2", OledLine2.c_str());
  s_tmp = String(OledLine3); s_tmp.replace("\xF7", "°");
  s = s + "\n  " +  jsonLineFromString("OledLine3", s_tmp.c_str());
  s_tmp = String(OledLine4); s_tmp.replace("\xF7", "°");
  s = s + "\n  " +  jsonLineFromString("OledLine4", s_tmp.c_str());
  s = s + "\n  " +  jsonLineFromString("OledLine5", OledLine5.c_str());
  s = s + "\n  " +  jsonLineFromInt("stats_rx_freq_main", stats_rx_freq_main);
  s = s + "\n  " +  jsonLineFromInt("stats_rx_freq_secondary", stats_rx_freq_secondary);
  s = s + "\n  " +  jsonLineFromInt("stats_tx_freq_main", stats_tx_freq_main);
  s = s + "\n  " +  jsonLineFromInt("stats_tx_freq_secondary", stats_tx_freq_secondary);
  s = s + "\n  " +  jsonLineFromInt("stats_tx_beacon", stats_tx_beacon);
  s = s + "\n  " +  jsonLineFromInt("stats_tx_status", stats_tx_status);
  s = s + "\n  " +  jsonLineFromInt("stats_tx_telemetry", stats_tx_telemetry);
  s = s + "\n  " +  jsonLineFromInt("stats_digipeated_rf2rf", stats_digipeated_rf2rf);
  s = s + "\n  " +  jsonLineFromInt("stats_from_aprsis", stats_from_aprsis);
  s = s + "\n  " +  jsonLineFromInt("stats_to_aprsis", stats_to_aprsis);
  s = s + "\n  " +  jsonLineFromInt("stats_from_serial", stats_from_serial);
  s = s + "\n  " +  jsonLineFromInt("stats_to_serial", stats_to_serial);
  s = s + "\n  " +  jsonLineFromInt("stats_from_kiss", stats_from_kiss);
  s = s + "\n  " +  jsonLineFromInt("stats_to_kiss", stats_to_kiss);
  s = s + "\n  " +  jsonLineFromString("CompileFlags", compile_flags, true);

  s += "\n}\n";
  // Store copy of jsonData in our global variable
  preferences_as_jsonData = String(s);

}

void handle_saveCfg2FS() {
  #if defined(ENABLE_SYSLOG)
    syslog_log(LOG_INFO, String("WebServer: Button save Config to Filesystem pressed."));
    do_serial_println("WebServer: Button save Config to Filesystem pressed.");
  #endif

  refill_preferences_as_jsonData();
  if (!preferences_as_jsonData.isEmpty()) {
    // Launches SPIFFS file system
    save_to_file("Webserver", "/preferences.cfg", preferences_as_jsonData);
  }

  server.sendHeader("Location", "/");
  server.send(302,"text/html", "");
}

void handle_Cfg() {
  refill_preferences_as_jsonData();
  server.send(200,"application/json", preferences_as_jsonData);
}

void fill_wifi_config_as_jsonData() {
  int pos;
  String s = "{\n  \"AP\": [\n";
  if (apcnt) {
    for (pos = 0 ; pos < apcnt; pos++) {
      s += "    {\n      ";
      s = s + "\"SSID\": \"" + jsonEscape(APs[pos].ssid) + "\"";
      s += ",\n      ";
      s = s + "\"password\": \"" + jsonEscape(APs[pos].pw) + "\"";
      if (!pos) {
        s += ",\n      ";
        s = s + "\"prio\": 1";
      }
      s += "\n    }";
      if (pos < apcnt-1)
        s += ",";
      s += "\n";
    }
  } else {
    s += "    {\n      ";
    s = s + "\"SSID\": \"" + jsonEscape(preferences.getString(PREF_WIFI_PASSWORD, "").c_str()) + "\"";
    s += ",\n      ";
    s = s + "\"password\": \"" + jsonEscape(preferences.getString(PREF_WIFI_PASSWORD, "").c_str()) + "\"";
    s += "\n    }\n";
  }
  s += "  ],\n\n";
  s = s + "  \"SelfAP_PW\": \"" + jsonEscape(preferences.getString(PREF_AP_PASSWORD, "").c_str()) + "\"";

  s += "\n}\n";

  wifi_config_as_jsonData = String(s);
}

void handle_saveWifi2FS() {
  #if defined(ENABLE_SYSLOG)
    syslog_log(LOG_INFO, String("WebServer: Button save WifiConfig to Filesystem pressed."));
    do_serial_println("WebServer: Button save WifiConfig to Filesystem pressed.");
  #endif

  fill_wifi_config_as_jsonData();
  if (!wifi_config_as_jsonData.isEmpty()) {
    // Launches SPIFFS file system
    save_to_file("Webserver", "/wifi.cfg", wifi_config_as_jsonData);
  }

  server.sendHeader("Location", "/");
  server.send(302,"text/html", "");
}

void handle_WifiCfg() {
  fill_wifi_config_as_jsonData();
  server.send(200,"application/json", wifi_config_as_jsonData);
}

void handle_ReceivedList() {
  // DynamicJsonDocument: heap size problem? *500 is too small (json array will be truncated if received list tends tot 50 entries;
  // *1000 results in '{}' on TTGO t-beam (while it's correct on TTGO lora32)
  // -> Better not rely on complex functions that may not work as expected. We already have that simple and
  // -> ifdef notdef ;)
#ifdef notdef
  //PSRAMJsonDocument doc(MAX_RECEIVED_LIST_SIZE * 1000);
  DynamicJsonDocument doc(MAX_RECEIVED_LIST_SIZE * 1000);
  JsonObject root = doc.to<JsonObject>();
  auto received = root.createNestedArray("received");
  for (auto element: receivedPackets){
    char buf[64];
    strftime(buf, 64, "%Y-%m-%d %H:%M:%S", &element->rxTime);
    auto packet_data = received.createNestedObject();
    packet_data["time"] = String(buf);
    packet_data["packet"] = htmlFreetextEscape(element->packet->c_str());
    packet_data["rssi"] = element->RSSI;
    packet_data["snr"] = element->SNR;
    packet_data["qrg"] = (element->QRG == 'M') ? "M" : "S";
  }

  server.send(200,"application/json", doc.as<String>());

#else

  String jsonData = "{\"received\":[";
  boolean first_entry = true;
  for (auto element: receivedPackets) {
    char buf[64];
    strftime(buf, 64, "%Y-%m-%d %H:%M:%S", &element->rxTime);
    if (first_entry) {
      jsonData += "{";
      first_entry = false;
    } else {
      jsonData += ",{";
    }
    jsonData += jsonLineFromString("time", buf);
    jsonData += jsonLineFromString("packet", htmlFreetextEscape(element->packet->c_str()).c_str());
    jsonData += jsonLineFromInt("rssi", element->RSSI);
    jsonData += jsonLineFromInt("snr", element->SNR);
    jsonData += jsonLineFromString("qrg", (element->QRG == 'M') ? "M" : "S", true);
    jsonData += "}";
  }
  jsonData += "]}";

  server.send(200,"application/json", jsonData);

#endif
}


void store_lat_long(float f_lat, float f_long) {
  char buf[13];

  int i_deg = (int ) (f_lat < 0 ? (f_lat*-1) : f_lat);
  float f_min = ((f_lat < 0 ? f_lat*-1 : f_lat) -((int ) i_deg))*60.0;
  if (f_min > 59.99995) { f_min = 0.0; i_deg++; };
  if (i_deg >= 180) { i_deg = 179; f_min = 59.9999; };
  sprintf(buf, "%2.2d-%07.4f%c", i_deg, f_min, f_lat < 0 ? 'S' :  'N');
  preferences.putString(PREF_APRS_LATITUDE_PRESET, String(buf));

  i_deg = (int ) (f_long < 0 ? (f_long*-1) : f_long);
  f_min = ((f_long < 0 ? f_long*-1 : f_long) -((int ) i_deg))*60.0;
  if (f_min > 59.99995) { f_min = 0.0; i_deg++; };
  if (i_deg >= 90) { i_deg = 89; f_min = 59.9999; };
  sprintf(buf, "%3.3d-%07.4f%c", i_deg, f_min, f_long < 0 ? 'W' :  'E');
  preferences.putString(PREF_APRS_LONGITUDE_PRESET, String(buf));
  #if defined(ENABLE_SYSLOG)
    if (debug_verbose)
      syslog_log(LOG_DEBUG, String("FlashWrite preferences: store_lat_long()"));
  #endif

}


boolean is_locator(String s) {
  int i = s.length();
  if ((i % 2) || i < 6)
    return false;
  const char *p = s.c_str();
  if (*p < 'A' || p[1] > 'R')
    return false;
  p += 2;
  while (*p) {
    if (*p < '0' || p[1] > '9')
      return false;
    p += 2;
    if (!*p) return true;
    if (*p < 'A' || p[1] > 'X')
      return false;
    p += 2;
  }
  return true;
}


void locator_to_lat_lon(String s)
{
  float f_long, f_lat;
  const char *p = s.c_str();
  int s_len = s.length();
  f_long = (-9 + p[0]-'A') * 10 + p[2]-'0';
  p = p+4;
  uint32_t divisor = 1;
  while (p-s.c_str() < s_len) {
    if (*p >= 'A' && *p <= 'Z') {
      divisor *= 24;
      f_long = f_long + (*p-'A')/(float ) divisor;
    } else {
      divisor *= 10;
      f_long = f_long + (p[0]-'0')/(float ) divisor;
    }
    p = p+2;
  }
  f_long = f_long *2.0;

  p = s.c_str() + 1;
  f_lat = (-9 + p[0]-'A') * 10 + p[2]-'0';
  p = p+4;
  divisor = 1;
  while (p-s.c_str() < s_len) {
    if (*p >= 'A' && *p <= 'Z') {
      divisor *= 24;
      f_lat = f_lat + (*p-'A')/(float ) divisor;
    } else {
      divisor *= 10;
      f_lat = f_lat + (*p-'0')/(float ) divisor;
    }
    p = p+2;
  }
  store_lat_long(f_lat, f_long);
}


void set_lat_long(String s_lat, String s_long) {
  /*  Valid notations:
    53 14 59 N
    53 14 59.1 N
   -53 14 59
    53 15
    53 14.983
    53-14.983
    53-14.983 N
    53-14,983 N
    53° 14.983
    53° 14.983'
    53° 14' 59"
    53° 14' 59.999"
    53.2500
    5314.98 // aprs notation. pos value 2-3 < 60.
    JO62QN10

    Invalid notations:
    -53 14 59 N
    5375 // -> No, not 53° 45".
    53 -14.0 N
    53.25.01 N
    53,25.01 N
    91 N
    537 N (-> is it 53° 7min or 5° 37min ?)
    1301 E (deg in aprs notation is always len 3 (and 0<=x<180))
    3194.98 // deg*60 + minutes == (53 *60 + 14.983)
    5399.99 N
    185 E (instead of writing 175 W)
    11h55min ;)
  */

  float f_lat = 0.0f;
  float f_long = 0.0f;
  int curr_deg = 0;

  // If notation min ' sec "" is used, then sec has to follow min
  if (s_lat.indexOf("\"") > -1 && (s_lat.indexOf("\"") == -1 || s_lat.indexOf("\"") < s_lat.indexOf("'"))) goto out;
  if (s_long.indexOf("\"") > -1 && (s_long.indexOf("\"") == -1 || s_long.indexOf("\"") < s_long.indexOf("'"))) goto out;
  s_lat.toUpperCase(); s_lat.trim(); s_lat.replace("DEG", "-"); s_lat.replace("'", ""); s_lat.replace("\"", "");
  s_long.toUpperCase(); s_long.trim(); s_long.replace("DEG", "-"); s_long.replace("'", ""); s_long.replace("\"", "");

  if (s_long.isEmpty()) {
    if (s_lat.isEmpty())
      goto out;
    if (is_locator(s_lat)) {
      locator_to_lat_lon(s_lat);
      return;
    }
  } else {
    if (is_locator(s_long)) {
      locator_to_lat_lon(s_long);
      return;
    }
  }

  char buf[64];
  for (curr_deg = 0; curr_deg < 2; curr_deg++) {
    float f_degree = 0.0f;
    boolean is_negative = false;
    strncpy(buf, (curr_deg == 0) ? s_lat.c_str() : s_long.c_str(), sizeof(buf)-1);
    buf[sizeof(buf)-1] = 0;
    char *p = buf;
    char c = p[strlen(p)-1];
    char *q;

    if (c >= '0' && c <= '9') {
      if (*p == '-') {
        is_negative = 1;
        p = p+1;
        while (*p && *p == ' ') p++;
      }
    } else if ((curr_deg == 0 && (c == 'N' || c == 'S')) || (curr_deg == 1 && (c == 'E' || c == 'W'))) {
      if (c == 'S' || c == 'W')
        is_negative = 1;
      buf[strlen(buf)-1] = 0;
      q = buf + strlen(buf)-1;
      while(q > p && *q == ' ')
        q--;
    } else goto out;

    if (!*p) goto out;
    // Notations like position 53-02.13 N
    if ((q = strchr(p, '-')))
      *q = ' ';

    if (!isdigit(*p))
      goto out;

    if ((q = strchr(p, '.')) || (q = strchr(p, ','))) {
      *q++ = '.';
      while (*q && isdigit(*q))
        q++;
      *q = 0;
    }

    if ((q = strchr(p, ' '))) {
      float f_minute = 0.0f;
      while (*q && *q == ' ')
        *q++ = 0;
      char *r = strchr(q, ' ');
      if (r) {
        while (*r && *r == ' ')
          *r++ = 0;
        if (*r) {
          char *s = strchr(r, ' ');
          if (s) *s = 0;
          float f_seconds = atof(r);
          if (f_seconds < 0 || f_seconds >= 60)
            goto out;
          f_minute = f_seconds / 60.0;
        }
      }
      f_minute = atof(q) + f_minute;
      if (f_minute > 60.0)
        goto out;
      f_degree = f_minute / 60.0;
    } else {
      // dd.nnnn, or aprs notation ddmm.nn (latt) / dddmm.nn (long)
      if (strlen(p) == (curr_deg ? 8 : 7)) {
        q = p + (curr_deg ? 3 : 2);
        if (q[2] == '.') {
          // atof of mm.nn; dd part comes further down
          f_degree = atof(q) / 60.0;
          if (f_degree >= 1.0)
            goto out;
          // cut after dd
          *q = 0;
        }
      }
    }
    f_degree = atof(p) + f_degree + 0.0000001;
    if (curr_deg == 0) {
      if (f_degree > 89.99999) f_degree = 89.99999;
    } else {
       if (f_degree > 179.99999) f_degree = 179.99999;
    }

    if (f_degree < 0 || f_degree > (curr_deg == 1 ? 180 : 90))
      goto out;

    if (is_negative) f_degree *= -1;
    if (curr_deg == 0)
      f_lat = f_degree;
    else
      f_long = f_degree;
  }

out:
  store_lat_long(f_lat, f_long);
}

void handle_SaveAPRSCfg() {
  #if defined(ENABLE_SYSLOG)
    syslog_log(LOG_INFO, String("WebServer: handle_SaveAPPRSCfg()"));
  #endif
  do_serial_println("WebServer: WebServer: handle_SaveAPRSCfg()");
  // LoRa settings
  if (server.hasArg(PREF_LORA_FREQ_PRESET)){
    preferences.putDouble(PREF_LORA_FREQ_PRESET, server.arg(PREF_LORA_FREQ_PRESET).toDouble());
  }
  if (server.hasArg(PREF_LORA_SPEED_PRESET)){
    preferences.putInt(PREF_LORA_SPEED_PRESET, server.arg(PREF_LORA_SPEED_PRESET).toInt());
  }
  preferences.putBool(PREF_LORA_RX_ENABLE, server.hasArg(PREF_LORA_RX_ENABLE));
  preferences.putBool(PREF_LORA_TX_ENABLE, server.hasArg(PREF_LORA_TX_ENABLE));
  if (server.hasArg(PREF_LORA_TX_POWER)) {
    preferences.putInt(PREF_LORA_TX_POWER, server.arg(PREF_LORA_TX_POWER).toInt());
  }
  if (server.hasArg(PREF_LORA_TX_PREAMBLE_LEN)) {
    int i = server.arg(PREF_LORA_TX_PREAMBLE_LEN).toInt();
    if (i < 8) i = 8; else if (i > 4096) i = 4096;
    preferences.putInt(PREF_LORA_TX_PREAMBLE_LEN, i);
  }
  preferences.putBool(PREF_LORA_AUTOMATIC_CR_ADAPTION_PRESET, server.hasArg(PREF_LORA_AUTOMATIC_CR_ADAPTION_PRESET));
  if (server.hasArg(PREF_LORA_ADD_SNR_RSSI_TO_PATH_PRESET)){
    preferences.putInt(PREF_LORA_ADD_SNR_RSSI_TO_PATH_PRESET, server.arg(PREF_LORA_ADD_SNR_RSSI_TO_PATH_PRESET).toInt());
  }
  preferences.putBool(PREF_LORA_ADD_SNR_RSSI_TO_PATH_END_AT_KISS_PRESET, server.hasArg(PREF_LORA_ADD_SNR_RSSI_TO_PATH_END_AT_KISS_PRESET));
  if (server.hasArg(PREF_APRS_DIGIPEATING_MODE_PRESET)){
    preferences.putInt(PREF_APRS_DIGIPEATING_MODE_PRESET, server.arg(PREF_APRS_DIGIPEATING_MODE_PRESET).toInt());
  }
  if (server.hasArg(PREF_APRS_DIGIPEATING_MYALIAS)) {
    String s = server.arg(PREF_APRS_DIGIPEATING_MYALIAS);
    s.toUpperCase();
    s.trim();
    if (s.endsWith("-0"))
      s.replace("-0", "");

    if (!s.isEmpty()) {
      uint8_t is_valid = 1;
      const char *p;
      const char *q;

      p = s.c_str();
      for (q = p; *q; q++) {
        if (isalnum(*q) || *q == '-')
          continue;
        is_valid = 0;
        break;
      }

      if (is_valid) {
        is_valid = 0;
        q = strchr(p, '-');
        if (q) {
          if (q > p && q-p <= 6 && strlen(q) > 1 && strlen(q) <= 3) {
            q++;
            if (q[0]) {
              if (q[1]) {
                if (q[0] == '1' && q[1] >= '0' && q[1] <= '5') {
                  is_valid = 1;
                }
              } else {
                if (q[0] > '0' && q[0] <= '9') {
                  is_valid = 1;
                }
              }
            }
          }
        } else {
          if (*p && strlen(p) <= 6) {
            is_valid = 1;
          }
        }
      }
      if (!is_valid) {
        s = "";
      }
    }
    preferences.putString(PREF_APRS_DIGIPEATING_MYALIAS, s);
  }
  if (server.hasArg(PREF_APRS_CROSS_DIGIPEATING_MODE_PRESET)){
    preferences.putInt(PREF_APRS_CROSS_DIGIPEATING_MODE_PRESET, server.arg(PREF_APRS_CROSS_DIGIPEATING_MODE_PRESET).toInt());
  }
  if (server.hasArg(PREF_LORA_TX_BEACON_AND_KISS_TO_FREQUENCIES_PRESET)) {
    preferences.putInt(PREF_LORA_TX_BEACON_AND_KISS_TO_FREQUENCIES_PRESET, server.arg(PREF_LORA_TX_BEACON_AND_KISS_TO_FREQUENCIES_PRESET).toInt());
  }
  preferences.putBool(PREF_LORA_TX_BEACON_AND_KISS_TO_APRSIS_PRESET, server.hasArg(PREF_LORA_TX_BEACON_AND_KISS_TO_APRSIS_PRESET));
  preferences.putBool(PREF_LORA_TX_STATUSMESSAGE_TO_APRSIS_PRESET, server.hasArg(PREF_LORA_TX_STATUSMESSAGE_TO_APRSIS_PRESET));
  if (server.hasArg(PREF_LORA_FREQ_CROSSDIGI_PRESET)){
    preferences.putDouble(PREF_LORA_FREQ_CROSSDIGI_PRESET, server.arg(PREF_LORA_FREQ_CROSSDIGI_PRESET).toDouble());
  }
  if (server.hasArg(PREF_LORA_SPEED_CROSSDIGI_PRESET)){
    preferences.putInt(PREF_LORA_SPEED_CROSSDIGI_PRESET, server.arg(PREF_LORA_SPEED_CROSSDIGI_PRESET).toInt());
  }
  if (server.hasArg(PREF_LORA_TX_POWER_CROSSDIGI_PRESET)) {
    preferences.putInt(PREF_LORA_TX_POWER_CROSSDIGI_PRESET, server.arg(PREF_LORA_TX_POWER_CROSSDIGI_PRESET).toInt());
  }
  if (server.hasArg(PREF_LORA_RX_ON_FREQUENCIES_PRESET)) {
    preferences.putInt(PREF_LORA_RX_ON_FREQUENCIES_PRESET, server.arg(PREF_LORA_RX_ON_FREQUENCIES_PRESET).toInt());
  }
  // APRS station settings
  if (server.hasArg(PREF_APRS_CALLSIGN) && !server.arg(PREF_APRS_CALLSIGN).isEmpty()){
    uint8_t is_valid = 2;
    const char *p;
    const char *q;
    String s = server.arg(PREF_APRS_CALLSIGN);
    s.toUpperCase();
    s.trim();
    if (s.endsWith("-0"))
      s.replace("-0", "");

    p = s.c_str();
    for (q = p; *q; q++) {
      if (isalnum(*q) || *q == '-')
        continue;
      is_valid = 0;
      break;
    }

    if (is_valid) {
      is_valid = 0;
      q = strchr(p, '-');
      if (q) {
        if (q > p && q-p <= 6 && strlen(q) > 1 && strlen(q) <= 3) {
          q++;
          if (q[0]) {
            if ((q[0] >= 'A' && q[0] <= 'Z') || (q[1] >= 'A' && q[1] <= 'Z')) {
              // non-conformal SSIDs like "-L4" are ok for aprs-is, but should not be sent on RF -> disable TX
              is_valid = 1;
            } else {
              if (q[1]) {
                if (q[0] == '1' && q[1] >= '0' && q[1] <= '5') {
                  is_valid = 2;
                }
              } else {
                if (q[0] > '0' && q[0] <= '9') {
                  is_valid = 2;
                }
              }
            }
          }
        }
      } else {
        if (*p && strlen(p) <= 6) {
          is_valid = 2;
        }
      }
    }

    if (is_valid > 0) {
      preferences.putString(PREF_APRS_CALLSIGN, s);
    }
    if (is_valid < 2) {
      preferences.putBool(PREF_LORA_TX_ENABLE, false);
    }
  }
  if (server.hasArg(PREF_APRS_SYMBOL_TABLE) && !server.arg(PREF_APRS_SYMBOL_TABLE).isEmpty()){
    preferences.putString(PREF_APRS_SYMBOL_TABLE, server.arg(PREF_APRS_SYMBOL_TABLE));
  }
  if (server.hasArg(PREF_APRS_SYMBOL) && !server.arg(PREF_APRS_SYMBOL).isEmpty()){
    preferences.putString(PREF_APRS_SYMBOL, server.arg(PREF_APRS_SYMBOL));
  }
  if (server.hasArg(PREF_APRS_RELAY_PATH)){
    String s = server.arg(PREF_APRS_RELAY_PATH);
    s.toUpperCase(); s.trim(); s.replace(" ", ","); s.replace(",,", ",");
    if (s.endsWith("WIDE1") || s.endsWith("WIDE2")) s = s + "-1";
    if (s.indexOf("WIDE1,") > -1) s.replace("WIDE1,", "WIDE1-1,");
    if (s.indexOf("WIDE2,") > -1) s.replace("WIDE2,", "WIDE2-1,");
    // Avoid wrong paths like WIDE2-1,WIDE1-1
    if (! ( s.indexOf("WIDE1") > s.indexOf("WIDE") || s.indexOf("WIDE-") > -1 || s.startsWith("RFONLY,WIDE") || s.startsWith("NOGATE,WIDE") || s.indexOf("*") > -1 ))
      preferences.putString(PREF_APRS_RELAY_PATH, s);
  }
  if (server.hasArg(PREF_APRS_OBJECT_NAME)){
    String s = server.arg(PREF_APRS_OBJECT_NAME);
    s.trim();
    if (s == preferences.getString(PREF_APRS_CALLSIGN))
      preferences.putString(PREF_APRS_OBJECT_NAME, "");
    else
      preferences.putString(PREF_APRS_OBJECT_NAME, s);
  }
  if (server.hasArg(PREF_APRS_COMMENT)){
    preferences.putString(PREF_APRS_COMMENT, server.arg(PREF_APRS_COMMENT));
  }
  if (server.hasArg(PREF_APRS_LATLON_FROM_GPS) && gps_state && gps.location.isValid() && gps.location.age() < 10000) {
    // String conversion precison: 6 decimals. 5 shoud be enough, the six's is for good rounding
    set_lat_long(String(gps.location.lat(), 6), String(gps.location.lng(), 6));
  } else {
    set_lat_long(server.hasArg(PREF_APRS_LATITUDE_PRESET) ? server.arg(PREF_APRS_LATITUDE_PRESET) : String(""), server.hasArg(PREF_APRS_LONGITUDE_PRESET) ? server.arg(PREF_APRS_LONGITUDE_PRESET) : String(""));
  }
  //if (server.hasArg(PREF_APRS_LATITUDE_PRESET)){
    ////preferences.putString(PREF_APRS_LATITUDE_PRESET, server.arg(PREF_APRS_LATITUDE_PRESET));
    //String s_lat = latORlon(server.arg(PREF_APRS_LATITUDE_PRESET), 0);
    //if (s_lat.length() == 8)
      //preferences.putString(PREF_APRS_LATITUDE_PRESET, s_lat);
  //}
  //if (server.hasArg(PREF_APRS_LONGITUDE_PRESET)){
    ////preferences.putString(PREF_APRS_LONGITUDE_PRESET, server.arg(PREF_APRS_LONGITUDE_PRESET));
    //String s_lon = latORlon(server.arg(PREF_APRS_LONGITUDE_PRESET), 1);
    //if (s_lon.length() == 9)
      //preferences.putString(PREF_APRS_LONGITUDE_PRESET, s_lon);
  //}
  if (server.hasArg(PREF_APRS_POSITION_AMBIGUITY)) {
    preferences.putInt(PREF_APRS_POSITION_AMBIGUITY, server.arg(PREF_APRS_POSITION_AMBIGUITY).toInt());
  }
  if (server.hasArg(PREF_APRS_SENDER_BLACKLIST)){
    String s = server.arg(PREF_APRS_SENDER_BLACKLIST);
    s.toUpperCase(); s.trim(); s.replace(" ", ","); s.replace(",,", ",");
    preferences.putString(PREF_APRS_SENDER_BLACKLIST, (s.isEmpty() || s == ",") ? "" : s);
  }
  if (server.hasArg(PREF_TNC_SELF_TELEMETRY_INTERVAL)){
    preferences.putInt(PREF_TNC_SELF_TELEMETRY_INTERVAL, server.arg(PREF_TNC_SELF_TELEMETRY_INTERVAL).toInt());
  }
  if (server.hasArg(PREF_TNC_SELF_TELEMETRY_MIC)){
    preferences.putInt(PREF_TNC_SELF_TELEMETRY_MIC, server.arg(PREF_TNC_SELF_TELEMETRY_MIC).toInt());
  }
  if (server.hasArg(PREF_TNC_SELF_TELEMETRY_PATH)){
    String s = server.arg(PREF_TNC_SELF_TELEMETRY_PATH);
    s.toUpperCase(); s.trim(); s.replace(" ", ","); s.replace(",,", ",");
    preferences.putString(PREF_TNC_SELF_TELEMETRY_PATH, s);
  }
  if (server.hasArg(PREF_TNC_SELF_TELEMETRY_ALLOW_RF)){
    preferences.putInt(PREF_TNC_SELF_TELEMETRY_ALLOW_RF, server.arg(PREF_TNC_SELF_TELEMETRY_ALLOW_RF).toInt());
  }

  // Smart Beaconing settings
  if (server.hasArg(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET)){
    preferences.putInt(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET, server.arg(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET).toInt());
  }
  if (server.hasArg(PREF_APRS_SB_MIN_INTERVAL_PRESET)){
    preferences.putInt(PREF_APRS_SB_MIN_INTERVAL_PRESET, server.arg(PREF_APRS_SB_MIN_INTERVAL_PRESET).toInt());
  }
  if (server.hasArg(PREF_APRS_SB_MAX_INTERVAL_PRESET)){
    //preferences.putInt(PREF_APRS_SB_MAX_INTERVAL_PRESET, server.arg(PREF_APRS_SB_MAX_INTERVAL_PRESET).toInt());
    int i = server.arg(PREF_APRS_SB_MAX_INTERVAL_PRESET).toInt();
    if (i <= preferences.getInt(PREF_APRS_SB_MIN_INTERVAL_PRESET)) i = preferences.getInt(PREF_APRS_SB_MIN_INTERVAL_PRESET) +1;
    preferences.putInt(PREF_APRS_SB_MAX_INTERVAL_PRESET, i);
  }
  if (server.hasArg(PREF_APRS_SB_MIN_SPEED_PRESET)){
    preferences.putInt(PREF_APRS_SB_MIN_SPEED_PRESET, server.arg(PREF_APRS_SB_MIN_SPEED_PRESET).toInt());
  }
  if (server.hasArg(PREF_APRS_SB_MAX_SPEED_PRESET)){
    //preferences.putInt(PREF_APRS_SB_MAX_SPEED_PRESET, server.arg(PREF_APRS_SB_MAX_SPEED_PRESET).toInt());
    int i = server.arg(PREF_APRS_SB_MAX_SPEED_PRESET).toInt();
    if (i <= preferences.getInt(PREF_APRS_SB_MIN_SPEED_PRESET)) i = preferences.getInt(PREF_APRS_SB_MIN_SPEED_PRESET) +1;
    preferences.putInt(PREF_APRS_SB_MAX_SPEED_PRESET, i);
  }
  if (server.hasArg(PREF_APRS_SB_ANGLE_PRESET)){
    preferences.putDouble(PREF_APRS_SB_ANGLE_PRESET, server.arg(PREF_APRS_SB_ANGLE_PRESET).toDouble());
  }
  if (server.hasArg(PREF_APRS_SB_TURN_SLOPE_PRESET)){
    preferences.putInt(PREF_APRS_SB_TURN_SLOPE_PRESET, server.arg(PREF_APRS_SB_TURN_SLOPE_PRESET).toInt());
  }
  if (server.hasArg(PREF_APRS_SB_TURN_TIME_PRESET)){
    preferences.putInt(PREF_APRS_SB_TURN_TIME_PRESET, server.arg(PREF_APRS_SB_TURN_TIME_PRESET).toInt());
  }

  preferences.putBool(PREF_APRSIS_EN, server.hasArg(PREF_APRSIS_EN));
  if (server.hasArg(PREF_APRSIS_SERVER_NAME)){
    String s = server.arg(PREF_APRSIS_SERVER_NAME);
    preferences.putString(PREF_APRSIS_SERVER_NAME, s);
  }
  if (server.hasArg(PREF_APRSIS_SERVER_PORT)){
    preferences.putInt(PREF_APRSIS_SERVER_PORT, server.arg(PREF_APRSIS_SERVER_PORT).toInt());
  }
  if (server.hasArg(PREF_APRSIS_FILTER)){
    String s = server.arg(PREF_APRSIS_FILTER);
    s.trim();
    preferences.putString(PREF_APRSIS_FILTER, s);
  }
  if (server.hasArg(PREF_APRSIS_FILTER_LOCAL_INCOMING)){
    String s = server.arg(PREF_APRSIS_FILTER_LOCAL_INCOMING);
    s.trim();
    s.replace(" ", "");
    preferences.putString(PREF_APRSIS_FILTER_LOCAL_INCOMING, s);
  }
  if (server.hasArg(PREF_APRSIS_FILTER_LOCAL_WORDS_INCOMING)){
    String s = server.arg(PREF_APRSIS_FILTER_LOCAL_WORDS_INCOMING);
    s.trim();
    preferences.putString(PREF_APRSIS_FILTER_LOCAL_WORDS_INCOMING, s);
  }
  if (server.hasArg(PREF_APRSIS_CALLSIGN)){
    String s = server.arg(PREF_APRSIS_CALLSIGN);
    s.toUpperCase(); s.trim();
    preferences.putString(PREF_APRSIS_CALLSIGN, s);
  }
  if (server.hasArg(PREF_APRSIS_PASSWORD)){
    String s = server.arg(PREF_APRSIS_PASSWORD);
    s.trim();
    preferences.putString(PREF_APRSIS_PASSWORD, s);
  }
  if (server.hasArg(PREF_APRSIS_ALLOW_INET_TO_RF)){
    preferences.putInt(PREF_APRSIS_ALLOW_INET_TO_RF, server.arg(PREF_APRSIS_ALLOW_INET_TO_RF).toInt());
  }

  preferences.putBool(PREF_APRS_SHOW_BATTERY, server.hasArg(PREF_APRS_SHOW_BATTERY));
  preferences.putInt(PREF_APRS_STATUS_WINLINK_NOTIFICATION, server.arg(PREF_APRS_STATUS_WINLINK_NOTIFICATION).toInt());
  preferences.putBool(PREF_APRS_STATUS_SHUTDOWN_NOTIFICATION, server.hasArg(PREF_APRS_STATUS_SHUTDOWN_NOTIFICATION));
  preferences.putBool(PREF_ENABLE_TNC_SELF_TELEMETRY, server.hasArg(PREF_ENABLE_TNC_SELF_TELEMETRY));
  //preferences.putBool(PREF_APRS_SHOW_ALTITUDE, server.hasArg(PREF_APRS_SHOW_ALTITUDE));
  if (server.hasArg(PREF_APRS_ALTITUDE_RATIO)){
    preferences.putInt(PREF_APRS_ALTITUDE_RATIO, server.arg(PREF_APRS_ALTITUDE_RATIO).toInt());
  }
  preferences.putBool(PREF_APRS_ALWAYS_SEND_CSE_SPEED_AND_ALTITUDE, server.hasArg(PREF_APRS_ALWAYS_SEND_CSE_SPEED_AND_ALTITUDE));
  preferences.putBool(PREF_APRS_FIXED_BEACON_PRESET, server.hasArg(PREF_APRS_FIXED_BEACON_PRESET));
  preferences.putBool(PREF_APRS_GPS_EN, server.hasArg(PREF_APRS_GPS_EN));
  preferences.putInt(PREF_GPS_POWERSAVE, server.arg(PREF_GPS_POWERSAVE).toInt());
  preferences.putBool(PREF_ACCEPT_OWN_POSITION_REPORTS_VIA_KISS, server.hasArg(PREF_ACCEPT_OWN_POSITION_REPORTS_VIA_KISS));
  preferences.putBool(PREF_GPS_ALLOW_SLEEP_WHILE_KISS, server.hasArg(PREF_GPS_ALLOW_SLEEP_WHILE_KISS));
  preferences.putBool(PREF_APRS_SHOW_CMT, server.hasArg(PREF_APRS_SHOW_CMT));
  preferences.putBool(PREF_APRS_COMMENT_RATELIMIT_PRESET, server.hasArg(PREF_APRS_COMMENT_RATELIMIT_PRESET));

  // runtime reconfiguration with changed settings
  load_preferences_from_flash();
  setup_phase2_soft_reconfiguration(1);

  server.sendHeader("Location", "/");
  server.send(302,"text/html", "");

}

void handle_saveDeviceCfg(){
  #if defined(ENABLE_SYSLOG)
    syslog_log(LOG_INFO, String("WebServer: handle_SaveDeviceCfg()"));
  #endif
  do_serial_println("WebServer: WebServer: handle_SaveDeviceCfg()");
  preferences.putBool(PREF_DEV_BT_EN, server.hasArg(PREF_DEV_BT_EN));
  if (server.hasArg(PREF_DEV_USBSERIAL_DATA_TYPE)){
    preferences.putInt(PREF_DEV_USBSERIAL_DATA_TYPE, server.arg(PREF_DEV_USBSERIAL_DATA_TYPE).toInt());
  }
  preferences.putBool(PREF_DEV_OL_EN, server.hasArg(PREF_DEV_OL_EN));
  if (server.hasArg(PREF_DEV_SHOW_RX_TIME)){
    preferences.putInt(PREF_DEV_SHOW_RX_TIME, server.arg(PREF_DEV_SHOW_RX_TIME).toInt());
  }
  // Manage OLED Timeout
  if (server.hasArg(PREF_DEV_SHOW_OLED_TIME)){
    preferences.putInt(PREF_DEV_SHOW_OLED_TIME, server.arg(PREF_DEV_SHOW_OLED_TIME).toInt());
  }
  preferences.putBool(PREF_DEV_AUTO_SHUT, server.hasArg(PREF_DEV_AUTO_SHUT));
  if (server.hasArg(PREF_DEV_AUTO_SHUT_PRESET)){
    preferences.putInt(PREF_DEV_AUTO_SHUT_PRESET, server.arg(PREF_DEV_AUTO_SHUT_PRESET).toInt());
  }
  if (server.hasArg(PREF_DEV_REBOOT_INTERVAL)){
    preferences.putInt(PREF_DEV_REBOOT_INTERVAL, server.arg(PREF_DEV_REBOOT_INTERVAL).toInt());
  }
  if (server.hasArg(PREF_DEV_CPU_FREQ)){
    uint8_t cpufreq = server.arg(PREF_DEV_CPU_FREQ).toInt();
    if (cpufreq != 0 && cpufreq < 10)
      cpufreq = 10;
    preferences.putInt(PREF_DEV_CPU_FREQ, cpufreq);
  }
  if (server.hasArg(PREF_DEV_UNITS)){
    preferences.putInt(PREF_DEV_UNITS, server.arg(PREF_DEV_UNITS).toInt());
  }
  if (server.hasArg(PREF_DEV_OLED_L3_L4_FORMAT)){
    preferences.putInt(PREF_DEV_OLED_L3_L4_FORMAT, server.arg(PREF_DEV_OLED_L3_L4_FORMAT).toInt());
  }
  if (server.hasArg(PREF_DEV_OLED_LOCATOR)){
    preferences.putInt(PREF_DEV_OLED_LOCATOR, server.arg(PREF_DEV_OLED_LOCATOR).toInt());
  }
  if (server.hasArg(PREF_DEV_OLED_LOCATOR_AMBIGUITY)){
    preferences.putInt(PREF_DEV_OLED_LOCATOR_AMBIGUITY, server.arg(PREF_DEV_OLED_LOCATOR_AMBIGUITY).toInt());
  }

  // runtime reconfiguration with changed settings
  load_preferences_from_flash();
  setup_phase2_soft_reconfiguration(1);

  server.sendHeader("Location", "/");
  server.send(302,"text/html", "");
}


// WIFI related functions

boolean restart_STA(String use_ssid, String use_password) {
  int retryWifi = 0;

  if (!use_ssid.length() || (use_password.length() && (use_password.length() < 8 || use_password.length() > 63))) return false;

  WiFi.disconnect();
  WiFi.softAPdisconnect();
  WiFi.mode(WIFI_STA);
  WiFi.hostname(dnsHostname.c_str());
  WiFi.begin(use_ssid.c_str(), use_password.length() ? use_password.c_str() : nullptr);
  WiFi.setAutoReconnect(true);
  // Set power:  minimum 8 (2dBm) (max 80 (20dBm))
  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html
  // Mapping Table {Power, max_tx_power} = {{8, 2}, {20, 5}, {28, 7}, {34, 8}, {44, 11}, {52, 13}, {56, 14}, {60, 15}, {66, 16}, {72, 18}, {80, 20}}.
  // We'll use "min", "low", "mid", "high", "max" -> 2dBm (1.5mW) -> 8, 11dBm (12mW) -> 44, 15dBm (32mW) -> 60, 18dBm (63mW) ->72, 20dBm (100mW) ->80
  esp_wifi_set_max_tx_power(wifi_txpwr_mode_STA);
  wifi_connection_status = WIFI_SEARCHING_FOR_AP;

  do_serial_println("WiFi: Searching for AP " + use_ssid);
  while (WiFi.status() != WL_CONNECTED) {
    esp_task_wdt_reset();
    if (retryWifi > 30) {
      do_serial_println(String("WiFi: Status " + String((int ) WiFi.status()) + ". Try " + retryWifi + ". Giving up."));
      return false;
    }
    do_serial_println(String("WiFi: Status " + String((int ) WiFi.status()) + ". Try " + retryWifi));
    retryWifi += 1;
    vTaskDelay(500/portTICK_PERIOD_MS);
  }
  // WL_CONNECTED = Status 3
  do_serial_println(String("WiFi: Status " + String((int ) WiFi.status()) + ". Try " + retryWifi + ". Connected."));
  esp_task_wdt_reset();
  return true;;
}


void restart_AP_or_STA(void) {

  static boolean mode_sta_once_successfully_connected = false;
  static boolean first_run = true;

  boolean start_soft_ap = first_run ? true : false;
  first_run = false;
  String log_msg;
  String used_wifi_ModeSTA_SSID;
  String used_wifi_ModeSTA_PASS;

  if (apcnt) {
    static boolean successfully_associated = false;
    oled_wifi_SSID_curr = "[not connected]";
    oled_wifi_PASS_curr = "";
    oled_wifi_IP_curr = "0.0.0.0";
    int pos;

    start_soft_ap = false;
    for (pos = 0; pos < apcnt; pos++) {
      do_serial_println("Wifi: Trying AP " + String(pos)+1 + "/" + String(apcnt) + ": SSID " + APs[pos].ssid);
      wifi_connection_status = WIFI_SEARCHING_FOR_AP;
      successfully_associated = restart_STA(String(APs[pos].ssid), String(APs[pos].pw));
      if (successfully_associated) {
        used_wifi_ModeSTA_SSID = String(APs[pos].ssid);
        used_wifi_ModeSTA_PASS = String(APs[pos].pw);

        static uint32_t last_write = 0L;
        boolean changed = false;
        // store last successfull association into flash (preferences); ratelimit writing to flash
        if (!last_write || millis() > last_write + 5*60*1000L) {
          if (used_wifi_ModeSTA_PASS != preferences.getString(PREF_WIFI_PASSWORD, "")) {
            preferences.putString(PREF_WIFI_PASSWORD, used_wifi_ModeSTA_PASS);
            changed = true;
            last_write = millis();
          }
          if (used_wifi_ModeSTA_SSID != preferences.getString(PREF_WIFI_SSID, "")) {
            preferences.putString(PREF_WIFI_SSID, used_wifi_ModeSTA_SSID);
            changed = true;
            last_write = millis();
          }
          #if defined(ENABLE_SYSLOG)
            if (changed && debug_verbose)
              syslog_log(LOG_DEBUG, String("FlashWrite preferences: restart_AP_or_STA()"));
          #endif
        }
        if (pos) {
          // New high priority for this AP
          strcpy(APs[pos].ssid, APs[0].ssid);
          strcpy(APs[pos].pw, APs[0].pw);
          strcpy(APs[0].ssid, used_wifi_ModeSTA_SSID.c_str());
          strcpy(APs[0].pw, used_wifi_ModeSTA_PASS.c_str());
        }
        break;
      }
    }

    if (!successfully_associated) {
      if (wifi_do_fallback_to_mode_AP || (!mode_sta_once_successfully_connected && millis() < +3*60*1000L)) {
        start_soft_ap = true;
      } else {
        // if not fallback to mode ap, we are finished here. wifi_connection_status is still WIFI_SEARCHING_FOR_AP
        return;
      }
    }

  } else {

    // may start soft AP, if not already running as AP
    if (WiFi.getMode() == WIFI_MODE_AP) {
      // nothing to do
      return;
    }
    start_soft_ap = true;

  }

  if (start_soft_ap) {

    do_serial_println("WiFi: Status: " + String((int ) WiFi.status()) + ". Will run as mode AP (SSID: '" +  wifi_ModeAP_SSID + "')");
    WiFi.disconnect();
    WiFi.softAPdisconnect();
    WiFi.mode(WIFI_AP);

    log_msg="WiFi: Starting AP";
    #ifdef ENABLE_SYSLOG
      syslog_log(LOG_INFO, log_msg);
    #endif
    WiFi.softAP(wifi_ModeAP_SSID.c_str(), wifi_ModeAP_PASS.c_str());
    wifi_connection_status = WIFI_RUNNING_AS_AP;
    esp_wifi_set_max_tx_power(wifi_txpwr_mode_AP);

    oled_wifi_SSID_curr = wifi_ModeAP_SSID;
    oled_wifi_PASS_curr = wifi_ModeAP_PASS;
    oled_wifi_IP_curr = WiFi.softAPIP().toString();

    log_msg = "WiFi: Running AP. SSID: " + oled_wifi_SSID_curr + ". IP: " + oled_wifi_IP_curr;
    #ifdef ENABLE_SYSLOG
      syslog_log(LOG_INFO, log_msg);
    #endif
    do_serial_println(log_msg);
    wifi_connection_status = WIFI_RUNNING_AS_AP;

  } else if (WiFi.getMode() == WIFI_MODE_STA) {

    do_serial_println("WiFi: Status: " + String((int ) WiFi.status()) + ". Will run as mode STA (remote SSID: '" +  used_wifi_ModeSTA_SSID + "')");
    esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
    oled_wifi_SSID_curr = used_wifi_ModeSTA_SSID;
    oled_wifi_PASS_curr = used_wifi_ModeSTA_PASS;
    if (WiFi.status() == WL_CONNECTED) {
      oled_wifi_IP_curr = WiFi.localIP().toString();

      mode_sta_once_successfully_connected = true;
      wifi_connection_status = WIFI_CONNECTED_TO_AP;

      log_msg = "WiFi: Connected to AP " + oled_wifi_SSID_curr + "; got IP: " + oled_wifi_IP_curr;
      #ifdef ENABLE_SYSLOG
        syslog_log(LOG_INFO, log_msg);
      #endif
      do_serial_println(log_msg);

    } else {
      log_msg ="WiFi: Not successfully associated with AP " + oled_wifi_SSID_curr;
      wifi_connection_status = WIFI_NOT_CONNECTED_TO_AP;
      // reduce power consumption
      esp_wifi_set_max_tx_power(8);
    }
  } else {
    log_msg=String("WiFi: Unexpected Mode: ") + String(WiFi.getMode());
    #ifdef ENABLE_SYSLOG
      syslog_log(LOG_INFO, log_msg);
    #endif
    do_serial_println(log_msg);
    // reduce power consumption
    esp_wifi_set_max_tx_power(8);
    wifi_connection_status = WIFI_DISABLED;
    return;
  }

  String ntp_server = preferences.getString(PREF_NTP_SERVER, "");
  ntp_server.trim();
  if (ntp_server.isEmpty()) {
    if (oled_wifi_IP_curr.startsWith("44."))
      ntp_server = "ntp.hamnet.cloud";
    else
      ntp_server = "pool.ntp.org";
    preferences.putString(PREF_NTP_SERVER, ntp_server);
  }

  // configTime() stores the memory address of ntp-server.
  // Thus the variable needs to be static, else ntp will later will see garbage.
  // This forbids the use of ntp_server.c_str().
  // Now, we use there a static char array, instead of a more
  // inefficient String buffer. configTime() expects a char array anyway.
  // For more fun with bad concepts, see https://github.com/opendata-stuttgart/sensors-software/issues/471
  { static char buf[64];
    *buf = 0;
    strncpy(buf, ntp_server.c_str(), sizeof(buf) -1);
    buf[sizeof(buf)-1] = 0;
    configTime(0, 0, buf);
  }
  #ifdef ENABLE_SYSLOG
    struct tm timeinfo{};
    if(!getLocalTimeTheBetterWay(&timeinfo)){
      syslog_log(LOG_WARNING, "NTP: Failed to obtain time");
    } else {
      char buf[64];
      strftime(buf, 64, "%Y-%m-%d %H:%M:%S", &timeinfo);
      syslog_log(LOG_INFO, String("NTP: updated time: ") + String(buf));
    }
  #endif

  server.stop();
  server.begin();

  #ifdef KISS_PROTOCOL
    if (tncServer_enabled) {
      tncServer.stop();
      tncServer.begin();
    }
  #endif
  if (gpsServer_enabled) {
    gpsServer.stop();
    gpsServer.begin();
  }
  esp_task_wdt_reset();
}


// APRSIS related functions


#define SSMASTA_SHUTDOWN 0
#define SSMASTA_REBOOT 1

// on axp.shutdown(), say goodby to APRS-IS and close connection
void do_send_status_message_about_shutdown_or_reboot_to_aprsis(int why) {
  if (!aprsis_enabled || !aprsis_client.connected())
    return;

  if (send_status_message_to_aprsis) {
    String outString = aprsis_callsign + ">" + MY_APRS_DEST_IDENTIFYER + ":>APRSIS-Conn: ";
    char buf[19];// Room for len(20220917 01:02:03z) + 1 /* \0 */  -> 19
    struct tm timeinfo{};
    if (getLocalTimeTheBetterWay(&timeinfo)) {
      strftime(buf, sizeof(buf), "%Y%m%d %H:%M:%Sz", &timeinfo);
      //sprintf(buf, "%X%2.2d %2.2d:%2.2d:%2.2dz", timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    } else {
      if (strlen(gps_time_s) == 8)
        sprintf(buf, "%.8sz", gps_time_s);
      else
        strncpy(buf, gps_time_s, sizeof(buf));
    }
    if (*buf)
      outString = outString + String(buf) + ", ";
    if (aprsis_time_last_successful_connect.length()) {
      outString = outString + "since " + aprsis_time_last_successful_connect + ", ";
    }
    if (why == SSMASTA_SHUTDOWN)
      outString = outString + "Shutdown";
    else
      outString = outString + "Rebooting";
    //outString = outString + "[B" + buildnr + "]";
    aprsis_client.print(outString + "\r\n");
  }
  esp_task_wdt_reset();
  delay(500);
  aprsis_client.stop();
}

void do_send_status_message_about_reboot_to_aprsis()
{
  do_send_status_message_about_shutdown_or_reboot_to_aprsis(SSMASTA_REBOOT);
}
void do_send_status_message_about_shutdown_to_aprsis()
{
  do_send_status_message_about_shutdown_or_reboot_to_aprsis(SSMASTA_SHUTDOWN);
}

// send status mesg to APRS-IS. If (re)booted, print BUILDNUMBER
void do_send_status_message_about_connect_to_aprsis(void) {
  String log_msg;

  String outString = aprsis_callsign + ">" + MY_APRS_DEST_IDENTIFYER + ":>APRSIS-Conn: ";
  char buf[19];// Room for len(20220917 01:02:03z) + 1 /* \0 */  -> 19
  struct tm timeinfo{};
  if (getLocalTimeTheBetterWay(&timeinfo)) {
    strftime(buf, sizeof(buf), "%Y%m%d %H:%M:%Sz", &timeinfo);
    //sprintf(buf, "%X%2.2d %2.2d:%2.2d:%2.2dz", timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  } else {
    if (strlen(gps_time_s) == 8)
      sprintf(buf, "%.8sz", gps_time_s);
    else
      strncpy(buf, gps_time_s, sizeof(buf));
  }
  if (*buf)
    outString = outString + String(buf) + ", ";
  if (aprsis_time_last_successful_connect.length())
    outString = outString + "last " + aprsis_time_last_successful_connect;
  else {
    outString = outString + "Booted[B" + buildnr;
    if (millis() > 60*1000)
      outString = outString + ",up:" + String((int ) (millis()/1000/60));
    outString = outString + "]";
  }

  // remember this time as last connect time, for being able to reference it next time
  aprsis_time_last_successful_connect = String(buf);

  if (aprsis_connect_tries > 1)
    outString = outString + ", tries " + String(aprsis_connect_tries);

  log_msg = String("APRS-IS: sent status '" + outString + String("'"));
  #if defined(ENABLE_SYSLOG)
    syslog_log(LOG_INFO, log_msg);
  #endif
  do_serial_println(log_msg);

  esp_task_wdt_reset();
  aprsis_client.print(outString + "\r\n");

  outString.replace(":>", ",RFONLY:>");
  int do_delay = 0;
  if (lora_tx_enabled && tx_own_beacon_from_this_device_or_fromKiss__to_frequencies) {
    if (tx_own_beacon_from_this_device_or_fromKiss__to_frequencies % 2) {
      loraSend(txPower, lora_freq, lora_speed, 0, outString);  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
      stats_tx_freq_main++;
      do_delay = 600000 / lora_speed;
    }
    if (((tx_own_beacon_from_this_device_or_fromKiss__to_frequencies > 1 && lora_digipeating_mode > 1) || tx_own_beacon_from_this_device_or_fromKiss__to_frequencies == 5) && lora_freq_cross_digi > 1.0 && lora_freq_cross_digi != lora_freq) {
      loraSend(txPower_cross_digi, lora_freq_cross_digi, lora_speed_cross_digi, 0, outString);  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
      stats_tx_freq_secondary++;
      if (!do_delay || lora_speed_cross_digi < lora_speed)
        do_delay = 600000 / lora_speed_cross_digi;
    }
  }
  if (do_delay)
    delay(do_delay);

  esp_task_wdt_reset();
  #ifdef KISS_PROTOCOL
    sendToTNC(outString);
  #endif

  stats_tx_status++;
}


// connect to apprsis
int connect_to_aprsis(void) {
  String log_msg;

  if (aprsis_client.connected())
    aprsis_client.stop();

  aprsis_connect_tries++;
  log_msg = String("APRS-IS: connecting to '") + aprsis_host + "', tries " + String(aprsis_connect_tries);
  #if defined(ENABLE_SYSLOG)
    syslog_log(LOG_INFO, log_msg);
  #endif
  do_serial_println(log_msg);

  aprsis_status = "Connecting";
  aprsis_client.connect(aprsis_host.c_str(), aprsis_port);
  if (!aprsis_client.connected()) { aprsis_status = "Error: connect failed"; return -1; }
  aprsis_status = "Connected. Waiting for greeting.";

  uint32_t t_start = millis();
  while (!aprsis_client.available() && (millis()-t_start) < 25000L) { vTaskDelay(100 / portTICK_PERIOD_MS); esp_task_wdt_reset(); }
  if (aprsis_client.available()) {
    // check
    String s = aprsis_client.readStringUntil('\n');
    if (s.isEmpty() || !s.startsWith("#")) {
      aprsis_status = "Error: unexpected greeting";
      return -2;
    }
  } else {
    aprsis_status = "Error: No response"; return -3;
  }

  aprsis_status = "Login";

  char buffer[1024];
  sprintf(buffer, "user %.9s pass %.5s vers TTGO-T-Beam-LoRa-APRS-MDD-SAU-EL 2023-02-19",
    aprsis_callsign.c_str(), aprsis_password.c_str());
  if (!aprsis_filter.isEmpty())
    sprintf(buffer+strlen(buffer), " filter %.900s", aprsis_filter.c_str());
  aprsis_client.print(String(buffer) + "\r\n");

  t_start = millis();
  while (!aprsis_client.available() && (millis()-t_start) < 5000L) { vTaskDelay(100 / portTICK_PERIOD_MS); esp_task_wdt_reset(); }
  if (aprsis_client.available()) {
    // check
    String s = aprsis_client.readStringUntil('\n');
    if (s.isEmpty() || !s.startsWith("#")) { aprsis_status = "Error: unexpected reponse on login"; return -4; }
    if (s.indexOf(" logresp") == -1) { aprsis_status = "Error: Login denied: " + s; aprsis_status.trim(); return -5; }
    if (s.indexOf(" verified") == -1) { aprsis_status = "Notice: server responsed not verified: " + s; aprsis_status.trim(); }
  } else { aprsis_status = "Error: No response"; return -6; }
  aprsis_status = "Logged in";

  log_msg = String("APRS-IS: connected to '" + aprsis_host + String("' [") + aprsis_client.remoteIP().toString() + "]");
  #if defined(ENABLE_SYSLOG)
    syslog_log(LOG_INFO, log_msg);
  #endif
  do_serial_println(log_msg);

  // send aprs-status packet?
  if (send_status_message_to_aprsis)
    do_send_status_message_about_connect_to_aprsis();

  aprsis_connect_tries = 0;

  // avoid sending old data
  to_aprsis_data = "";

  // WiFi powermanagement. Disable sleep -> better network performance
  // Default for WiFi sleep is on.
  // But (bugfix): A user reported crash
  //   "E (27793) wifi:Error! Should enable WiFi modem sleep when both WiFi and Bluetooth are enabled!!!!!!"
  //   abort() was called at PC 0x40139607 on core 0
  // -> Enable this feature only if bluetooth is disabled
  if (!enable_bluetooth)
    WiFi.setSleep(false);

  return 0;
}


String generate_third_party_packet(String callsign, String packet_in) {
  String packet_out = "";
  const char *s = packet_in.c_str();
  char *p = strchr(s, '>');
  char *q = strchr(s, ',');
  char *r = strchr(s, ':');
  char buf[82]; // room for max (due to spec) 'DL9SAU-15>APRSXX-NN' + \0. No, we use buf also for digi path
                // -> room for len("DL9SAU-12,") == 10, * up-to 8 digipeater, plus '*' plus \0. 10*8+1+1 = 82

  if (p && q && r && p > s && p-s < 10 && p < q && q < r && (q-s) < sizeof(buf)) {
    strncpy(buf, s, q-s);
    buf[q-s] = 0;
    packet_out = callsign + ">" + MY_APRS_DEST_IDENTIFYER + ":}" + buf;
                             // ^ 3rd party traffic should be addressed directly (-> not to WIDE2-1 or so)

    // Copy repeater ",DIGI1,DIGI2*,..". Buf size is 82 (incl. \0). Copy header (excl. ':'), without leading ','
    // Skip leading ','
    q++;
    // snprintf len is incl. \0
    snprintf(buf, r-q +1, "%.81s", q);
    buf[sizeof(buf)-1] = 0;
    // strip q-construct
    if (*(q = buf) == 'q' || (q = strstr(buf, ",q"))) {
      *q = 0;
    }
    // Strip TCPIP if present. We'll add it. We like to avoid packets like TCPIP,TCPIP*
    if ( ((q = strstr(buf, "TCPIP")) && (!q[5] || q[5] == '*' || q[5] == ',') && (q == buf || *(q-1) == ',')) ) {
      if (q > buf && *(q-1) == ',') {
        q--;
      }
      *q = 0;
    }
    // cut unused digipeaters. Remove '*' from the last-repeated one.
    if ((q = strchr(buf, '*'))) {
      *q = 0;
    } else {
      // remove unused digipeaters from path, according to aprs spec
      *buf = 0;
    }
    if (*buf) {
      packet_out = packet_out + "," + buf;
    }
    packet_out = packet_out + ",TCPIP," + callsign + "*" + r;
  }
  return packet_out;
}

char aprsis_own_filter_check(const char *data) {
  char *q;
  int len;
  char call[10];

  switch (data[0]) {

  case '!':
  case '=':
    len = strlen(data);
    if ((len > 12 && data[12] == '_') || (len > 19 && data[19] == '_'))
      return 'w';
    return 'p';
  case '/':
  case '@':
    len = strlen(data);
    if ((len > 18 && data[18] == '_') || (len > 26 && data[26] == '_'))
      return 'w';
    return 'p';
  case '[':
  case '$':  case '\'':
  case '`':
    return 'p';
  case ';':
    len = strlen(data);
    if ((len > 29 && data[29] == '_') || (len > 36 && data[36] == '_') ||
        (len > 20 && data[20] == '_') || (len > 28 && data[27] == '_'))
      return 'w';
    return 'o';
  case ')':
    return 'i';
  case '#':
  case '*':
  case '_':
    return 'w';
  case 'T':
    return 't';
  case ':':
    if (strlen(data) < 11 || data[10] != ':')
      return '~';
    strncpy(call, data+1, 9);
    call[9] = 0;
    if ((q = strchr(call, ' ')))
      *q = 0;
    if (!strncmp(call, "BLN", 3))
      return 'b';
    else if (!strncmp(call, "NWS-", 4))
      return 'n';
    else if (is_call_blacklisted(call))
      return '~';
    else if (data[11] == '?')
      return 'q';
    else if (!strncmp(data + 11, "PARM.", 5) || !strncmp(data + 11, "UNIT.", 5) || !strncmp(data + 11, "EQNS.", 5) ||!strncmp(data + 11, "BITS.", 5))
      return 't';
    return 'm';
  case '?':
  case '<':
    return 'q';
  case '>':
    return 's';
  case '}':
    if ((q = strchr(data+1, ':')) && q > strchr(data+1, '>'))
      return aprsis_own_filter_check(q + 1);
    return '~';
  case '{':
  case ',':
    return 'u';
  case '%':
    return 'p';
  }
  return '~';
}


// read packets from APRSIS
void read_from_aprsis(void) {
  static uint32_t t_last_rxqueue_was_empty = 0L;
  bool src_call_not_for_rf = false;
  uint8_t err = 0;;
  String log_msg;

  if (!aprsis_client.connected()) {
    t_last_rxqueue_was_empty = 0L;
    return;
  }

  // packet in rx buffer?
  if (!aprsis_client.available()) {
    t_last_rxqueue_was_empty = millis();
    return;
  }
  if (t_last_rxqueue_was_empty == 0L)
     t_last_rxqueue_was_empty = millis();

  // lora transmissions take abt 3-5s. aprs-is can easily flood the channel.
  // With aprs-filter positions in radius 150km, our queue went so large.
  // -> if in between 25s the rx-queue did not become empty at least one time, tidy up.
  if (millis() - t_last_rxqueue_was_empty > 25000L) {
    // clear queue
    while (aprsis_client.available()) {
      String s = aprsis_client.readStringUntil('\n');
      esp_task_wdt_reset();
      if (s.isEmpty())
        break;
    }
    t_last_rxqueue_was_empty = millis();
    return;
  }

  const char *q = 0;
  const char *header_end = 0;

  //aprsis_status = "OK, reading";
  String s = aprsis_client.readStringUntil('\n');

  if (s) {
    //aprsis_status = "OK";
    t_aprsis_lastRXorTX = millis();
    s.trim();
    if (s.isEmpty()) {
      err = 2;
      log_msg = "unexpected answer from aprsis-connection";
    } else {
      // idle message from aprs-is server?
      if (*(s.c_str()) == '#')
        return;
      // a, _, ' ', >, ... are invalid in the header. First do a simple check
      if (!isalnum(*(s.c_str()))) {
        log_msg = "unexpected answer from aprsis-connection";
        err = 2;
      } else {
        char *header_end = strchr(s.c_str(), ':');
        if (!header_end || !(*(header_end+1))) {
          log_msg = "looks not like an aprs frame (separator ':' between path and payload is missing, or no payload)";
          err = 2;
        } else {
          char *src_call_end = strchr(s.c_str(), '>');
          if (!src_call_end) {
            log_msg = "looks not like an aprs frame (separator '>' in the header between src-call and dst-call is missing";
            err = 2;
          } else {
            if (src_call_end > header_end-2) {
              log_msg = "looks not like an aprs frame (separator ':' in AX.25 header!";
              err = 2;
            } else {
              // do not interprete packets coming back from aprs-is net (either our source call, or if we have repeated it with one of our calls
              // sender is our call?
              if (s.startsWith(aprs_callsign + '>') || s.startsWith(aprsis_callsign + '>'))
                return;
              q = strchr(s.c_str(), '-');
              if (q && q < src_call_end) {
                // len callsign > 6?
                if (q-s.c_str() > 6) {
                  log_msg = "bad length of call (> 6)!";
                  err = 1;
                } else {
                  // SSID optional, only 0..15
                  if (q[2] == '>') {
                    if (q[1] < '1' || q[1] > '9') {
                      //err = 1;
                      src_call_not_for_rf = true;
                    } // else: ssid is fine
                  } else if (q[3] == '>') {
                    if (q[1] != '1' || q[2] < '0' || q[2] > '5') {
                      //err = 1;
                      src_call_not_for_rf = true;
                    } // else: ssid is fine
                  } else {
                    //err = 1;
                    src_call_not_for_rf = true;
                  }
                  if (err) {
                    log_msg = "src-call with bad SSID";
                  }
                }
              } else {
                // call without ssid is fine
                if (src_call_end-s.c_str() > 6) {
                  //log_msg = "bad length of call (> 6)!";
                  //err = 1;
                  src_call_not_for_rf = true;
                }
              }
              if (!err && src_call_end-s.c_str() > 9) {
                log_msg = "src-call length > 9 breaks APRS standard";
                err=1;
              }
            }
          }
          if (!err) {
            for (q = s.c_str(); *q && *q != ':'; q++) {
              // q (for qAR) is also a valid character
              //if (! ( (*q >= '0' && *q <= '9') || (*q >= 'A' && *q <= 'Z') || *q == 'q' || *q == '>' || *q == '-' || *q == ',' || *q == '*' ) ) {
              //No, unfortunately some aprs-submitters have a lowercase call
              if (! ( isalnum(*q) || *q == '>' || *q == '-' || *q == ',' || *q == '*' ) ) {
                err = 1;
                log_msg = "bad character in header";
                break;
              }
            }
          }
        }
      }
    }
  } else {
    s = "";
    err = 2;
  }

  if (err) {
    if (err > 1) {
      aprsis_client.stop();
      log_msg = "disconnecting: ";
    }
    log_msg = "APRS-IS: read_from_aprsis(): " + log_msg + ": '" + s + "'";
    #if defined(ENABLE_SYSLOG)
      syslog_log(LOG_INFO, log_msg);
    #endif
    do_serial_println(log_msg);
    return;
  }
  // bug: header_end may be 0 (due to crash trace), but shuoldn't: we tried to assure.
  // Can't see why this happens.
  // Code without "goto" is ugly and can lead to strange results ;)
  // Needs to be resolved. This is a the&D fix:
  // search for ':' in String s again
  if (!header_end && !(header_end = strchr(s.c_str(), ':'))) {
    log_msg = "APRS-IS: read_from_aprsis(): BUG! header_end is NULL: '" + s + "'";
    #if defined(ENABLE_SYSLOG)
      syslog_log(LOG_INFO, log_msg);
    #endif
    do_serial_println(log_msg);
    return;
  }

  if (!header_end[1])
    return;

  // packet has our call in i.E. ...,qAR,OURCALL:...
  q = strstr(s.c_str(), (',' + aprsis_callsign + ':').c_str());
  if (q && q < header_end) return;
  for (int i = 0; i < 2; i++) {
    String call = (i == 0 ? aprs_callsign : aprsis_callsign);
    // digipeated frames look like "..,DL9SAU,DL1AAA,DL1BBB*,...", or "..,DL9SAU*,DL1AAA,DL1BBB,.."
    if (((q = strstr(s.c_str(), (',' + call + '*').c_str())) || (q = strstr(s.c_str(), (',' + call + ',').c_str()))) && q < header_end) {
      char *digipeatedflag = strchr(q, '*');
      if (digipeatedflag && digipeatedflag < header_end && digipeatedflag > q)
        return;
    }
  }

  if (is_call_blacklisted(s.c_str()))
    return;

  if (!aprsis_own_filters_words_in.isEmpty()) {
    char *p = strdup(aprsis_own_filters_words_in.c_str());
    char *q = strtok(p, " ");
    while(q) {
      if (s.indexOf(q) > -1) {
        free(p);
        return;
      }
      q = strtok(NULL, " ");
    }
    if (p)
      free(p);
  }

  String answer_message = handle_aprs_messsage_addressed_to_us(s.c_str());
  boolean its_an_aprs_message_for_us = !answer_message.isEmpty();
  if (answer_message == "M") {
    // It was a message for us. It contained an answer message we have to send;
    // or a Placeholder "M" if no ack message is required -- in this case, clear the message
    answer_message = "";
  }


  // generate third party packet. Use aprs_callsign (deriving from webServerCfg->callsign), because aprsis_callsign may have a non-aprs (but only aprsis-compatible) ssid like '-L4'
  String third_party_packet = generate_third_party_packet(aprs_callsign, s);
  if (third_party_packet.isEmpty())
    return;

  if (!its_an_aprs_message_for_us) {
    if (!aprsis_own_filters_in.isEmpty()) {
      char aprs_data_type = aprsis_own_filter_check(header_end+1);

      // Sometheing really negative, like a blacklisted call writing a message
      if (aprs_data_type == '~')
        return;

      if (aprsis_own_filter_in_is_whitelist) {
        if (aprsis_own_filters_in.indexOf(aprs_data_type) == -1) {
          // Data type is not whitelisted
          return;
        // else: // Found in whitelist, may pass
        }
      } else {
        if (aprsis_own_filters_in.indexOf(aprs_data_type) != -1) {
          // Data type is blacklisted
          return;
        } // else: Not found in blacklist, may pass
      }
    }

    // Don't gate, if packet has word TCPXX, NOGATE or RFONLY in header (TCPIP is allowed). See aprs-is iGateDetails spec
    if (((q = strstr(s.c_str(), ",TCPXX")) && (q[6] == '*' || q[6] == ',' || q[6] == ':')) && q < header_end)
      return;
    //if ((q = strstr(s.c_str(), ",NOGATE")) && (q[7] == '*' || q[7] == ',' || q[7] == ':') && q < header_end)
      //return;
    if ((q = strstr(s.c_str(), ",RFONLY")) && (q[7] == '*' || q[7] == ',' || q[7] == ':') && q < header_end)
      return;

  }

  aprsis_status = "OK, fromAPRSIS: " + s + " => " + third_party_packet; aprsis_status.trim();

  esp_task_wdt_reset();

  stats_from_aprsis++;
  if (usb_serial_data_type & 2) {
    Serial.println(third_party_packet);
    stats_to_serial++;
  }

#ifdef KISS_PROTOCOL
  if (!src_call_not_for_rf) {
    if ((q = strstr(s.c_str(), ",NOGATE")) && (q[7] == '*' || q[7] == ',' || q[7] == ':') && q < header_end)
      return;
    sendToTNC(third_party_packet);
  }
#endif

  if (its_an_aprs_message_for_us) {
    if (!answer_message.isEmpty()) {
      to_aprsis_data = String(answer_message);
      send_queue_to_aprsis();
      if (usb_serial_data_type & 2)
        Serial.println(third_party_packet);
    }
    return; // Message was for us -> do not TX on RF
  }

  // not query
  // Format: "..::DL9SAU-15:..."
  //q = header_end + 1;
  //if (*q == '?')
    //return;
  // aprs-message addressed to our call (check both, aprs_callsign and aprsis_callsign)?
  // check is in this code part, because we may like to see those packets via kiss (sent above)
  // was already checked with our new handle_aprs_messsage_addressed_to_us() function. checked at least for aprs_callsign
  q = header_end + 1;
  if (*q == ':' && strlen(q) > 10 && q[10] == ':' &&
      //((!strncmp(q+1, aprs_callsign.c_str(), aprs_callsign.length()) && (aprs_callsign.length() == 9 || q[9] == ' ')) ||
      //(!strncmp(q+1, aprsis_callsign.c_str(), aprsis_callsign.length()) && (aprsis_callsign.length() == 9 || q[9] == ' ')) ))
      !strncmp(q+1, aprsis_callsign.c_str(), aprsis_callsign.length()) && (aprsis_callsign.length() == 9 || q[9] == ' ') )
    return;

   if (!(lora_tx_enabled && aprsis_data_allow_inet_to_rf && !src_call_not_for_rf))
    return;

  // Now, finally, we may gate to RF.

  // Give main loop a chance to finish loraSend and parse a new packet in the incoming queue.
  // Sema locking is important because waitAvailableTimeout() calls available() which calls setModeRx() and this may cause a
  // panic if the chip is currently transmitting. We can skip this, if lora rx is disabled (which, btw, would not make sense)
  if (lora_rx_enabled) {
    #ifdef HAS_SX127X
      sema_lock_or_unlock__lora(1);
      //flag_lora_packet_available = rf95.waitAvailableTimeout(lora_speed_rx_curr > 300 ? 2500 : 10000);
      flag_lora_packet_available = rf95.available();
      sema_lock_or_unlock__lora(0);
    #elif HAS_SX126X
      // RadioLib driver sets the flag if a packet is received
    #endif
    // After lock release, main thread could gather packet. Give him enough time to receive and parse the packet
    if (flag_lora_packet_available) {
      delay(250);
    }
  }
  int do_delay = 0;
  if (aprsis_data_allow_inet_to_rf % 2) {
    loraSend(txPower, lora_freq, lora_speed, 0, third_party_packet);
    stats_tx_freq_main++;
    do_delay = 600000 / lora_speed;
  }
  if (aprsis_data_allow_inet_to_rf > 1 && lora_freq_cross_digi > 1.0 && lora_freq_cross_digi != lora_freq) {
    loraSend(txPower_cross_digi, lora_freq_cross_digi, lora_speed_cross_digi, 0, third_party_packet);
    stats_tx_freq_secondary++;
    if (!do_delay || lora_speed_cross_digi < lora_speed)
      do_delay = 600000 / lora_speed_cross_digi;
  }
  if (do_delay) {
    delay(do_delay);
    esp_task_wdt_reset();
  }
}


// forward packets to APRSIS
void send_queue_to_aprsis()
{
  if (!aprsis_client.connected() || !to_aprsis_data || !to_aprsis_data.length())
    return;

  // copy(). We are threaded..
  String data = String(to_aprsis_data);
  // clear queue
  to_aprsis_data = "";
  // Avoid breaking aprs-is protocol with frames lile "FOO>APRS:>helo\rworld"
  data.replace("\r", " "); data.replace("\n", " "); data.replace("\0", " ");
  data.trim();
  char *p = strchr(data.c_str(), '>');
  char *header_end;
  // some plausibility checks.
  if (p && p > data.c_str() && (header_end = strchr(p+1, ':'))) {
    // Due to http://www.aprs-is.net/IGateDetails.aspx , never gate third-party traffic with TCPIP or TCPXX
    // IGATECALL>APRS,GATEPATH:}FROMCALL>TOCALL,TCPIP,IGATECALL*:original packet data
    const char *p_data = data.c_str();
    int len = header_end - p_data;
    char *r;
    if (header_end[1] != '?' &&
        !( ((r = strstr(p+1, ",TCPIP")) && (r[6] == '*' || r[6] == ',' || r[6] == ':') && r < header_end) ||
           ((r = strstr(p+1, ",TCPXX")) && (r[6] == '*' || r[6] == ',' || r[6] == ':') && r < header_end) ||
           ((r = strstr(p+1, ",NOGATE")) && (r[7] == '*' || r[7] == ',' || r[7] == ':') && r < header_end) ||
           ((r = strstr(p+1, ",RFONLY")) && (r[7] == '*' || r[7] == ',' || r[7] == ':') && r < header_end) ) ) {
      char optional_third_party_sender[10] = ""; // room for DL9SAU-15 + \0 == 10
      if (header_end[1] == '}') {
        // Third party traffic. Strip the RF header, due to aprsis-spec
        snprintf(optional_third_party_sender, p-p_data +1, "%.9s*", p_data);
        optional_third_party_sender[sizeof(optional_third_party_sender)-1] = 0;
        p_data = header_end+2; //  // go to start of callsign behind '}'-marker.
        p = strchr(p_data, '>');
        header_end = strchr(p_data, ':');
        len = 0;
        if (p && header_end) {
          int len_optional_third_party_sender = strlen(optional_third_party_sender);
          if ((header_end - p > len_optional_third_party_sender) && *(header_end -1) == '*' && !strncmp(header_end - 1 - len_optional_third_party_sender, optional_third_party_sender, len_optional_third_party_sender)) {
            // src call already in path?
            *optional_third_party_sender = 0;
            len_optional_third_party_sender = 0;
          }
          if (*p_data && p && header_end && p < header_end) {
            // Look also in the third-party header
            if (!( ((r = strstr(p+1, ",TCPIP")) && (r[6] == '*' || r[6] == ',' || r[6] == ':') && r < header_end) ||
               ((r = strstr(p+1, ",TCPXX")) && (r[6] == '*' || r[6] == ',' || r[6] == ':') && r < header_end) ||
               ((r = strstr(p+1, ",NOGATE")) && (r[7] == '*' || r[7] == ',' || r[7] == ':') && r < header_end) ||
               ((r = strstr(p+1, ",RFONLY")) && (r[7] == '*' || r[7] == ',' || r[7] == ':') && r < header_end) ) ) {
             len = header_end - p_data;
           }
         }
       }
      }
      char buf[256];
      if (len && len < sizeof(buf) && *p_data && header_end) {
        // just to be sure
        strncpy(buf, p_data, len);
        buf[len] = 0;
        String s_buf = String(buf);
        if (*optional_third_party_sender) {
          if ((p = strchr(buf, '*')) && p < header_end) {
            s_buf.replace("*", (String(",") + String(optional_third_party_sender) + String("*")));
          } else {
            s_buf.replace(":", s_buf + String(",") + String(optional_third_party_sender) + String("*:"));
          }
        }
        String s_data = s_buf + (lora_tx_enabled ? ",qAR," : ",qAO,") + aprsis_callsign + header_end + "\r\n";
        aprsis_status = "OK, toAPRSIS: " + s_data; aprsis_status.trim();
        aprsis_client.print(s_data);
        t_aprsis_lastRXorTX = millis();
        esp_task_wdt_reset();
      }
    }
    //aprsis_status = "OK";
  }
}


// main task: our Webserver

[[noreturn]] void taskWebServer(void *parameter) {
  auto *webServerCfg = (tWebServerCfg*)parameter;
  wifi_ModeAP_SSID = webServerCfg->callsign + " AP";

  server.on("/", handle_Index);
  server.on("/favicon.ico", handle_NotFound);
  server.on("/style.css", handle_Style);
  server.on("/js.js", handle_Js);
  server.on("/scan_wifi", handle_ScanWifi);
  server.on("/save_wifi_cfg", handle_SaveWifiCfg);
  server.on("/reboot", handle_Reboot);
  server.on("/beacon", handle_Beacon);
  server.on("/shutdown", handle_Shutdown);
  server.on("/cfg", handle_Cfg);
  server.on("/wificfg", handle_WifiCfg);
  server.on("/received_list", handle_ReceivedList);
  server.on("/save_aprs_cfg", handle_SaveAPRSCfg);
  server.on("/save_device_cfg", handle_saveDeviceCfg);
  server.on("/save2fs", handle_saveCfg2FS);
  server.on("/savewifi2fs", handle_saveWifi2FS);
  server.on("/restore", handle_Restore);
  server.on("/update", HTTP_POST, []() {
#if defined(ENABLE_SYSLOG)
    syslog_log(LOG_WARNING, String("Firmware: Update finished. Status: ") + (!Update.hasError() ? "Ok" : "Error"));
#endif
    do_send_status_message_about_reboot_to_aprsis();
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    delay(500);
    ESP.restart();
  }, []() {
    static bool aprsis_enabled_prev = aprsis_enabled;
    static bool lora_rx_enabled_prev = lora_rx_enabled;
    static bool lora_tx_enabled_prev = lora_tx_enabled;
    bool unlock_sema_and_reinit_lora_chip = false;
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      lora_rx_enabled = false;
      lora_tx_enabled = false;
      if (aprsis_enabled && aprsis_client.connected()) {
        aprsis_client.stop();
      }
      aprsis_enabled = false;
      sema_lock_or_unlock__lora(1);
      #ifdef HAS_SX127X
        rf95.setTxPower(0);
        // hack to circumvent crash on interrupt handler isr0():
        rf95.setFrequency(lora_freq_rx_curr + 1);
        rf95.sleep(); // disable rf95 before update
      #elif HAS_SX126X
        radio.setOutputPower(0);
        // hack to circumvent crash on interrupt handler isr0():
        radio.setFrequency(lora_freq_rx_curr + 1);
        radio.sleep(); // disable rf95 before update
      #endif
      // switch LORA chip off during firmware upload
      #ifdef T_BEAM_V1_0
        axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF);
      #elif T_BEAM_V1_2
        axp.disableALDO2();
      #endif
      if (!enable_bluetooth)
        WiFi.setSleep(false);
      Serial.printf("Firmware: Update: %s\r\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        #if defined(ENABLE_SYSLOG)
          syslog_log(LOG_ERR, String("Firmware: Update begin error: ") + Update.errorString());
        #endif
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        #if defined(ENABLE_SYSLOG)
          syslog_log(LOG_ERR, String("Firmware: Update error: ") + Update.errorString());
        #endif
        Update.printError(Serial);
        unlock_sema_and_reinit_lora_chip = true;
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Firmware: Update Success: %u\r\nFirmware: Rebooting...\r\n", upload.totalSize);
        #if defined(ENABLE_SYSLOG)
          syslog_log(LOG_WARNING, String("Firmware: Update Success: ") + String((int)upload.totalSize) + "byte. Rebooting...");
        #endif
      } else {
        #if defined(ENABLE_SYSLOG)
          syslog_log(LOG_ERR, String("Firmware: Update error: ") + Update.errorString());
        #endif
        Update.printError(Serial);
      }
      unlock_sema_and_reinit_lora_chip = true;
    }
    if (unlock_sema_and_reinit_lora_chip) {
      sema_lock_or_unlock__lora(0);
      #ifdef T_BEAM_V1_0
        axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
      #elif T_BEAM_V1_2
        axp.setALDO2Voltage(3300);
        axp.enableALDO2();
      #endif
      // hack to circumvent crash on interrupt handler isr0():
      #ifdef HAS_SX127X
        rf95.setFrequency(lora_freq_rx_curr);
        rf95.setTxPower(txPower);
      #elif HAS_SX126X
        radio.setFrequency(lora_freq_rx_curr);
        radio.setOutputPower(txPower);
      #endif
      lora_set_speed(lora_speed_rx_curr);
      lora_tx_enabled = lora_tx_enabled_prev;
      lora_rx_enabled = lora_rx_enabled_prev;
      aprsis_enabled = aprsis_enabled_prev;
      unlock_sema_and_reinit_lora_chip = false;
    }
  });
  server.onNotFound(handle_NotFound);


  // esp_task_wdt_init() has already been done in main task during setup()
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  // 8 characters is requirements for WPA2
  // May be already set by wifi.cfg
  if (wifi_ModeAP_PASS.length() < 8) {
    wifi_ModeAP_PASS = preferences.getString(PREF_AP_PASSWORD, "");
    if (wifi_ModeAP_PASS.length() < 8) {
      wifi_ModeAP_PASS = wifi_ModeAP_PASS_default;
    }
  }


 #ifdef ENABLE_SYSLOG
   String syslog_server = preferences.getString(PREF_SYSLOG_SERVER, "");
   syslog_server.trim();
   #ifdef SYSLOG_IP
     if (syslog_server.isEmpty())
       syslog_server=String(SYSLOG_IP);
   #endif
   // syslog server configured?
   if (syslog_server.length()) {
     syslog.server(syslog_server.c_str(), 514);
   } else {
     syslog.server(NULL, 0);
     preferences.putString(PREF_SYSLOG_SERVER, "");
   }
   syslog.deviceHostname(webServerCfg->callsign.c_str());
   syslog.appName("TTGO");
   //syslog.defaultPriority(LOG_KERN);
   syslog.defaultPriority(LOG_LOCAL0);
 #endif
  // set Call as default wifi hostname for Wifi
  dnsHostname = String(webServerCfg->callsign.c_str());
  restart_AP_or_STA();

  if (MDNS.begin(webServerCfg->callsign.c_str())) {
    MDNS.setInstanceName(webServerCfg->callsign + " TTGO LoRa APRS TNC " + String(TXFREQ, 4) + "MHz");
    MDNS.addService("http", "tcp", 80);
    #ifdef KISS_PROTOCOL
      MDNS.addService("kiss-tnc", "tcp", NETWORK_TNC_PORT);
    #endif
  }


  webListReceivedQueue = xQueueCreate(4,sizeof(tReceivedPacketData *));
  tReceivedPacketData *receivedPacketData = nullptr;

  uint32_t t_aprsis_last_connect_try = 0L;
  aprs_callsign = webServerCfg->callsign;
  aprsis_host.trim();
  aprsis_filter.trim();
  aprsis_password.trim();
  if (aprsis_callsign.length() < 3 || aprsis_callsign.length() > 9)
    aprsis_callsign = "";
  if (aprsis_callsign.isEmpty()) aprsis_callsign = aprs_callsign;
  if (aprsis_callsign.length() < 3 || aprsis_callsign.length() > 9)
    aprsis_callsign = "";
  aprsis_callsign.toUpperCase(); aprsis_callsign.trim();

  // sanity check
  if (aprsis_enabled) {
    if (aprsis_callsign.isEmpty() || aprsis_host.isEmpty() || aprsis_port == 0) {
      aprsis_enabled = false;
   } else {
      const char *p = aprsis_callsign.c_str();
      for (; *p && aprsis_enabled; p++) {
        if (*p == '-') {
          int len = strlen(p+1);
          if (len < 1 || len > 2)
            aprsis_enabled = false;
        } else if ( !((*p >= 'A' && *p <= 'Z') || (*p >= '0' && *p <= '9')) )
            aprsis_enabled = false;
      }
    }
  }

  uint32_t webserver_started = millis();


  // main loop
  while (true) {

    esp_task_wdt_reset();

    if (apcnt && millis() - webserver_started > 60*1000L &&
         (millis() > 3*60*1000L && (!wifi_do_fallback_to_mode_AP || WiFi.getMode() != WIFI_MODE_AP || WiFi.softAPgetStationNum() < 1) ) ) {
      // If a remote AP is configured and we are in self-AP-mode,
      // try to give up self AP mode and try to reconnect an remote AP again. But only do this,
      // uptime is >3min AND if wifi_do_fallback_to_mode_AP is not allowed AND
      //   we are either in WIFI_MODE_STA OR: in MODE_AP user is currently associated with our self-AP
      // -> If a remoteAP is configured or uptime <3min (grace time for him to be able to reach Webinterface for change config):
      //      If wifi-client is connected:
      //         If wifi_do_fallback_mode is enabled, we'll keep his connection alive.
      //         else: we'll disconnect the wifi-client and search for a remoteAP
      //      else: search for a remoteAP
      //    else:
      //     selfAP will stay on and connections are not interrupted.
      // wifi_do_fallback_to_mode_AP: false: Disable this for igates, where you don't need your tracker to be a hotspot; or if your phone (which knows your tracker's AP SSID) should not mis-interpre te the tracker as Wifi-Hotspot with Internet-Access.
      //                              true: You like to enable, if you use your tracker portable and it should automatically be wifi client to your home network, and be AP if you are outside, and should not search for a remoteAP

      // Mode STA and connection lost? -> reconnect. Or when dhcp renew failed?
      if (WiFi.getMode() == WIFI_MODE_STA) {
        if (WiFi.status() != WL_CONNECTED || WiFi.localIP() == IP_NULL || WiFi.subnetMask() == IP_SUBNET_NULL || WiFi.gatewayIP() == IP_GATEWAY_NULL) {
          static uint32_t last_connection_attempt = millis();
          if (aprsis_client.connected()) aprsis_client.stop();
          if (millis() - last_connection_attempt > 20000L) {
            esp_task_wdt_reset();
            restart_AP_or_STA();
            webserver_started = millis();
            esp_task_wdt_reset();
            last_connection_attempt = millis();
          }
        }
      } else {
        if (aprsis_client.connected()) aprsis_client.stop();
        restart_AP_or_STA();
        webserver_started = millis();
      }
    }

    server.handleClient();
    if (xQueueReceive(webListReceivedQueue, &receivedPacketData, (1 / portTICK_PERIOD_MS)) == pdPASS) {
      auto *receivedPacketToQueue = new tReceivedPacketData();
      receivedPacketToQueue->packet = new String();
      receivedPacketToQueue->packet->concat(*receivedPacketData->packet);
      receivedPacketToQueue->RSSI = receivedPacketData->RSSI;
      receivedPacketToQueue->SNR = receivedPacketData->SNR;
      receivedPacketToQueue->QRG = receivedPacketData->QRG;
      receivedPacketToQueue->rxTime = receivedPacketData->rxTime;
      receivedPackets.push_back(receivedPacketToQueue);
      if (receivedPackets.size() > MAX_RECEIVED_LIST_SIZE){
        auto *packetDataToDelete = receivedPackets.front();
        delete packetDataToDelete->packet;
        delete packetDataToDelete;
        receivedPackets.pop_front();
      }
      delete receivedPacketData->packet;
      delete receivedPacketData;
    }


    if (aprsis_enabled) {
      if (WiFi.getMode() == WIFI_MODE_STA) {
        String log_msg;

        if (WiFi.status() == WL_CONNECTED) {

          if (!aprsis_client.connected()) {
            if (t_aprsis_last_connect_try + 10000 > millis()) {
              if (aprsis_status == "Error: no internet") {
                aprsis_status = "Internet unavailable";
                // inform about state change
                log_msg = String("APRS-IS: ") + aprsis_status;
                #if defined(ENABLE_SYSLOG)
                  syslog_log(LOG_INFO, log_msg);
                #endif
                do_serial_println(log_msg);
              }
            } else {
              int ret;
              t_aprsis_last_connect_try = millis();
              // connect to aprsis
              // try to connect to aprsis and login
              if ((ret = connect_to_aprsis()) < 0) {
                log_msg = String("APRS-IS: on_Err: '") + aprsis_status + String("' [") + aprsis_client.remoteIP().toString() + String("], tries ") +  String(aprsis_connect_tries) + String(", connect_to_aprsis() returned ") + String(ret);
                aprsis_client.stop();
                if (!enable_bluetooth)
                  WiFi.setSleep(true);
                // Known problems which usually resolve by reboot:
                if (ret == -5 /* login denied, until reboot. Reason unknown */ ||
                     (ret == -1 /* sometimes after boot it can't connect. DNS- or IP-stack Problem? */ && lora_digipeating_mode > 1) /* we are a digi */ ) {

                  // sometimes after boot, connection does not work:
                  //   APRS-IS: connecting to 'aprs.hamnet.cloud', tries 1
                  //   [ 10068][E][WiFiClient.cpp:268] connect(): socket error on fd 50, errno: 104, "Connection reset by peer"
                  //   APRS-IS: on_Err: 'Error: connect failed' [0.0.255.0], tries 1
                  // connection retry intervall is every 10s. -> In 5min, tries == 30.
                  // -> Q&D fix: Restart WIFI.
                  if (!(aprsis_connect_tries % 30)) {
                    //log_msg = log_msg + ". Restarted WIFI!";
                    //restart_AP_or_STA();
                    // ^does not help
                    // first thing we now try: trigger watchdog for taskWebserver thread.
                    // we had configured watchdog timeout to 120s.
                    String m = log_msg + ". connect bug? - Restarting webserver the soft way, by watchdog timer expiry";
                    #if defined(ENABLE_SYSLOG)
                      syslog_log(LOG_CRIT, m);
                    #endif
                    do_serial_println(m);
                    delay(3*60*1000);
                    // wdt reseted? Then we'll not have survived to be not here. -> next try: reboot
                    m = "APRS-IS: rebooting with ESP.restart()";
                    #if defined(ENABLE_SYSLOG)
                      syslog_log(LOG_CRIT, m);
                    #endif
                    do_serial_println(m);
                    delay(2000);
                    ESP.restart();
                  }
                }
              }

              if (log_msg.length()) {
                #if defined(ENABLE_SYSLOG)
                  syslog_log(LOG_INFO, log_msg);
                #endif
                do_serial_println(log_msg);
              }
              if (!aprsis_status.startsWith("Error: "))
                aprsis_status = "Disconnected";
            }
          }

          // session died during read / write?
          if (aprsis_client.connected()) {
            // read packets from APRSIS
            read_from_aprsis();
          }

          // session died during read / write? - log
          if (aprsis_client.connected()) {
            // forward packets to APRSIS
            send_queue_to_aprsis();
          }

          // anti-idle timer for the tcp session
          if (aprsis_client.connected() && millis() - t_aprsis_lastRXorTX > 2*60*1000L) {
            aprsis_client.print("#\r\n");
            t_aprsis_lastRXorTX = millis();
          }

        } else {

          if (!enable_bluetooth)
            WiFi.setSleep(true);

          if (aprsis_status != "Error: no internet") {
            aprsis_status = "Error: no internet";
            // inform about state change
            log_msg = String("APRS-IS: ") + aprsis_status;
            #if defined(ENABLE_SYSLOG)
              syslog_log(LOG_INFO, log_msg);
            #endif
            do_serial_println(log_msg);
          }

        } // WiFi.status != WL_CONNECTED


      } else {

        aprsis_status = "Error: no internet";
      } // WiFi.getMode() == WIFI_MODE_STA

    } // aprsis_enabled

    vTaskDelay(5/portTICK_PERIOD_MS);
  }
}

#endif // ENABLE_WIFI
