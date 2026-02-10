//
// This monitors the operation of the well house
//
#include <Adafruit_ADS1X15.h> // from package "Adafruit ADS1X15" by Adafruit
#include <WiFi.h>
#include <Wire.h>
#include <WebServer.h>
#include <Update.h>
#include "html.h"
#include "street_cred.h"

#define MSG_SIZE_MAX    16
#define FLOAT_SIZE_MAX  8
#define MAX_WIFI_WAIT   10

// the builtin led is active low for some dipshit reason
#define ON  HIGH
#define OFF LOW

#define ADS_SDA  21
#define ADS_SCL  22


const char* host = "optiplex";
const uint16_t port = 27910;
bool remote_control_inited = false;

int16_t  a0_a1;
float prev_mv = 0.0f;
float mv = 0.0f;

float current_rms = 0.0f;

int prev_vector = 0;
int vector = 0;

int delta = 0;

// WiFiClient client;
char msg[MSG_SIZE_MAX];
char tempFloat[FLOAT_SIZE_MAX];

int led_timer = 0;
bool led_on = false;

Adafruit_ADS1115 ads;

WebServer web_server(80);

#define SYS_STATUS_PAGE_STR_LEN 2560
char httpStr[256] = {0};
char systemStatusPageStr[SYS_STATUS_PAGE_STR_LEN];
uint8_t wifi_disconnect_reason = 0;
char current_time[41];

float multiplier = 0.0625f;

// Converts milliseconds into natural language
void millisToDaysHoursMinutes(unsigned long milliseconds, char* str, int length)
{
  uint seconds = milliseconds / 1000;
  memset(str, 0, length);

  if (seconds <= 60) {
    // It's only been a few seconds
    // Longest string example, 11 chars: 59 seconds\0
    snprintf(str, 11, "%d second%s", seconds, seconds == 1 ? "" : "s");
    return;
  }
  uint minutes = seconds / 60;
  if (minutes <= 60) {
    // It's only been a few minutes
    // Longest string example, 11 chars: 59 minutes\0
    snprintf(str, 11, "%d minute%s", minutes, minutes == 1 ? "" : "s");
    return;
  }
  uint hours = minutes / 60;
  minutes -= hours * 60;
  if (hours <= 24) {
    // It's only been a few hours
    if (minutes == 0)
      // Longest string example, 9 chars: 23 hours\0
      snprintf(str, 9, "%d hour%s", hours, hours == 1 ? "" : "s");
    else
      // Longest string example, 24 chars: 23 hours and 59 minutes\0
      snprintf(str, 24, "%d hour%s and %d minute%s", hours, hours == 1 ? "" : "s", minutes, minutes == 1 ? "" : "s");
    return;
  }

  // It's been more than a day
  uint days = hours / 24;
  hours -= days * 24;
  if (minutes == 0)
    // Longest string example, 23 chars: 9999 days and 23 hours\0
    snprintf(str, 23, "%d day%s and %d hour%s", days, days == 1 ? "" : "s", hours, hours == 1 ? "" : "s");
  else
    // Longest string example, 35 chars: 9999 days, 23 hours and 59 minutes\0
    snprintf(str, 35, "%d day%s, %d hour%s and %d minute%s", days, days == 1 ? "" : "s", hours, hours == 1 ? "" : "s", minutes, minutes == 1 ? "" : "s");
}

static const char* wifi_reason_str(uint8_t r) {
  switch (r) {
    case 0:                                   return "It hasn't. yet.";
    case WIFI_REASON_AUTH_EXPIRE:             return "AUTH_EXPIRE";
    case WIFI_REASON_AUTH_LEAVE:              return "AUTH_LEAVE";
    case WIFI_REASON_ASSOC_EXPIRE:            return "ASSOC_EXPIRE";
    case WIFI_REASON_ASSOC_TOOMANY:           return "ASSOC_TOOMANY";
    case WIFI_REASON_NOT_AUTHED:              return "NOT_AUTHED";
    case WIFI_REASON_NOT_ASSOCED:             return "NOT_ASSOCED";
    case WIFI_REASON_ASSOC_LEAVE:             return "ASSOC_LEAVE";
    case WIFI_REASON_ASSOC_NOT_AUTHED:        return "ASSOC_NOT_AUTHED";
    case WIFI_REASON_DISASSOC_PWRCAP_BAD:     return "DISASSOC_PWRCAP_BAD";
    case WIFI_REASON_DISASSOC_SUPCHAN_BAD:    return "DISASSOC_SUPCHAN_BAD";
    case WIFI_REASON_BSS_TRANSITION_DISASSOC: return "BSS_TRANSITION_DISASSOC";
    case WIFI_REASON_IE_INVALID:              return "IE_INVALID";
    case WIFI_REASON_MIC_FAILURE:             return "MIC_FAILURE";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:  return "4WAY_HANDSHAKE_TIMEOUT";
    case WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT:return "GROUP_KEY_UPDATE_TIMEOUT";
    case WIFI_REASON_IE_IN_4WAY_DIFFERS:      return "IE_IN_4WAY_DIFFERS";
    case WIFI_REASON_GROUP_CIPHER_INVALID:    return "GROUP_CIPHER_INVALID";
    case WIFI_REASON_PAIRWISE_CIPHER_INVALID: return "PAIRWISE_CIPHER_INVALID";
    case WIFI_REASON_AKMP_INVALID:            return "AKMP_INVALID";
    case WIFI_REASON_UNSUPP_RSN_IE_VERSION:   return "UNSUPP_RSN_IE_VERSION";
    case WIFI_REASON_INVALID_RSN_IE_CAP:      return "INVALID_RSN_IE_CAP";
    case WIFI_REASON_802_1X_AUTH_FAILED:      return "802_1X_AUTH_FAILED";
    case WIFI_REASON_CIPHER_SUITE_REJECTED:   return "CIPHER_SUITE_REJECTED";
    case WIFI_REASON_BEACON_TIMEOUT:          return "BEACON_TIMEOUT";
    case WIFI_REASON_NO_AP_FOUND:             return "NO_AP_FOUND";
    case WIFI_REASON_AUTH_FAIL:               return "AUTH_FAIL";
    case WIFI_REASON_ASSOC_FAIL:              return "ASSOC_FAIL";
    case WIFI_REASON_HANDSHAKE_TIMEOUT:       return "HANDSHAKE_TIMEOUT";
    case WIFI_REASON_CONNECTION_FAIL:         return "CONNECTION_FAIL";
    case WIFI_REASON_AP_TSF_RESET:            return "AP_TSF_RESET";
    case WIFI_REASON_ROAMING:                 return "ROAMING";
    default: return "UNKNOWN";
  }
}



char* getSystemStatus()
{
  String html;
  // Pardon the html mess. Gotta tell the browser to not make the text super tiny.
  html = "<!DOCTYPE html><html><head><title>Driveway Lights</title></head><body><p style=\"font-size:36px\">";
  html += "<span style=\"font-size:90px\">";

  // Longest string example, 82 chars: Notifications are <span id='lights_span' style="color:Green;">ON</span>
  snprintf(httpStr, 100, "RSSI: %d, last disconnect reason: <span style=\"color:Green;\">%s</span>", WiFi.RSSI(), wifi_reason_str(wifi_disconnect_reason));
  html += httpStr;
  html += "</br>";
  millisToDaysHoursMinutes(millis(), current_time, 40);
  snprintf(httpStr, 60, "Uptime: %s", current_time);
  html += httpStr;
  html += "</br>";
  snprintf(httpStr, 60, "rms: %f", current_rms);
  html += httpStr;
  html += "</span></br>";
  
  // Close it off
  html += "</p></body></html>";

  memset(systemStatusPageStr, 0, SYS_STATUS_PAGE_STR_LEN);
  html.toCharArray(systemStatusPageStr, html.length() + 1);
  return systemStatusPageStr;
}


void init_remote_control() {
  
  if (remote_control_inited) return;

  web_server.on("/", HTTP_GET, []() {
    web_server.sendHeader("Connection", "close");
    web_server.send(200, "text/html", getSystemStatus());
  });
  // web_server.on("/toggle_mute", HTTP_POST, []() {
  //   char notificationsMuted = EEPROM.read(EEPROM_MUTE_NOTIFICATIONS_BYTE);
  //   notificationsMuted = notificationsMuted == 1 ? 0 : 1;
  //   EEPROM.write(EEPROM_MUTE_NOTIFICATIONS_BYTE, (byte)notificationsMuted);
  //   EEPROM.commit();
  //   web_server.send(200, "text/plain", notificationsMuted == 1 ? "Turn On" : "Turn Off");
  // });
  web_server.on("/update", HTTP_GET, []() {
    web_server.sendHeader("Connection", "close");
    web_server.send(200, "text/html", update_html);
  });
  /*handling uploading firmware file */
  web_server.on("/update_backend", HTTP_POST, []() {
    web_server.sendHeader("Connection", "close");
    web_server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = web_server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  web_server.begin();
  Serial.println("Web server initialized");
  remote_control_inited = true;
}



bool connectToWifi()
{
  int wifiRetries = 0;
  WiFi.mode(WIFI_STA);
  Serial.print("WiFi is down. Connecting");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && wifiRetries < MAX_WIFI_WAIT) {
    delay(1000);
    wifiRetries++;
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
     Serial.println("WiFi failed to connect");
  }
  return false;
}

unsigned long reconnect_interval = 1000;
void reconnect_wifi()
{
  static unsigned long prev_millis = 0;
  static bool led_state = false;
  unsigned long current_millis = millis();
  if(current_millis - prev_millis >= reconnect_interval) {
    prev_millis = current_millis;
    Serial.print("WiFi is down. Connecting to ");
    Serial.println(ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    led_state = !led_state;
    digitalWrite(LED_BUILTIN, led_state);
  }
}

void wifi_event(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED)
    wifi_disconnect_reason = info.wifi_sta_disconnected.reason;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, OFF);

  Serial.begin(115200);
  delay(1000);
  Serial.println("Hello");
  Serial.print("My mac address is: ");
  Serial.println(WiFi.macAddress());

  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  if(connectToWifi()) init_remote_control();

  WiFi.onEvent(wifi_event);

  Wire.begin(ADS_SDA, ADS_SCL);
  ads.begin();

  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  // The above is taken from https://github.com/GreenPonik/Adafruit_ADS1X15/blob/7fce1e43c48ae32c40f806362060d91b34de3318/examples/differential/differential.pde
  // The output of the SCT-013-000V (which I'm pretty sure is what i have) will be between 0V-1V.
  // 1V means 100A which we should never see, BUT, anything can happen sometimes. i don't wanna damage my ADC.
  // So go with GAIN_TWO. That should keep us safe.
  ads.setGain((adsGain_t)GAIN_TWO);
}



void loop() {

  if (WiFi.status() != WL_CONNECTED) {
    // wifi died. try to reconnect
    reconnect_wifi();
  } else {
    if (remote_control_inited) {
      web_server.handleClient();
      // MDNS.update();
    } else {
      init_remote_control();
    }
  }

  // if (led_timer > 0) {
  //   if (!led_on) {
  //     digitalWrite(LED_BUILTIN, ON);
  //     led_on = true;
  //   }
  //   led_timer--;
  // } else {
  //   if (led_on) {
  //     digitalWrite(LED_BUILTIN, OFF);
  //     led_on = false;
  //   }
  // }

  delay(8);

  // Read current
  a0_a1 = ads.readADC_Differential_0_1();

  if (a0_a1 != 0 && a0_a1 != -1) {
    // Because you're going to come back in here years later and not know wth this is doing, here's a bone.
    // Remember that a0_a1 is a reading of the differential voltage between A0 and A1 of the ADC.
    // We set the gain at "GAIN_TWO", which means the adc is reading voltage between +2.048V and -2.048V,
    // at 16 bits of resolution (65535 possible values). That's a full peak to peak range of (2.048 * 2 = 4.096).
    // 4.096 / 65535 = .0625. So 16 bits can tell us a value between +-2.048v within .0625v of accuracy.
    // To calculate the actual voltage value, you can think of it like divisions on an oscilliscope.
    // Whatever it spits out, you have to multiply it by whatever each division represents.
    // In our case, .0625. If you change the gain in the future, you gotta see what that full pk2pk range is,
    // divide it by the resolution of the adc (16 bits [65535] for the ADS 1115), and use that as your 'multiplier'.
    mv = a0_a1 * multiplier;
    delta = mv - prev_mv;
    vector = delta > 0 ? 1 : -1;

    if (vector == -1 && prev_vector == 1) {
      // Voltage is dropping from its positive peak.
      // This means the last mv value is the peak.
      // Calculate rms of the peak voltage. Keep it simple.
      // prev_mv is in millivolts, so divide by 1000 to turn it back into whole Volts.
      // Then x100 because the SCT-013-000V puts out 1V per 100A.
      // Then x.707 to get rough rms.
      current_rms = prev_mv / 1000 * 100 * 0.707f;
      // if (current_rms > 0 && WiFi.status() == WL_CONNECTED) {
      //   if (client.connect(host, port)) {
      //     memset(tempFloat, 0, FLOAT_SIZE_MAX);
      //     memset(msg, 0, MSG_SIZE_MAX);
      //     dtostrf(current_rms, 3, 2, tempFloat);
      //     snprintf(msg, MSG_SIZE_MAX, "sp:%s", tempFloat);
      //     if (client.connected()) { client.println(msg); }
      //     //Serial.println(msg);
      //     client.stop();
      //   } else {
      //     //Serial.println("connection failed");
      //     delay(500);
      //   }
      // }
      led_timer = 20;
    }
    prev_vector = vector;
    prev_mv = mv;
  }
}
