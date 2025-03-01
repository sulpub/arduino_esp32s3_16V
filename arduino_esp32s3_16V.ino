#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <time.h>

// ---- CONFIG I2C ----
#define PCF1_ADDR 0x21  // PCF8574 - Sorties 1 à 8
#define PCF2_ADDR 0x25  // PCF8574 - Sorties 9 à 16
uint8_t pcf1_state = 0x00;
uint8_t pcf2_state = 0x00;

// ---- CONFIG WIFI & MQTT ----
const char* ssid = "****";           // SSID WIFI
const char* password = "****";       //PASS WIFI
const char* mqtt_server = "****";    // IP MQTT
const char* mqtt_user = "****";      // Identifiant MQTT
const char* mqtt_password = "****";  // Mot de passe MQTT
const char* mqtt_topic = "esp32s3/timer/#";
const char* mqtt_status_topic = "esp32s3/status";

//test trame json
//topic : esp32s3/timer/D0EF76196278
//{"start": 0,"week": "1111111","during": 30,"delay":  30,"repeat": 24,"output": 7}
//https://m.convert-me.com/fr/convert/time/dhms/dhms-to-millisecond.html

WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;

// ---- CONFIG NTP ----
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 3600;

// ---- CONFIG WATCHDOG ----
#define WDT_TIMEOUT 10  // Temps max sans reset du watchdog (10s)

// ---- CONFIG GÉNÉRALE ----
#define NUM_OUTPUTS 16
bool stopAll = false;

// ---- STRUCTURE DES TIMERS ----
struct TimerConfig {
  int start_time;
  String week;
  int during;
  int delay;
  int repeat;
  int output;
  unsigned long next_event_time;
  int cycle_count;
  bool isActive;
};

// ---- STOCKAGE DES TIMERS ----
TimerConfig timers[NUM_OUTPUTS];

// ---- FONCTIONS ----
void connectWiFi() {
  Serial.print("Connexion à WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnecté !");
}

void connectMQTT() {
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  while (!client.connected()) {
    Serial.print("Connexion MQTT...");
    if (client.connect("ESP32S3Client", mqtt_user, mqtt_password)) {
      Serial.println("Connecté !");
      client.subscribe(mqtt_topic);
    } else {
      Serial.print("Échec, rc=");
      Serial.print(client.state());
      Serial.println(" Retente dans 5s...");
      delay(5000);
    }
  }
}

void syncNTP() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Erreur NTP !");
  } else {
    Serial.println("Temps synchronisé !");
  }
}

void saveConfigToFlash() {
  preferences.begin("timer_config", false);
  for (int i = 0; i < NUM_OUTPUTS; i++) {
    String key = "timer_" + String(i);
    preferences.putBytes(key.c_str(), &timers[i], sizeof(TimerConfig));
  }
  preferences.end();
}

void loadConfigFromFlash() {
  preferences.begin("timer_config", true);
  for (int i = 0; i < NUM_OUTPUTS; i++) {
    String key = "timer_" + String(i);
    if (preferences.isKey(key.c_str())) {
      preferences.getBytes(key.c_str(), &timers[i], sizeof(TimerConfig));
    }
  }
  preferences.end();
}

// ---- FONCTIONS DE GESTION DES PCF8574 ----
void writePCF(uint8_t address, uint8_t state) {
  Wire.beginTransmission(address);
  Wire.write(state);
  Wire.endTransmission();
}

void updateOutputs() {
  writePCF(PCF1_ADDR, pcf1_state);
  writePCF(PCF2_ADDR, pcf2_state);
}

void setOutputState(int output, bool state) {
  int bitPos = (output - 1) % 8;
  if (output <= 8) {
    if (state) pcf1_state |= (1 << bitPos);
    else pcf1_state &= ~(1 << bitPos);
  } else {
    if (state) pcf2_state |= (1 << bitPos);
    else pcf2_state &= ~(1 << bitPos);
  }
  updateOutputs();
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("Message MQTT reçu !");

  StaticJsonDocument<256> doc;
  deserializeJson(doc, payload);

  if (doc.containsKey("stopall")) {
    stopAll = doc["stopall"] == 1;
    if (stopAll) {
      pcf1_state = 0x00;
      pcf2_state = 0x00;
      updateOutputs();
    }
    return;
  }

  int outputIndex = String(topic).substring(14).toInt();
  if (outputIndex < 0 || outputIndex >= NUM_OUTPUTS) return;

  timers[outputIndex].start_time = doc["start"];
  timers[outputIndex].week = doc["week"].as<String>();
  timers[outputIndex].during = doc["during"];
  timers[outputIndex].delay = doc["delay"];
  timers[outputIndex].repeat = doc["repeat"];
  timers[outputIndex].output = doc["output"];
  timers[outputIndex].cycle_count = 0;
  timers[outputIndex].isActive = true;
  timers[outputIndex].next_event_time = millis();

  saveConfigToFlash();
}

void processTimers() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return;


  int current_min = timeinfo.tm_hour * 60 + timeinfo.tm_min;
  int current_day = timeinfo.tm_wday;

  for (int i = 0; i < NUM_OUTPUTS; i++) {
    if (!timers[i].isActive || stopAll) continue;

    if (timers[i].week[current_day] == '1' && timers[i].cycle_count < timers[i].repeat) {
      if (millis() >= timers[i].next_event_time) {
        setOutputState(timers[i].output, HIGH);
        client.publish(mqtt_status_topic, ("{\"output\":" + String(timers[i].output) + ", \"state\":1}").c_str());
        timers[i].next_event_time = millis() + (timers[i].during * 60000);
        Serial.print(("{\"output\":" + String((timers[i].output)) + ", \"state\":0}").c_str());
        Serial.print(" -cur ");
        Serial.print(current_min);
        Serial.print(" -day ");
        Serial.print(current_day);
        Serial.print(" -nxt ");
        Serial.println((timers[i].next_event_time / 60000));

        delay(50);
        setOutputState(timers[i].output, LOW);
        client.publish(mqtt_status_topic, ("{\"output\":" + String(timers[i].output) + ", \"state\":0}").c_str());
        timers[i].next_event_time = millis() + (timers[i].delay * 60000);
        timers[i].cycle_count++;
        Serial.print(("{\"output\":" + String((timers[i].output)) + ", \"state\":0}").c_str());
        Serial.print(" -cur ");
        Serial.print(current_min);
        Serial.print(" -day ");
        Serial.print(current_day);
        Serial.print(" -nxt ");
        Serial.println((timers[i].next_event_time / 60000));
        delay(50);
      }
    }
  }
}

// ---- SETUP ----
void setup() {
  Serial.begin(115200);
  Wire.begin();

  connectWiFi();
  syncNTP();
  loadConfigFromFlash();
  connectMQTT();

  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
}

// ---- BOUCLE PRINCIPALE ----
void loop() {
  if (!client.connected()) connectMQTT();
  client.loop();
  processTimers();
  esp_task_wdt_reset();
}
