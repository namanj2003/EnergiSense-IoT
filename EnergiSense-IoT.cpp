#define BLYNK_TEMPLATE_ID "{Your Template ID Here}"
#define BLYNK_TEMPLATE_NAME "{Your Template Name Here}"
#define BLYNK_PRINT Serial
#include <LiquidCrystal_I2C.h>
#include "EmonLib.h"
#include <EEPROM.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <BlynkSimpleEsp32.h>
#include <time.h>
#include <WiFiManager.h>
#include <Ticker.h>
#include <ArduinoJson.h>

#define RESET_PIN 26
#define LED_GREEN 14
#define LED_RED 13
#define LED_BLUE 12
#define EEPROM_START_ADDR 100
#define vCalibration 1.0 
#define currCalibration 0.50
#define FILTER_SIZE 5

LiquidCrystal_I2C lcd(0x27, 20, 4);
EnergyMonitor emon;
BlynkTimer timer;
WiFiManager wifiManager;
Ticker ledBlinker;

bool ledState = false;
char auth[] = "{Your Auth Token Here}";
unsigned long lastSendTime = 0;
float voltageReadings[FILTER_SIZE];
int filterIndex = 0;
float voltageSum = 0.0;
float kWh = 0;
unsigned long lastmillis = millis();

String getDateTimeIST() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return "";
  }
  char datetime[30];
  strftime(datetime, sizeof(datetime), "%m/%d/%Y, %I:%M:%S %p", &timeinfo);
  return String(datetime);
}

enum WiFiState {
  WIFI_SETUP,
  WIFI_CONNECTED
};
WiFiState wifiState = WIFI_SETUP;

const unsigned long WIFI_RECONNECT_TIMEOUT = 30000;
unsigned long wifiDisconnectedTime = 0;

void checkResetButton() {
  static bool prevWifiStatus = false;
  bool currentWifiStatus = WiFi.status() == WL_CONNECTED;

  if (digitalRead(RESET_PIN) == LOW) {
    Serial.println("Reset button pressed. Clearing WiFi data and restarting...");
    wifiManager.resetSettings();
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, LOW);
    ESP.restart();
  } else if (currentWifiStatus) {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_RED, LOW);
    wifiDisconnectedTime = 0;
  } else {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, !digitalRead(LED_RED));

    if (prevWifiStatus && !currentWifiStatus)
      if (wifiDisconnectedTime == 0) {
        wifiDisconnectedTime = millis();
      } else if (millis() - wifiDisconnectedTime >= WIFI_RECONNECT_TIMEOUT) {
        Serial.println("WiFi connection lost for a long time. Restarting in setup mode...");
        wifiManager.resetSettings();
        wifiState = WIFI_SETUP;
        ESP.restart();
      }
    }
  }
  prevWifiStatus = currentWifiStatus;
}

const unsigned long DATA_SAVE_INTERVAL = 60000;
unsigned long lastDataSaveTime = 0;

void saveDataToEEPROM() {
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long currentMillis = millis();
    int addr = EEPROM_START_ADDR;

    EEPROM.put(addr, emon.Vrms);
    addr += sizeof(float);
    EEPROM.put(addr, emon.Irms);
    addr += sizeof(float);
    EEPROM.put(addr, emon.apparentPower);
    addr += sizeof(float);
    EEPROM.put(addr, kWh);
    addr += sizeof(float);

    for (int i = 0; i < strlen(auth); ++i) {
      EEPROM.put(addr, auth[i]);
      addr++;
    }
    EEPROM.put(addr, '\0');
    addr++;

    String currentTimestamp = getDateTimeIST();
    for (int i = 0; i < currentTimestamp.length(); ++i) {
      EEPROM.put(addr, currentTimestamp[i]);
      addr++;
    }
    EEPROM.put(addr, '\0');

    lastDataSaveTime = currentMillis;
  }
}

void sendHistoricalDataFromEEPROM() {
  if (WiFi.status() == WL_CONNECTED) {
    int addr = EEPROM_START_ADDR;
    while (addr < EEPROM.length()) {
      float v0, v1, v2, v3;
      char deviceID[30];
      char timestamp[30];
]
      EEPROM.get(addr, v0);
      addr += sizeof(float);
      EEPROM.get(addr, v1);
      addr += sizeof(float);
      EEPROM.get(addr, v2);
      addr += sizeof(float);
      EEPROM.get(addr, v3);
      addr += sizeof(float);

      int i = 0;
      char ch;
      do {
        EEPROM.get(addr, ch);
        deviceID[i++] = ch;
        addr++;
      } while (ch != '\0');

      i = 0;
      do {
        EEPROM.get(addr, ch);
        timestamp[i++] = ch;
        addr++;
      } while (ch != '\0');

      StaticJsonDocument<200> doc;
      doc["v0"] = v0;
      doc["v1"] = v1;
      doc["v2"] = v2;
      doc["v3"] = v3;
      doc["deviceID"] = deviceID;
      doc["timeStamp"] = timestamp;

      String payload;
      serializeJson(doc, payload);

      HTTPClient http;
      http.begin("https://namanjain.me/historydata-send");
      http.addHeader("Content-Type", "application/json");
      int httpResponseCode = http.POST(payload);
      if (httpResponseCode > 0) {
        Serial.print("HTTP Response code eeprom saved Data sent: ");
        Serial.println(httpResponseCode);
        int addr = EEPROM_START_ADDR;
        while (addr < EEPROM.length()) {
          EEPROM.write(addr, 0);
          addr++;
        }
      } else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      http.end();
    }
  }
}

void myTimerEvent() {
emon.calcVI(20, 2000);
  kWh = kWh + emon.apparentPower * (millis() - lastmillis) / 3600000.0;

  float voltage = emon.Vrms;
  voltageSum += voltage;
  filterIndex++;
  if (filterIndex >= FILTER_SIZE) {
    filterIndex = 0;
  }
  voltageReadings[filterIndex] = voltage;
  float averageVoltage = voltageSum / FILTER_SIZE;
  emon.Vrms = averageVoltage;

  yield();
  Serial.print("Vrms: ");
  Serial.print(emon.Vrms, 3);
  Serial.print("V");
  delay(100);

  Serial.print("\tIrms: ");
  Serial.print(emon.Irms, 3);
  Serial.print("A");
  delay(100);

  Serial.print("\tPower: ");
  Serial.print(emon.apparentPower, 3);
  Serial.print("W");
  delay(100);

  Serial.print("\tkWh: ");
  Serial.print(kWh, 3);
  Serial.println("kWh");
  delay(100);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Vrms:");
  lcd.print(emon.Vrms, 3);
  lcd.print("V");
  lcd.setCursor(0, 1);
  lcd.print("Irms:");
  lcd.print(emon.Irms, 3);
  lcd.print("A");
  lcd.setCursor(0, 2);
  lcd.print("Power:");
  lcd.print(emon.apparentPower, 3);
  lcd.print("W");
  lcd.setCursor(0, 3);
  lcd.print("kWh:");
  lcd.print(kWh, 3);
  lcd.print("W");
  lastmillis = millis();

  Blynk.virtualWrite(V0, round(emon.Vrms * 1000.0) / 1000.0);
  Blynk.virtualWrite(V1, round(emon.Irms * 1000.0) / 1000.0);
  Blynk.virtualWrite(V2, round(emon.apparentPower * 1000.0) / 1000.0);
  Blynk.virtualWrite(V3, round(kWh * 1000.0) / 1000.0);

  if (millis() - lastSendTime >= 60000) {
    HTTPClient http;
    http.begin("https://namanjain.me/historydata-send");
    http.addHeader("Content-Type", "application/json");

    StaticJsonDocument<200> doc;
    doc["v0"] = round(emon.Vrms * 1000.0) / 1000.0;
    doc["v1"] = round(emon.Irms * 1000.0) / 1000.0;
    doc["v2"] = round(emon.apparentPower * 1000.0) / 1000.0;
    doc["v3"] = round(kWh * 1000.0) / 1000.0;
    String timestamp = getDateTimeIST();
    doc["timeStamp"] = timestamp;
    doc["deviceID"] = String(auth);

    String payload;
    serializeJson(doc, payload);

    Serial.print(payload);
    int httpResponseCode = http.POST(payload);
    if (httpResponseCode > 0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
    } else {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
    }
    http.end();
    lastSendTime = millis();
  }
}

void setup() {
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(RESET_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  emon.voltage(35, vCalibration, 1.7);
  emon.current(34, currCalibration);
  timer.setInterval(5000, myTimerEvent);
  lcd.setCursor(7, 0);
  lcd.print("Welcome");
  lcd.setCursor(9, 1);
  lcd.print("To");
  lcd.setCursor(5, 2);
  lcd.print("EnergiSense");
  lcd.setCursor(5, 3);
  lcd.print("Smart Meter");
  delay(3000);
  lcd.clear();
  configTime(5.5 * 3600, 0, "pool.ntp.org", "time.nist.gov");
}

void loop() {
  unsigned long currentMillis = millis();

  if (digitalRead(RESET_PIN) == LOW) {
    Serial.println("Reset button pressed. Clearing WiFi data and restarting...");
    wifiManager.resetSettings();
    ESP.restart();
  }

  switch (wifiState) {
    case WIFI_SETUP:
      ledBlinker.attach_ms(1000, []() {
        digitalWrite(LED_RED, !digitalRead(LED_RED));
      });

      if (wifiManager.autoConnect("{Give a name to your device AP here}", "{Password here}")) {
        Serial.println("Connected to WiFi!");
        Blynk.config(auth);
        wifiState = WIFI_CONNECTED;
        ledBlinker.detach();
      } else {
        Serial.println("Failed to connect to WiFi!");
      }
      break;

    case WIFI_CONNECTED:
      checkResetButton();
      Blynk.run();
      timer.run();
      if (currentMillis - lastDataSaveTime >= DATA_SAVE_INTERVAL) {
        saveDataToEEPROM();
        lastDataSaveTime = currentMillis;
      }
      sendHistoricalDataFromEEPROM();
      break;
  }
}
