#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>

// ========= WIFI =========
const char* ssid = "Ãñshùmàñ Vìshwákãrmå";
const char* password = "244466666";

// ========= MQTT =========
const char* mqtt_server = "broker.hivemq.com";
WiFiClient espClient;
PubSubClient client(espClient);

// ========= PINS =========
#define VOLTAGE_PIN 34
#define CURRENT_PIN 35
#define RELAY_PIN   26

// ========= CALIBRATION =========
float voltageCalibration = 645.0;
float currentCalibration = 7.8125;

// ========= VARIABLES =========
int voltageOffset = 0;
int currentOffset = 0;

// ========= PROTECTION =========
float OVER_VOLTAGE = 300.0;
float UNDER_VOLTAGE = 180.0;
float OVER_CURRENT = 5.0;
int SPIKE_THRESHOLD = 1200;

// ========= RELAY =========
bool faultActive = false;
bool relayState = true;
bool relayChanged = true;

// ========= TIMING =========
unsigned long startupDelay = 5000;
unsigned long tripTime = 0;
unsigned long lastPublish = 0;

int recloseAttempts = 0;
int maxAttempts = 2;
unsigned long recloseDelay = 3000;

unsigned long publishInterval = 500;

// ========= WIFI =========
void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
}

// ========= OFFSET =========
int getOffset(int pin) {
  long sum = 0;
  for (int i = 0; i < 1000; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200);
  }
  return sum / 1000;
}

// ========= MQTT =========
void reconnect() {

  while (!client.connected()) {

    Serial.println("Connecting MQTT...");

    // UNIQUE CLIENT ID
    String clientId = "ESP32_" + String((uint32_t)ESP.getEfuseMac(), HEX);

    if (client.connect(clientId.c_str())) {

      Serial.println("MQTT Connected");

      client.subscribe("scada/relay1");

      relayChanged = true;
      client.publish("scada/esp32/status1", "STARTING", true);

    } else {
      Serial.print("Failed, rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

// ========= CALLBACK =========
void callback(char* topic, byte* payload, unsigned int length) {

  String msg = "";
  for (int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  Serial.print("Received: ");
  Serial.print(topic);
  Serial.print(" -> ");
  Serial.println(msg);

  if (strcmp(topic, "scada/relay1") == 0) {

    if (msg == "ON") {
      digitalWrite(RELAY_PIN, LOW);
      relayState = true;
      relayChanged = true;
    } 
    else if (msg == "OFF") {
      digitalWrite(RELAY_PIN, HIGH);
      relayState = false;
      relayChanged = true;
    }
  }
}

// ========= CALCULATION =========
void calculateValues(float &Vrms, float &Irms, float &realPower, float &pf) {

  const int samples = 800;
  const float vRef = 3.3;
  const int adcMax = 4095;

  float sumV = 0, sumI = 0, sumP = 0;

  // Convert stored offsets to voltage
  float vOffsetVolt = (voltageOffset / (float)adcMax) * vRef;
  float iOffsetVolt = (currentOffset / (float)adcMax) * vRef;

  for (int i = 0; i < samples; i++) {

    // Read ADC
    int rawV = analogRead(VOLTAGE_PIN);
    int rawI = analogRead(CURRENT_PIN);
    // Serial.println(rawV);
    // Serial.println(rawI);

    // Convert to voltage
    float v = (rawV / (float)adcMax) * vRef;
    float i_val = (rawI / (float)adcMax) * vRef;
   

    // Remove offset (center waveform)
    v = v - vOffsetVolt;
    i_val = i_val - iOffsetVolt;

    // Serial.println(v);
    // Serial.println(i_val);

    // Accumulate
    sumV += v * v;
    sumI += i_val * i_val;
    sumP += v * i_val;

    delayMicroseconds(200);  // ~5kHz sampling
  }

  // RMS
  Vrms = sqrt(sumV / samples)  * voltageCalibration;

  Irms = sqrt(sumI / samples)  * currentCalibration;

  float Vrms_raw = sqrt(sumV / samples);
  float Irms_raw = sqrt(sumI / samples);

  // Noise filtering BEFORE scaling
  if (Vrms_raw < 0.03) Vrms_raw = 0;
  if (Irms_raw < 0.008) Irms_raw = 0;

  Vrms = Vrms_raw * voltageCalibration;
  Irms = Irms_raw * currentCalibration;



  // Real power
  realPower = (sumP / samples) * voltageCalibration * currentCalibration;

  // Apparent power
  float apparentPower = Vrms * Irms;

  // Power factor
  if (apparentPower > 0.01)
    pf = realPower / apparentPower;
  else
    pf = 0;

  // Clamp PF
  if (pf > 1) pf = 1;
  if (pf < -1) pf = -1;

  // Low signal cleanup
  if (Vrms < 55) Vrms = 0;
  if (Irms < 0.2) Irms = 0;

  if (Vrms == 0 || Irms == 0) {
    realPower = 0;
    pf = 0;
  }
}

// ========= SETUP =========
void setup() {

  Serial.begin(115200);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  setup_wifi();

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  client.setBufferSize(512);

  analogSetAttenuation(ADC_11db);
  analogSetWidth(12);

  delay(2000);
  voltageOffset = getOffset(VOLTAGE_PIN);
  Serial.println(voltageOffset);
  currentOffset = getOffset(CURRENT_PIN);
  Serial.println(currentOffset);
}

// ========= LOOP =========
void loop() {

  if (!client.connected()) reconnect();
  client.loop();

  //  Relay state publish
  if (relayChanged) {
    client.publish("scada/esp32/relay_state1", relayState ? "ON" : "OFF", true);
    relayChanged = false;
  }

  if (millis() < startupDelay) return;

  float Vrms, Irms, realPower, pf;
  calculateValues(Vrms, Irms, realPower, pf);
  // Serial.println(Vrms);
  // Serial.println(Irms);

  int rawCurrent = analogRead(CURRENT_PIN);
  int deviation = abs(rawCurrent - currentOffset);

  String faultType = "NORMAL";

  if (deviation > SPIKE_THRESHOLD) faultType = "SHORT_CIRCUIT";
  else if (Irms > OVER_CURRENT) faultType = "OVERCURRENT";
  else if (Vrms > OVER_VOLTAGE) faultType = "OVERVOLTAGE";
  else if (Vrms > 30 && Vrms < UNDER_VOLTAGE) faultType = "UNDERVOLTAGE";

  bool faultNow = (faultType != "NORMAL");

  // ===== TRIP =====
  if (faultNow && relayState) {

    digitalWrite(RELAY_PIN, HIGH);
    relayState = false;
    relayChanged = true;

    tripTime = millis();
    faultActive = true;

    client.publish("scada/esp32/status1", "TRIPPED", true);
    client.publish("scada/esp32/fault1", faultType.c_str(), true);
  }

  // ===== RECLOSE =====
 if (!relayState && faultActive) {

  // Only reclose if fault is gone
  if (!faultNow && millis() - tripTime > recloseDelay) {

    if (recloseAttempts < maxAttempts) {

      digitalWrite(RELAY_PIN, LOW);
      relayState = true;
      relayChanged = true;

      recloseAttempts++;

      client.publish("scada/esp32/status1", "RECLOSING", true);

    } else {
      client.publish("scada/esp32/status1", "LOCKOUT", true);
    }
  }
}
  // ===== NORMAL =====
  if (relayState && !faultNow) {
    faultActive = false;
    recloseAttempts = 0;
    client.publish("scada/esp32/status1", "NORMAL", true);
  }

  // Controlled publishing
  if (millis() - lastPublish > publishInterval) {

    lastPublish = millis();

    client.publish("scada/esp32/voltage1", String(Vrms,2).c_str());
    client.publish("scada/esp32/current1", String(Irms,2).c_str());
    client.publish("scada/esp32/power1", String(realPower,2).c_str());
    client.publish("scada/esp32/pf1", String(pf,2).c_str());
  }
}