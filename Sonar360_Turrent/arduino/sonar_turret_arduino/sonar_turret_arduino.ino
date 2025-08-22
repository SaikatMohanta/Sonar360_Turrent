#include <DHT.h>
#include <ArduinoJson.h>

#define DHTPIN 2
#define DHTTYPE DHT22

// HC-SR04 Sensors
#define TRIGGER_PIN_A 3
#define ECHO_PIN_A 4
#define TRIGGER_PIN_B 5
#define ECHO_PIN_B 6
#define TRIGGER_PIN_C 7
#define ECHO_PIN_C 8

DHT dht(DHTPIN, DHTTYPE);

float temperature = 0.0;
float humidity = 0.0;

void setup() {
  Serial.begin(115200);
  dht.begin();

  pinMode(TRIGGER_PIN_A, OUTPUT);
  pinMode(ECHO_PIN_A, INPUT);
  pinMode(TRIGGER_PIN_B, OUTPUT);
  pinMode(ECHO_PIN_B, INPUT);
  pinMode(TRIGGER_PIN_C, OUTPUT);
  pinMode(ECHO_PIN_C, INPUT);

  delay(2000); // stabilize sensors
}

void loop() {
  readEnv();
  float sos = 331.4 + (0.6 * temperature) + (0.0124 * humidity);

  float distA = readDistance(TRIGGER_PIN_A, ECHO_PIN_A, sos);
  float distB = readDistance(TRIGGER_PIN_B, ECHO_PIN_B, sos);
  float distC = readDistance(TRIGGER_PIN_C, ECHO_PIN_C, sos);

  sendData(distA, distB, distC);
  delay(100);
}

void readEnv() {
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
  if (isnan(humidity) || isnan(temperature)) {
    humidity = 50.0; temperature = 20.0;
  }
}

float readDistance(int trig, int echo, float sos) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH);
  float distance = (duration * (sos / 10000.0)) / 2.0;
  if (distance > 400 || distance < 2) return -1.0;
  return distance;
}

void sendData(float a, float b, float c) {
  StaticJsonDocument<200> doc;
  doc["temp"] = temperature;
  doc["humidity"] = humidity;

  JsonObject sensors = doc.createNestedObject("sensors");

  if (a > 0) sensors["A"] = a;
  if (b > 0) sensors["B"] = b;
  if (c > 0) sensors["C"] = c;

  Serial.println("<START>");
  serializeJson(doc, Serial);
  Serial.println();
  Serial.println("<END>");
}
