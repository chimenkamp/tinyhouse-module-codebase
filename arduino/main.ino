/*
 * Arduino Nano IoT Sensor Hub
 * Auto-detects connected sensors and transmits data via UART
 * Supports: DHT22, DS18B20, BMP280, Analog sensors, Ultrasonic HC-SR04
 */

#include <Wire.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_BMP280.h>

// Pin definitions for potential sensors
#define DHT_PIN 2
#define ONE_WIRE_BUS 3
#define TRIGGER_PIN 4
#define ECHO_PIN 5
#define ANALOG_PINS_START A0
#define ANALOG_PINS_COUNT 6

// Sensor type identifiers
enum SensorType {
  SENSOR_NONE = 0,
  SENSOR_DHT22 = 1,
  SENSOR_DS18B20 = 2,
  SENSOR_BMP280 = 3,
  SENSOR_ULTRASONIC = 4,
  SENSOR_ANALOG = 5,
  SENSOR_DIGITAL = 6
};

// Sensor structure
struct Sensor {
  SensorType type;
  uint8_t pin;
  bool active;
  float lastValue;
  float lastValue2; // For sensors with multiple values
  unsigned long lastRead;
  char id[16];
};

// Global sensor array
#define MAX_SENSORS 16
Sensor sensors[MAX_SENSORS];
int sensorCount = 0;

// Sensor objects
DHT dht(DHT_PIN, DHT22);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);
Adafruit_BMP280 bmp;

// Configuration
struct Config {
  unsigned long readInterval = 2000;  // ms between readings
  unsigned long heartbeatInterval = 10000;  // ms between heartbeats
  bool autoDetect = true;
  bool debugMode = false;
  uint8_t retryAttempts = 3;
} config;

// Timing
unsigned long lastSensorRead = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastAutoDetect = 0;

// Communication protocol
const char START_MARKER = '<';
const char END_MARKER = '>';
const char SEPARATOR = '|';

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port
  }
  
  Wire.begin();
  
  // Send startup message
  sendMessage("BOOT", "Arduino IoT Hub v1.0");
  
  // Initial sensor detection
  detectSensors();
  
  // Send sensor inventory
  sendSensorInventory();
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Check for commands
  checkSerialCommands();
  
  // Auto-detect sensors periodically
  if (config.autoDetect && currentMillis - lastAutoDetect > 30000) {
    detectSensors();
    lastAutoDetect = currentMillis;
  }
  
  // Read sensors
  if (currentMillis - lastSensorRead >= config.readInterval) {
    readAllSensors();
    lastSensorRead = currentMillis;
  }
  
  // Send heartbeat
  if (currentMillis - lastHeartbeat >= config.heartbeatInterval) {
    sendHeartbeat();
    lastHeartbeat = currentMillis;
  }
}

void detectSensors() {
  sendMessage("STATUS", "Detecting sensors...");
  sensorCount = 0;
  
  // Try DHT22
  if (detectDHT22()) {
    addSensor(SENSOR_DHT22, DHT_PIN, "DHT22");
  }
  
  // Try DS18B20
  if (detectDS18B20()) {
    ds18b20.begin();
    int deviceCount = ds18b20.getDeviceCount();
    for (int i = 0; i < deviceCount && sensorCount < MAX_SENSORS; i++) {
      char id[16];
      sprintf(id, "DS18B20_%d", i);
      addSensor(SENSOR_DS18B20, ONE_WIRE_BUS, id);
    }
  }
  
  // Try BMP280
  if (detectBMP280()) {
    addSensor(SENSOR_BMP280, 0, "BMP280");
  }
  
  // Check Ultrasonic
  if (detectUltrasonic()) {
    addSensor(SENSOR_ULTRASONIC, TRIGGER_PIN, "HC-SR04");
  }
  
  // Check analog pins
  for (int i = 0; i < ANALOG_PINS_COUNT && sensorCount < MAX_SENSORS; i++) {
    if (detectAnalogSensor(ANALOG_PINS_START + i)) {
      char id[16];
      sprintf(id, "ANALOG_A%d", i);
      addSensor(SENSOR_ANALOG, ANALOG_PINS_START + i, id);
    }
  }
  
  sendMessage("DETECT", String("Found ") + String(sensorCount) + " sensors");
}

bool detectDHT22() {
  dht.begin();
  delay(1000);
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  return (!isnan(h) && !isnan(t));
}

bool detectDS18B20() {
  ds18b20.begin();
  return (ds18b20.getDeviceCount() > 0);
}

bool detectBMP280() {
  return bmp.begin(0x76) || bmp.begin(0x77);
}

bool detectUltrasonic() {
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Test ultrasonic
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  return (duration > 0 && duration < 30000);
}

bool detectAnalogSensor(uint8_t pin) {
  int readings[5];
  int sum = 0;
  
  for (int i = 0; i < 5; i++) {
    readings[i] = analogRead(pin);
    sum += readings[i];
    delay(10);
  }
  
  float avg = sum / 5.0;
  float variance = 0;
  
  for (int i = 0; i < 5; i++) {
    variance += pow(readings[i] - avg, 2);
  }
  variance /= 5;
  
  // Sensor is likely connected if readings are stable and not at extremes
  return (avg > 50 && avg < 974 && variance < 100);
}

void addSensor(SensorType type, uint8_t pin, const char* id) {
  if (sensorCount >= MAX_SENSORS) return;
  
  sensors[sensorCount].type = type;
  sensors[sensorCount].pin = pin;
  sensors[sensorCount].active = true;
  sensors[sensorCount].lastRead = 0;
  strcpy(sensors[sensorCount].id, id);
  sensorCount++;
}

void readAllSensors() {
  for (int i = 0; i < sensorCount; i++) {
    if (!sensors[i].active) continue;
    
    switch (sensors[i].type) {
      case SENSOR_DHT22:
        readDHT22(&sensors[i]);
        break;
      case SENSOR_DS18B20:
        readDS18B20(&sensors[i]);
        break;
      case SENSOR_BMP280:
        readBMP280(&sensors[i]);
        break;
      case SENSOR_ULTRASONIC:
        readUltrasonic(&sensors[i]);
        break;
      case SENSOR_ANALOG:
        readAnalog(&sensors[i]);
        break;
    }
  }
}

void readDHT22(Sensor* sensor) {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  
  if (!isnan(h) && !isnan(t)) {
    sensor->lastValue = t;
    sensor->lastValue2 = h;
    sensor->lastRead = millis();
    
    String data = String(sensor->id) + "," + 
                  String(t, 2) + "," + 
                  String(h, 2) + "," +
                  "temp_C,humidity_%";
    sendMessage("DATA", data);
  }
}

void readDS18B20(Sensor* sensor) {
  ds18b20.requestTemperatures();
  
  // Extract sensor index from ID
  int index = sensor->id[8] - '0';
  float t = ds18b20.getTempCByIndex(index);
  
  if (t != DEVICE_DISCONNECTED_C) {
    sensor->lastValue = t;
    sensor->lastRead = millis();
    
    String data = String(sensor->id) + "," + 
                  String(t, 2) + "," +
                  "temp_C";
    sendMessage("DATA", data);
  }
}

void readBMP280(Sensor* sensor) {
  float t = bmp.readTemperature();
  float p = bmp.readPressure() / 100.0; // Convert to hPa
  float a = bmp.readAltitude(1013.25); // Standard sea level pressure
  
  sensor->lastValue = p;
  sensor->lastValue2 = t;
  sensor->lastRead = millis();
  
  String data = String(sensor->id) + "," + 
                String(t, 2) + "," + 
                String(p, 2) + "," +
                String(a, 2) + "," +
                "temp_C,pressure_hPa,altitude_m";
  sendMessage("DATA", data);
}

void readUltrasonic(Sensor* sensor) {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 300000);
  float distance = duration * 0.034 / 2; // Convert to cm
  
  if (distance > 0 && distance < 400) {
    sensor->lastValue = distance;
    sensor->lastRead = millis();
    
    String data = String(sensor->id) + "," + 
                  String(distance, 2) + "," +
                  "distance_cm";
    sendMessage("DATA", data);
  }
}

void readAnalog(Sensor* sensor) {
  int raw = analogRead(sensor->pin);
  float voltage = (raw / 1023.0) * 5.0;
  
  sensor->lastValue = voltage;
  sensor->lastRead = millis();
  
  String data = String(sensor->id) + "," + 
                String(raw) + "," +
                String(voltage, 3) + "," +
                "raw,voltage_V";
  sendMessage("DATA", data);
}

void sendMessage(const char* type, String content) {
  Serial.print(START_MARKER);
  Serial.print(type);
  Serial.print(SEPARATOR);
  Serial.print(millis());
  Serial.print(SEPARATOR);
  Serial.print(content);
  Serial.println(END_MARKER);
}

void sendSensorInventory() {
  String inventory = String(sensorCount) + SEPARATOR;
  for (int i = 0; i < sensorCount; i++) {
    inventory += String(sensors[i].id) + ":" + String(sensors[i].type);
    if (i < sensorCount - 1) inventory += ",";
  }
  sendMessage("INVENTORY", inventory);
}

void sendHeartbeat() {
  String status = "OK|" + String(sensorCount) + "|" + String(freeMemory());
  sendMessage("HEARTBEAT", status);
}

void checkSerialCommands() {
  static char cmdBuffer[64];
  static int cmdIndex = 0;
  static bool receiving = false;
  
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == START_MARKER) {
      receiving = true;
      cmdIndex = 0;
    }
    else if (receiving) {
      if (c == END_MARKER) {
        cmdBuffer[cmdIndex] = '\0';
        processCommand(cmdBuffer);
        receiving = false;
      }
      else if (cmdIndex < 63) {
        cmdBuffer[cmdIndex++] = c;
      }
    }
  }
}

void processCommand(char* cmd) {
  char* token = strtok(cmd, "|");
  if (!token) return;
  
  if (strcmp(token, "CONFIG") == 0) {
    token = strtok(NULL, "|");
    if (token) {
      if (strcmp(token, "INTERVAL") == 0) {
        token = strtok(NULL, "|");
        if (token) config.readInterval = atol(token);
      }
      else if (strcmp(token, "DEBUG") == 0) {
        token = strtok(NULL, "|");
        if (token) config.debugMode = atoi(token);
      }
      else if (strcmp(token, "AUTODETECT") == 0) {
        token = strtok(NULL, "|");
        if (token) config.autoDetect = atoi(token);
      }
    }
    sendMessage("CONFIG", "Updated");
  }
  else if (strcmp(token, "DETECT") == 0) {
    detectSensors();
    sendSensorInventory();
  }
  else if (strcmp(token, "RESET") == 0) {
    sendMessage("STATUS", "Resetting...");
    delay(100);
    asm volatile ("jmp 0");
  }
  else if (strcmp(token, "STATUS") == 0) {
    sendSensorInventory();
    sendHeartbeat();
  }
}

int freeMemory() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}