#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include "ACS712.h"

// =================== WIFI CONFIG ===================
const char* ssid = "Rifat";
const char* password = "rifat123";

WebServer server(80);

// =================== PIN CONFIGURATION ===================
#define IN1 4
#define IN2 5
#define ENA 19

#define IR_SENSOR 14
#define CURRENT_SENSOR 27

#define SERVO_PIN 25
#define SERVO_OPEN_ANGLE 90
#define SERVO_CLOSED_ANGLE 0

#define TRIG1 26
#define ECHO1 16
#define TRIG2 32
#define ECHO2 33

#define LED_GREEN 2
#define LED_RED 15

#define BUZZER1 22
#define BUZZER2 23

#define SWITCH_PIN 21

// =================== GLOBAL VARIABLES ===================
Servo doorServo;
bool doorStatus = false; // false = closed, true = open
int currentFloor = 1;
float currentLimit = 30;
float lastCurrent = 0;
float lastDist1 = 0;
float lastDist2 = 0;
int lastIR = 0;

// =================== FUNCTION DECLARATIONS ===================
void handleStatusAPI();
void handleControlAPI();
void handleOptions();
float readCurrent();
float getDistance1();
float getDistance2();
void LEDstateRG(bool red, bool green);
void beepBuzzer(int buzzerPin, int duration);
void blinkLED(int pin, int times, int delayMs);
void blinkAllLED(int times, int delayMs);
void safeDoorClose(int buzzerPin, int ledPin);
void doorOpen();
void motorSTOP();
void pullUp();
void pullDown();
void stayOnTop();

// =================== SETUP ===================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IR_SENSOR, INPUT);
  pinMode(CURRENT_SENSOR, INPUT);

  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER1, OUTPUT);
  pinMode(BUZZER2, OUTPUT);
  pinMode(SWITCH_PIN, INPUT);

  digitalWrite(TRIG1, LOW);
  digitalWrite(TRIG2, LOW);

  // ===== Servo Setup =====
  doorServo.attach(SERVO_PIN, 500, 2500); // correct pulse width range for SG90
  doorServo.write(SERVO_CLOSED_ANGLE);
  doorStatus = false;
  Serial.println("✅ Servo attached to pin 25 (SG90)");

  LEDstateRG(true, false);
  Serial.println("🚀 System Booted. Initializing WiFi...");

  // --- WiFi Setup ---
  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }
  // Serial.println("\n✅ WiFi Connected!");
  // Serial.print("📡 IP Address: ");
  // Serial.println(WiFi.localIP());

  // // --- Web Server Setup ---
  // server.on("/status", HTTP_GET, handleStatusAPI);
  // server.on("/status", HTTP_OPTIONS, handleOptions);
  // server.on("/control", HTTP_GET, handleControlAPI);
  // server.on("/control", HTTP_OPTIONS, handleOptions);
  // server.begin();
  // Serial.println("🌐 Web API Ready!");
}

// =================== API HEADERS ===================
void handleOptions() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "*");
  server.send(204);
}

// =================== STATUS API ===================
void handleStatusAPI() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS"); 

  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");

  // Use cached values for faster response
  StaticJsonDocument<512> json;
  json["currentFloor"] = currentFloor;
  json["irSensor"] = lastIR;
  json["currentA"] = lastCurrent;
  json["distance1_cm"] = lastDist1;
  json["distance2_cm"] = lastDist2;
  json["doorStatus"] = doorStatus ? "open" : "closed";

  String response;
  serializeJson(json, response);
  server.send(200, "application/json", response);

  Serial.println("📤 Sent /status response");
}

// =================== CONTROL API ===================
void handleControlAPI() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");

  if (server.hasArg("floor")) {
    int targetFloor = server.arg("floor").toInt();
    Serial.printf("🌐 API Request: Move to floor %d\n", targetFloor);

    if (targetFloor == 1 && currentFloor != 1) {
      pullDown();
      server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Moved to floor 1\"}");
    } else if (targetFloor == 2 && currentFloor != 2) {
      pullUp();
      server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Moved to floor 2\"}");
    } else {
      server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Already on floor\"}");
    }
  } else {
    server.send(400, "application/json", "{\"error\":\"missing floor param\"}");
  }
}

// =================== CURRENT SENSOR ===================
float readCurrent() {
  const int samples = 200;
  const float ADC_REF = 3.3;        // ESP32 ADC reference voltage
  const float sensitivity = 0.066;  // 66 mV/A for ACS712-30A
  const float zeroOffset = 1.65;    // mid-point voltage (≈ Vcc/2)

  float totalVoltage = 0;

  for (int i = 0; i < samples; i++) {
    int raw = analogRead(CURRENT_SENSOR);
    float voltage = (raw / 4095.0) * ADC_REF;
    totalVoltage += voltage;
    delayMicroseconds(50);
  }

  float avgVoltage = totalVoltage / samples;
  float currentA = (avgVoltage - zeroOffset) / sensitivity;

  // remove sign, filter noise, clamp realistic range
  currentA = fabs(currentA);
  if (currentA < 0.05) currentA = 0;   // ignore small ADC noise
  if (currentA > 30.0) currentA = 30.0;

  Serial.print("Current: ");
  Serial.print(currentA, 3);
  Serial.println(" A");

  lastCurrent = currentA;
  return currentA;
}

// =================== ULTRASONIC ===================
float getDistance1() {
  digitalWrite(TRIG1, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG1, LOW);
  long duration = pulseIn(ECHO1, HIGH, 50000UL);
  float distance = (duration * 0.0343) / 2.0;

  Serial.print("📏 Distance1: ");
  Serial.print(distance, 2);
  Serial.println(" cm");

  lastDist1 = distance;
  return distance;
}

float getDistance2() {
  digitalWrite(TRIG2, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG2, LOW);
  long duration = pulseIn(ECHO2, HIGH, 50000UL);
  float distance = (duration * 0.0343) / 2.0;

  Serial.print("📐 Distance2: ");
  Serial.print(distance, 2);
  Serial.println(" cm");

  lastDist2 = distance;
  return distance;
}

// =================== LED & BUZZER ===================
void LEDstateRG(bool red, bool green) {
  Serial.printf("💡 LED State -> RED: %d, GREEN: %d\n", red, green);
  digitalWrite(LED_RED, red);
  digitalWrite(LED_GREEN, green);
}

void beepBuzzer(int buzzerPin, int duration) {
  Serial.printf("🔊 Beep buzzer %d for %d ms\n", buzzerPin, duration);
  digitalWrite(buzzerPin, HIGH);
  delay(duration);
  digitalWrite(buzzerPin, LOW);
}

void blinkLED(int pin, int times, int delayMs) {
  Serial.printf("💡 Blinking LED pin %d %d times\n", pin, times);
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(delayMs);
    digitalWrite(pin, LOW);
    delay(delayMs);
  }
}

void blinkAllLED(int times, int delayMs) {
  Serial.printf("⚠ Blinking all LEDs %d times\n", times);
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    delay(delayMs);
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    delay(delayMs);
  }
}

// =================== DOOR CONTROL ===================
void doorOpen() {
  Serial.println("🚪 Opening Door...");
  doorServo.write(SERVO_OPEN_ANGLE);
  delay(1500); // give SG90 enough time
  doorStatus = true;
  Serial.println("✅ Door Opened");
}

void safeDoorClose(int buzzerPin, int ledPin) {
  Serial.println("🚧 Checking IR before door close...");
  int irValue = digitalRead(IR_SENSOR);

  while (irValue == LOW) {
    Serial.println("⚠ IR detected! Obstacle present!");
    digitalWrite(buzzerPin, HIGH);
    blinkAllLED(5, 150);
    irValue = digitalRead(IR_SENSOR);
    server.handleClient(); // Handle requests even during door closing
  }

  digitalWrite(buzzerPin, LOW);
  Serial.println("✅ No obstacle. Closing door...");
  doorServo.write(SERVO_CLOSED_ANGLE);
  delay(1500);
  doorStatus = false;
  Serial.println("🚪 Door Closed");
}

// =================== MOTOR CONTROL ===================
void motorSTOP() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  Serial.println("🛑 Motor Stopped");
}

void pullUp() {
  Serial.println("⬆ LIFT MOVING UP");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 240);

  for (int t = 0; t < 20; t++) {
    float currentNow = readCurrent();
    if (currentNow > currentLimit) {
      Serial.println("⚠ Overweight! Stopping motor");
      blinkAllLED(5, 150);
      beepBuzzer(BUZZER1, 1000);
      motorSTOP();
      return;
    }
    delay(100);
    server.handleClient(); // Handle requests during motor operation
  }

  motorSTOP();
  currentFloor = 2;
  Serial.println("✅ Reached Floor 2");
}

void stayOnTop() {
  Serial.println("🏗 Staying on Top Floor");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 100);
}

void pullDown() {
  Serial.println("⬇ LIFT MOVING DOWN");
  for (int i = 100; i >= 0; i--) {
    analogWrite(ENA, i);
    delay(10);
    if (i % 10 == 0) server.handleClient(); // Handle requests periodically
  }
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  for (int i = 70; i >= 20; i--) {
    analogWrite(ENA, i);
    delay(10);
    if (i % 10 == 0) server.handleClient(); // Handle requests periodically
  }
  motorSTOP();
  currentFloor = 1;
  Serial.println("✅ Reached Floor 1");
}

void autodetect(){
  server.handleClient(); // Handle API requests FIRST

  // Update sensor readings
  static unsigned long lastSensorUpdate = 0;
  if (millis() - lastSensorUpdate > 1000) {
    lastDist1 = getDistance1();
    delay(200);
    lastDist2 = getDistance2();
    delay(200);
    lastCurrent = readCurrent();
    lastIR = digitalRead(IR_SENSOR);
    lastSensorUpdate = millis();
  }

  // Check distance sensors with reduced delay
  delay(100);
  float d1 = lastDist1;
  delay(100);
  float d2 = lastDist2;

  if (d1 < 10) {
    Serial.println("📍 Sensor 1 Triggered — Moving Lift DOWN-UP Cycle");
    if (currentFloor == 2) pullDown();
    motorSTOP();
    delay(1000);
    doorOpen();
    
    // Wait with server handling
    for (int i = 0; i < 70; i++) {
      delay(100);
      server.handleClient();
    }
    
    beepBuzzer(BUZZER1, 1000);
    safeDoorClose(BUZZER1, LED_RED);
    pullUp();
    LEDstateRG(false, true);
    stayOnTop();
    delay(2000);
    doorOpen();
    
    // Wait with server handling
    for (int i = 0; i < 40; i++) {
      delay(100);
      server.handleClient();
    }
    
    beepBuzzer(BUZZER2, 1000);
    safeDoorClose(BUZZER2, LED_GREEN);
    currentFloor = 2;
    Serial.println("🏁 Lift reached 2nd Floor");
  }

  if (d2 < 10) {
    Serial.println("📍 Sensor 2 Triggered — Moving Lift UP-DOWN Cycle");
    if (currentFloor == 1) pullUp();
    stayOnTop();
    delay(1000);
    doorOpen();
    
    // Wait with server handling
    for (int i = 0; i < 70; i++) {
      delay(100);
      server.handleClient();
    }
    
    beepBuzzer(BUZZER2, 1000);
    safeDoorClose(BUZZER2, LED_GREEN);
    pullDown();
    motorSTOP();
    LEDstateRG(true, false);
    delay(2000);
    doorOpen();
    
    // Wait with server handling
    for (int i = 0; i < 40; i++) {
      delay(100);
      server.handleClient();
    }
    
    beepBuzzer(BUZZER1, 1000);
    safeDoorClose(BUZZER1, LED_RED);
    currentFloor = 1;
    Serial.println("🏁 Lift reached 1st Floor");
  }

  server.handleClient(); // Handle API requests again
}

void sensortest(){
  getDistance1();
  delay(400);
  getDistance2();
  delay(400);
  blinkAllLED(5, 150);
  delay(100);
  readCurrent();
  delay(100);
  doorOpen();
  delay(2000);
  safeDoorClose(BUZZER2, LED_GREEN);
  delay(1000);
}

void testrun(){
  blinkAllLED(5, 150);
  readCurrent();
  delay(100);
  getDistance1();
  delay(100);
  getDistance2();
  currentFloor = 1;
  LEDstateRG(true, false);

  motorSTOP();
  delay(1000);
  doorOpen();
    
  for (int i = 0; i < 70; i++) {
    delay(100);
    server.handleClient();
  }
    
  beepBuzzer(BUZZER2, 1000);
  beepBuzzer(BUZZER1, 1000);
  safeDoorClose(BUZZER2, LED_RED);
  pullUp();
  currentFloor = 2;
  LEDstateRG(false, true);
  Serial.println("🏁 Lift reached 2nd Floor");
  stayOnTop();
  delay(2000);
  doorOpen();
    
  // Wait with server handling
  for (int i = 0; i < 70; i++) {
    delay(100);
    server.handleClient();
  }

  beepBuzzer(BUZZER2, 1000);
  beepBuzzer(BUZZER1, 1000);
  safeDoorClose(BUZZER2, LED_GREEN);
  pullDown();
  currentFloor = 1;
  LEDstateRG(true, false);
  Serial.println("🏁 Lift reached 1st Floor");
  delay(2000);
}

// =================== MAIN LOOP ===================
void loop() {
  // autodetect();
  // sensortest();
  testrun();
}