#include <Wire.h>
#include <PCF8574.h>
#include <WiFi.h>
#include <WebServer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "config.h"
#include "motor.h"
#include "line_tracking.h"
#include "encoder.h"
#include "serial_proto.h"
#include "battery.h"


// Pins, sensors and speed constants are in config.h

// Global variables
PCF8574 pcf(PCF_ADDR);
WebServer server(80);

// Control mode
volatile ControlMode currentMode = MODE_OPEN_LOOP;

// Control source tracking
volatile ControlSource currentControlSource = SOURCE_NONE;
volatile unsigned long lastControlTime = 0;

// Line-follow smoothing state is handled inside line_tracking.cpp

// Latest sensor snapshot for telemetry
volatile bool lastS1 = false, lastS2 = false, lastS3 = false, lastS4 = false, lastS5 = false;

// Current wheel speeds (for web display)
volatile float currentLeftSpeedMps = 0.0;
volatile float currentRightSpeedMps = 0.0;

// Serial telemetry/command handling
const unsigned long telemetryPeriodMs = 50; // 20 Hz
static char serialBuf[128];
static size_t serialPos = 0;

// FreeRTOS task handles
TaskHandle_t hControlTask = nullptr;
TaskHandle_t hSerialTask = nullptr;
TaskHandle_t hTelemetryTask = nullptr;
TaskHandle_t hBatteryTask = nullptr;
TaskHandle_t hPIDTask = nullptr;
TaskHandle_t hWebServerTask = nullptr;

// Serial mutex for thread-safe printing
SemaphoreHandle_t serialMutex = nullptr;

// Function declarations (local module functions declared below)

// Tasks
void ControlTask(void*);
void SerialTask(void*);
void TelemetryTask(void*);
void PIDControlTask(void*);
void WebServerTask(void*);

// WiFi setup
void setupWiFi();
void setupWebServer();

// Thread-safe serial functions
void safePrint(const String& message);
void safePrintln(const String& message);

void setup() {
  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Initialize PCF8574
  if (!pcf.begin()) {
    Serial.println("PCF8574 not found!");
    while(1);
  }
  
  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  Serial.begin(115200);
  
  // Create serial mutex before any tasks start
  serialMutex = xSemaphoreCreateMutex();
  if (serialMutex == NULL) {
    Serial.println("FATAL: Failed to create serial mutex!");
    while(1);
  }
  
  // Initialize encoders
  encoderInit();
  
  // Initialize battery monitoring
  batteryInit();
  
  // Ensure motors are stopped and speeds are zero on boot
  stop_car();
  desiredSpeedL = 0.0;
  desiredSpeedR = 0.0;
  currentControlSource = SOURCE_NONE;
  currentMode = MODE_OPEN_LOOP;  // Start in safe open-loop mode
  
  safePrintln("=== ROBOT CONTROL SYSTEM (RTOS + WiFi PID) ===");
  
  // Setup WiFi
  setupWiFi();
  setupWebServer();

  // Spawn RTOS tasks (use core 1 for control, 0 for IO)
  xTaskCreatePinnedToCore(ControlTask, "ControlTask", 4096, nullptr, 2, &hControlTask, 1);
  xTaskCreatePinnedToCore(SerialTask,  "SerialTask",  4096, nullptr, 2, &hSerialTask,  0);
  xTaskCreatePinnedToCore(TelemetryTask,"TelemetryTask",3072, nullptr, 1, &hTelemetryTask,0);
  xTaskCreatePinnedToCore(BatteryTask, "BatteryTask", 2048, nullptr, 1, &hBatteryTask, 0);
  xTaskCreatePinnedToCore(PIDControlTask, "PIDControlTask", 4096, nullptr, 2, &hPIDTask, 1);
  xTaskCreatePinnedToCore(WebServerTask, "WebServerTask", 4096, nullptr, 1, &hWebServerTask, 0);
}

void loop() {
  vTaskDelay(1);
}

// (all logic except tasks moved to modules)

// ==========================
// FreeRTOS Tasks
// ==========================

void ControlTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(10); // 100 Hz control
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    // Read all line sensors
    bool s1 = 0;
    bool s2 = 0;
    bool s3 = 0;
    bool s4 = 0;
    bool s5 = 0;

    // Update snapshot (for telemetry)
    lastS1 = s1; lastS2 = s2; lastS3 = s3; lastS4 = s4; lastS5 = s5;

    // Run line follower step (non-blocking)
    // followLineSmooth(s1, s2, s3, s4, s5);  // DISABLED for rectangle navigation

    vTaskDelayUntil(&last, period);
  }
}

void SerialTask(void*) {
  for (;;) {
    handleSerial();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void TelemetryTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(telemetryPeriodMs);
  for (;;) {
    // Send telemetry using last snapshot collected in ControlTask
    sendTelemetry(lastS1, lastS2, lastS3, lastS4, lastS5);
    vTaskDelay(period);
  }
}

void PIDControlTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(PID_SAMPLE_INTERVAL_MS);
  TickType_t last = xTaskGetTickCount();

  float leftSpeedMps = 0.0;
  float rightSpeedMps = 0.0;
  
  for (;;) {
    // Check for control timeout - release control if no commands received
    if (currentControlSource != SOURCE_NONE && 
        (millis() - lastControlTime) > CONTROL_TIMEOUT_MS) {
      currentControlSource = SOURCE_NONE;
      desiredSpeedL = 0.0;
      desiredSpeedR = 0.0;
      currentMode = MODE_OPEN_LOOP;  // Switch to safe mode
      stop_car();  // Explicitly stop motors
      safePrintln("CONTROL TIMEOUT - Stopping");
    }
    
    // Only run PID if in PID mode
    if (currentMode == MODE_PID_VELOCITY) {
      // Deadband: if both desired speeds are near zero, just stop
      if (abs(desiredSpeedL) < 0.01 && abs(desiredSpeedR) < 0.01) {
        stop_car();
        resetPID();
        leftSpeedMps = 0.0;
        rightSpeedMps = 0.0;
      } else {
        // Measure current velocities
        long leftCount = getLeftPulseCountAndReset();
        long rightCount = getRightPulseCountAndReset();
        
        float leftRevs = (float)leftCount / PULSES_PER_REV;
        float rightRevs = (float)rightCount / PULSES_PER_REV;
        float wheelCirc = PI * WHEEL_DIAMETER_M;
        
        leftSpeedMps = (leftRevs * wheelCirc) / (PID_SAMPLE_INTERVAL_MS / 1000.0);
        rightSpeedMps = -(rightRevs * wheelCirc) / (PID_SAMPLE_INTERVAL_MS / 1000.0);  // Negate for reversed encoder
        
        // Compute PID outputs with per-wheel deadband
        float pwmLeft = (abs(desiredSpeedL) < 0.01) ? 0 : computePID_Left(desiredSpeedL, leftSpeedMps);
        float pwmRight = (abs(desiredSpeedR) < 0.01) ? 0 : computePID_Right(desiredSpeedR, rightSpeedMps);
        
        // Apply to motors
        setMotorLeft(pwmLeft);
        setMotorRight(pwmRight);
      }

    } else {
      leftSpeedMps = 0.0;
      rightSpeedMps = 0.0;
    }

    // Update global variables for web display
    currentLeftSpeedMps = leftSpeedMps;
    currentRightSpeedMps = rightSpeedMps;

    // Only print speeds occasionally to reduce serial spam
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
      safePrintln("Spd L:" + String(leftSpeedMps, 2) + " R:" + String(rightSpeedMps, 2) + " Src:" + String(currentControlSource));
      lastPrint = millis();
    }
    
    vTaskDelayUntil(&last, period);
  }
}

void WebServerTask(void*) {
  for (;;) {
    server.handleClient();
    
    // Keep WiFi control alive if:
    // 1. We're commanding non-zero speeds, OR
    // 2. We're in PID mode (to prevent immediate timeout after mode switch)
    if (currentControlSource == SOURCE_WIFI && 
        (abs(desiredSpeedL) > 0.01 || abs(desiredSpeedR) > 0.01 || 
         currentMode == MODE_PID_VELOCITY)) {
      lastControlTime = millis();
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz web server handling
  }
}

// ==========================
// WiFi Setup Functions
// ==========================

void setupWiFi() {
  safePrintln("Starting WiFi Access Point...");
  
  // Create AP with name "RobotAP" and password "robot123"
  WiFi.mode(WIFI_AP);
  WiFi.softAP("RobotAP", "robot123");
  
  IPAddress IP = WiFi.softAPIP();
  safePrint("AP IP address: ");
  safePrintln(IP.toString());
  safePrintln("Connect to WiFi: RobotAP");
  safePrintln("Password: robot123");
  safePrintln("Then open: http://192.168.4.1");
}

void setupWebServer() {
  // Main page with PID tuning interface
  server.on("/", HTTP_GET, [](){
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:Arial;margin:20px;} input{margin:5px;} .mode{background:#4CAF50;color:white;padding:10px;border-radius:5px;} .speed{background:#2196F3;color:white;padding:10px;border-radius:5px;margin-top:10px;}</style>";
    html += "</head><body>";
    html += "<h1>Robot PID Control</h1>";
    
    // Mode display
    html += "<div class='mode'>Current Mode: ";
    html += (currentMode == MODE_PID_VELOCITY) ? "PID VELOCITY" : "OPEN LOOP";
    html += "</div><br>";

    // Speed display with debug info
    html += "<div class='speed'>";
    html += "<b>MEASURED SPEEDS (from encoders)</b><br>";
    html += "Left: " + String(currentLeftSpeedMps, 3) + " m/s<br>";
    html += "Right: " + String(currentRightSpeedMps, 3) + " m/s<br><br>";
    html += "<b>DESIRED SPEEDS (PID targets)</b><br>";
    html += "Left: " + String(desiredSpeedL, 3) + " m/s<br>";
    html += "Right: " + String(desiredSpeedR, 3) + " m/s<br><br>";
    html += "<b>PID GAINS</b><br>";
    html += "Left Kp=" + String(KpL, 1) + " Ki=" + String(KiL, 3) + " Kd=" + String(KdL, 3) + "<br>";
    html += "Right Kp=" + String(KpR, 1) + " Ki=" + String(KiR, 3) + " Kd=" + String(KdR, 3);
    html += "</div><br>";
    
    // PID tuning form
    html += "<form action='/update'>";
    html += "<h3>Left Motor PID</h3>";
    html += "Kp: <input type='number' step='0.01' name='KpL' value='" + String(KpL) + "'><br>";
    html += "Ki: <input type='number' step='0.001' name='KiL' value='" + String(KiL) + "'><br>";
    html += "Kd: <input type='number' step='0.001' name='KdL' value='" + String(KdL) + "'><br>";
    html += "Desired Speed (m/s): <input type='number' step='0.01' name='dsL' value='" + String(desiredSpeedL) + "'><br>";
    
    html += "<h3>Right Motor PID</h3>";
    html += "Kp: <input type='number' step='0.01' name='KpR' value='" + String(KpR) + "'><br>";
    html += "Ki: <input type='number' step='0.001' name='KiR' value='" + String(KiR) + "'><br>";
    html += "Kd: <input type='number' step='0.001' name='KdR' value='" + String(KdR) + "'><br>";
    html += "Desired Speed (m/s): <input type='number' step='0.01' name='dsR' value='" + String(desiredSpeedR) + "'><br>";
    
    html += "<br><input type='submit' value='Update PID Parameters'></form>";
    
    // Mode control
    html += "<br><h3>Control Mode</h3>";
    html += "<button onclick=\"location.href='/mode/pid'\">Switch to PID Mode</button> ";
    html += "<button onclick=\"location.href='/mode/openloop'\">Switch to Open Loop</button><br>";
    html += "<button onclick=\"location.href='/stop'\">STOP MOTORS</button>";
    
    // Speed control (only works in PID mode)
    html += "<br><h3>Motor Speed Control</h3>";
    html += "<form action='/setspeed'>";
    html += "Left Speed (m/s): <input type='number' step='0.01' name='left' value='0'><br>";
    html += "Right Speed (m/s): <input type='number' step='0.01' name='right' value='0'><br>";
    html += "<input type='submit' value='Set Speeds'></form>";
    
    html += "</body></html>";
    server.send(200, "text/html", html);
  });

  // Update PID parameters
  server.on("/update", HTTP_GET, [](){
    // Check if ROS2/serial has control and hasn't timed out
    if (currentControlSource >= SOURCE_SERIAL && 
        (millis() - lastControlTime) <= CONTROL_TIMEOUT_MS) {
      server.send(403, "text/plain", "Control locked by ROS2 - wait for timeout");
      return;
    }
    
    // WiFi can take control now
    currentControlSource = SOURCE_WIFI;
    lastControlTime = millis();
    
    if (server.hasArg("KpL")) KpL = server.arg("KpL").toFloat();
    if (server.hasArg("KiL")) KiL = server.arg("KiL").toFloat();
    if (server.hasArg("KdL")) KdL = server.arg("KdL").toFloat();
    if (server.hasArg("dsL")) desiredSpeedL = server.arg("dsL").toFloat();
    if (server.hasArg("KpR")) KpR = server.arg("KpR").toFloat();
    if (server.hasArg("KiR")) KiR = server.arg("KiR").toFloat();
    if (server.hasArg("KdR")) KdR = server.arg("KdR").toFloat();
    if (server.hasArg("dsR")) desiredSpeedR = server.arg("dsR").toFloat();
    
    server.sendHeader("Location", "/");
    server.send(303);
  });
  
  // Mode switching
  server.on("/mode/pid", HTTP_GET, [](){
    if (currentControlSource >= SOURCE_SERIAL && 
        (millis() - lastControlTime) <= CONTROL_TIMEOUT_MS) {
      server.send(403, "text/plain", "Control locked by ROS2");
      return;
    }
    currentControlSource = SOURCE_WIFI;
    lastControlTime = millis();
    currentMode = MODE_PID_VELOCITY;
    desiredSpeedL = 0.0;  // Set to 0 to maintain control
    desiredSpeedR = 0.0;
    resetPID();
    safePrintln("Switched to PID mode");
    server.sendHeader("Location", "/");
    server.send(303);
  });
  
  server.on("/mode/openloop", HTTP_GET, [](){
    if (currentControlSource >= SOURCE_SERIAL && 
        (millis() - lastControlTime) <= CONTROL_TIMEOUT_MS) {
      server.send(403, "text/plain", "Control locked by ROS2");
      return;
    }
    currentControlSource = SOURCE_WIFI;
    lastControlTime = millis();
    currentMode = MODE_OPEN_LOOP;
    stop_car();
    safePrintln("Switched to Open Loop mode");
    server.sendHeader("Location", "/");
    server.send(303);
  });
  
  // Set motor speeds (only works in PID mode)
  server.on("/setspeed", HTTP_GET, [](){
    safePrint("SetSpeed called, mode=");
    safePrintln(String(currentMode));
    
    if (currentControlSource >= SOURCE_SERIAL && 
        (millis() - lastControlTime) <= CONTROL_TIMEOUT_MS) {
      server.send(403, "text/plain", "Control locked by ROS2");
      return;
    }
    if (currentMode != MODE_PID_VELOCITY) {
      String msg = "Must be in PID mode. Current mode: " + String(currentMode);
      server.send(400, "text/plain", msg);
      return;
    }
    currentControlSource = SOURCE_WIFI;
    lastControlTime = millis();
    
    if (server.hasArg("left")) desiredSpeedL = server.arg("left").toFloat();
    if (server.hasArg("right")) desiredSpeedR = server.arg("right").toFloat();
    
    safePrint("Web set speeds: L=");
    safePrint(String(desiredSpeedL, 3));
    safePrint(" R=");
    safePrintln(String(desiredSpeedR, 3));
    
    server.sendHeader("Location", "/");
    server.send(303);
  });
  
  // Emergency stop - always works regardless of control source
  server.on("/stop", HTTP_GET, [](){
    currentControlSource = SOURCE_NONE;
    currentMode = MODE_OPEN_LOOP;  // Exit PID mode
    stop_car();
    desiredSpeedL = 0.0;
    desiredSpeedR = 0.0;
    safePrintln("EMERGENCY STOP");
    server.sendHeader("Location", "/");
    server.send(303);
  });
  
  server.begin();
  safePrintln("Web server started");
}

// ==========================
// Thread-Safe Serial Functions
// ==========================

void safePrint(const String& message) {
  if (serialMutex != nullptr && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    Serial.print(message);
    xSemaphoreGive(serialMutex);
  }
}

void safePrintln(const String& message) {
  if (serialMutex != nullptr && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    Serial.println(message);
    xSemaphoreGive(serialMutex);
  }
}