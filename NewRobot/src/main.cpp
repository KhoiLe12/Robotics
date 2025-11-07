#include <Wire.h>
#include <PCF8574.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "config.h"
#include "motor.h"
#include "encoder.h"
#include "serial_proto.h"
#include "battery.h"
#include "dead_reckoning.h"

// Pins, sensors and speed constants are in config.h

// Global variables
PCF8574 pcf(PCF_ADDR);


// Serial telemetry/command handling
const unsigned long telemetryPeriodMs = 50; // 20 Hz
static char serialBuf[128];
static size_t serialPos = 0;

// FreeRTOS task handles
TaskHandle_t hControlTask = nullptr;
TaskHandle_t hSerialTask = nullptr;
TaskHandle_t hTelemetryTask = nullptr;
TaskHandle_t hBatteryTask = nullptr;
TaskHandle_t hDeadReckoningTask = nullptr;


// Serial mutex for thread-safe printing
SemaphoreHandle_t serialMutex = nullptr;

// Function declarations (local module functions declared below)

// Tasks
void ControlTask(void*);
void SerialTask(void*);
void TelemetryTask(void*);
void DeadReckoningTask(void*);
void BatteryTask(void* parameter);

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
  
  stop_car();
  safePrintln("=== SMOOTH LINE FOLLOWER (RTOS) ===");

  // Spawn RTOS tasks (use core 1 for control, 0 for IO)
  xTaskCreatePinnedToCore(ControlTask, "ControlTask", 4096, nullptr, 2, &hControlTask, 1);
  xTaskCreatePinnedToCore(SerialTask,  "SerialTask",  4096, nullptr, 2, &hSerialTask,  0);
  xTaskCreatePinnedToCore(TelemetryTask,"TelemetryTask",3072, nullptr, 1, &hTelemetryTask,0);
  xTaskCreatePinnedToCore(BatteryTask, "BatteryTask", 2048, nullptr, 1, &hBatteryTask, 0);
  xTaskCreatePinnedToCore(DeadReckoningTask, "DeadReckoningTask", 2048, nullptr, 1, &hDeadReckoningTask, 0);
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
    vTaskDelayUntil(&last, period);
  }
}

void SerialTask(void*) {
  for (;;) {
    handleSerial();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
void DeadReckoningTask(void*) {
    const TickType_t period = pdMS_TO_TICKS(10); // 100 Hz
    for (;;) {
        deadReckoningUpdate();
        vTaskDelay(period);
    }
}

void TelemetryTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(telemetryPeriodMs);
  for (;;) {
    // Send telemetry (no line sensor data)
    sendTelemetry();
    vTaskDelay(period);
  }
}

void BatteryTask(void* parameter) {
  const TickType_t period = pdMS_TO_TICKS(2000); // Check battery every 2 seconds
  TickType_t lastBlink = 0;
  bool ledState = false;
  
  for (;;) {
    TickType_t now = xTaskGetTickCount();
    
    // Read battery voltage
    int adcRawValue = analogRead(VOLTAGE_PIN);
    float adcVoltage = (adcRawValue / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
    float batteryVoltage = adcVoltage * (R1 + R2) / R2;
    
    // Send battery info via serial (with BATT prefix for easy parsing)
    String battMessage = "BATT," + String(adcRawValue) + "," + String(batteryVoltage, 2);
    safePrintln(battMessage);
    
    // Handle low battery warning LED
    if (batteryVoltage < LOW_BATT_THRESHOLD) {
      // Blink LED every 250ms when battery is low
      if (now - lastBlink >= pdMS_TO_TICKS(BLINK_INTERVAL)) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
        lastBlink = now;
      }
    } else {
      // Turn off LED when battery is OK
      digitalWrite(LED_PIN, LOW);
      ledState = false;
    }
    
    vTaskDelay(period);
  }
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