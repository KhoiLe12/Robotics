#include "battery.h"

// Battery monitoring constants
const int LED_PIN = 2; 
const int VOLTAGE_PIN = 34;
const float R1 = 16000.0; 
const float R2 = 10000.0; 
const float ADC_MAX_VALUE = 4095.0; 
const float ADC_REF_VOLTAGE = 3.3; 
const float LOW_BATT_THRESHOLD = 7.5; 
const int BLINK_INTERVAL = 250;

void batteryInit() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

float readBatteryVoltage() {
  int adcRawValue = analogRead(VOLTAGE_PIN);
  float adcVoltage = (adcRawValue / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
  float batteryVoltage = adcVoltage * (R1 + R2) / R2;
  return batteryVoltage;
}

