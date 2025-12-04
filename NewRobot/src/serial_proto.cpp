#include "serial_proto.h"
#include "encoder.h"
#include "motor.h"
#include "config.h"
#include <Arduino.h>

// External control mode variable
extern volatile ControlMode currentMode;

// External control source tracking
extern volatile ControlSource currentControlSource;
extern volatile unsigned long lastControlTime;

static char serialBuf_[128];
static size_t serialPos_ = 0;

static void processSerialLine_(const char* line) {
  if (strcmp(line, "R") == 0) {
    resetEncoderCounts();
    safePrintln("A,RESET_OK");
  } else if (strcmp(line, "PING") == 0) {
    safePrintln("PONG");
  } else if (strcmp(line, "GET") == 0) {
    // will be sent by TelemetryTask periodically; GET requests an immediate frame with last-known sensors
    // Note: caller should call sendTelemetry with current snapshot; since we don't have it here, just echo ack
    safePrintln("A,GET_OK");
  } else if (strcmp(line, "STOP") == 0) {
    currentControlSource = SOURCE_NONE;
    stop_car();
    desiredSpeedL = 0.0;
    desiredSpeedR = 0.0;
    safePrintln("A,STOP_OK");
  } else if (strcmp(line, "CELEBRATE") == 0) {
    // Celebration spin - temporarily take control
    ControlSource prevSource = currentControlSource;
    ControlMode prevMode = currentMode;
    currentControlSource = SOURCE_SERIAL;
    currentMode = MODE_OPEN_LOOP;
    
    celebrationSpin(3000);  // 3 second spin
    
    // Restore previous state
    currentControlSource = prevSource;
    currentMode = prevMode;
    lastControlTime = millis();  // Reset timeout
    safePrintln("A,CELEBRATE_OK");
  } else if (strcmp(line, "MODE_PID") == 0) {
    currentMode = MODE_PID_VELOCITY;
    resetPID();
    safePrintln("A,MODE_PID");
  } else if (strcmp(line, "MODE_OPENLOOP") == 0) {
    currentMode = MODE_OPEN_LOOP;
    stop_car();
    safePrintln("A,MODE_OPENLOOP");
  } else if (strncmp(line, "PID_VEL,", 8) == 0) {
    // PID_VEL,left_speed_mps,right_speed_mps
    float left_speed, right_speed;
    if (sscanf(line, "PID_VEL,%f,%f", &left_speed, &right_speed) == 2) {
      // ROS2 serial commands have SOURCE_SERIAL priority
      currentControlSource = SOURCE_SERIAL;
      lastControlTime = millis();
      
      desiredSpeedL = constrain(left_speed, -2.0, 2.0);  // Limit to reasonable speeds
      desiredSpeedR = constrain(right_speed, -2.0, 2.0);
      currentMode = MODE_PID_VELOCITY;
      
      // Debug: confirm PID_VEL received and mode set
      String debugMsg = "PID_VEL: L=" + String(desiredSpeedL, 3) + " R=" + String(desiredSpeedR, 3) + " mode=" + String(currentMode);
      safePrintln(debugMsg);
      safePrintln("A,PID_VEL_OK");
    } else {
      safePrintln("A,PID_VEL_ERROR,FORMAT");
    }
  } else if (strncmp(line, "VEL,", 4) == 0) {
    // VEL,left_pwm,right_pwm,duration_ms
    int left_pwm, right_pwm, duration_ms;
    if (sscanf(line, "VEL,%d,%d,%d", &left_pwm, &right_pwm, &duration_ms) == 3) {
      // Clamp PWM values to safe range
      left_pwm = constrain(left_pwm, -255, 255);
      right_pwm = constrain(right_pwm, -255, 255);
      duration_ms = constrain(duration_ms, 0, 10000); // Max 10 seconds
      
      // Set motor directions and speeds
      if (left_pwm >= 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, left_pwm);
      } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, -left_pwm);
      }
      
      if (right_pwm >= 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, right_pwm);
      } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, -right_pwm);
      }
      
      // Stop after duration (this is a simple approach)
      delay(duration_ms);
      stop_car();
      
      safePrintln("A,VEL_OK");
    } else {
      safePrintln("A,VEL_ERROR,FORMAT");
    }
  } else if (strncmp(line, "FORWARD,", 8) == 0) {
    // FORWARD,speed,duration_ms
    int speed, duration_ms;
    if (sscanf(line, "FORWARD,%d,%d", &speed, &duration_ms) == 2) {
      speed = constrain(speed, 0, 255);
      duration_ms = constrain(duration_ms, 0, 10000);
      
      go_Advance(speed);
      delay(duration_ms);
      stop_car();
      
      safePrintln("A,FORWARD_OK");
    } else {
      safePrintln("A,FORWARD_ERROR,FORMAT");
    }
  } else if (strncmp(line, "TURN_LEFT,", 10) == 0) {
    // TURN_LEFT,duration_ms
    int duration_ms;
    if (sscanf(line, "TURN_LEFT,%d", &duration_ms) == 1) {
      duration_ms = constrain(duration_ms, 0, 5000); // Max 5 seconds
      
      go_Left();
      delay(duration_ms);
      stop_car();
      
      safePrintln("A,TURN_LEFT_OK");
    } else {
      safePrintln("A,TURN_LEFT_ERROR,FORMAT");
    }
  } else if (strncmp(line, "TURN_RIGHT,", 11) == 0) {
    // TURN_RIGHT,duration_ms
    int duration_ms;
    if (sscanf(line, "TURN_RIGHT,%d", &duration_ms) == 1) {
      duration_ms = constrain(duration_ms, 0, 5000); // Max 5 seconds
      
      go_Right();
      delay(duration_ms);
      stop_car();
      
      safePrintln("A,TURN_RIGHT_OK");
    } else {
      safePrintln("A,TURN_RIGHT_ERROR,FORMAT");
    }
  } else {
    String unknownMsg = "A,UNKNOWN_CMD," + String(line);
    safePrintln(unknownMsg);
  }
}

void handleSerial() {
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      serialBuf_[serialPos_] = '\0';
      if (serialPos_ > 0) {
        processSerialLine_(serialBuf_);
      }
      serialPos_ = 0;
    } else {
      if (serialPos_ < sizeof(serialBuf_) - 1) {
        serialBuf_[serialPos_++] = (char)c;
      }
    }
  }
}

void sendTelemetry(bool s1, bool s2, bool s3, bool s4, bool s5) {
  long lc = getLeftCumulativeCount();  // Use cumulative counts for odometry
  long rc = getRightCumulativeCount(); // Use cumulative counts for odometry
  unsigned long t = millis();
  
  String telemetryMsg = "T," + String(t) + "," + String(lc) + "," + String(rc) + "," + 
                       String((int)s1) + "," + String((int)s2) + "," + String((int)s3) + "," + 
                       String((int)s4) + "," + String((int)s5);
  safePrintln(telemetryMsg);
}
