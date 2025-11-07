#pragma once
#include <Arduino.h>

// Process any pending serial input (line-buffered) and react to commands.
void handleSerial();

// Compose and send one telemetry frame with the provided sensor snapshot.
// Telemetry format: T,<ms>,<Lcount>,<Rcount>,<x>,<y>,<theta>
void sendTelemetry();

// Thread-safe serial functions (declared in main.cpp)
extern void safePrint(const String& message);
extern void safePrintln(const String& message);
