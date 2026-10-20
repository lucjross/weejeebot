#pragma once
#include <Arduino.h>

// Line-based serial link to the Nano.
namespace Link {

void begin();

// Non-blocking. Returns true when a complete line has arrived; it is copied (without the newline)
// into buf, which must be at least LINE_MAX bytes. Over-long lines are dropped.
bool readLine(char *buf);

void sendLine(const char *s);

}  // namespace Link
