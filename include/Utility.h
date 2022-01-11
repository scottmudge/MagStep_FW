#include <Arduino.h>

#ifdef DEBUG_OUTPUT
#define Dbg(expr, ...) Serial.printf((const char*)(F(expr)), ## __VA_ARGS__)
#define DbgLn(expr, ...) {Serial.printf((const char*)(F(expr)), ## __VA_ARGS__); Serial.println();}

#define Err(expr, ...) Serial.printf((const char*)(F("ERR: " expr)), ## __VA_ARGS__)
#define ErrLn(expr, ...) {Serial.printf((const char*)(F("ERR: " expr)), ## __VA_ARGS__); Serial.println();}

#define Warn(expr, ...) Serial.printf((const char*)(F("WARN: " expr)), ## __VA_ARGS__)
#define WarnLn(expr, ...) {Serial.printf((const char*)(F("WARN: " expr)), ## __VA_ARGS__); Serial.println();}
#else
#define Dbg(expr) void()
#define DbgLn(expr, ...) void()

#define Err(expr, ...) void()
#define ErrLn(expr, ...) void()

#define Warn(expr, ...) void()
#define WarnLn(expr, ...) void()
#endif
