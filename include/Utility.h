#include <Arduino.h>
#include <float.h>

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

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
inline bool is_equal(const float& left, const float& right, const float& epsilon /*= 0.001*/) {
    if (left == right)
        return true;
    const float diff = fabsf(left - right);
    if (left == 0 || right == 0 || diff < FLT_MIN) {
        // a or b is zero or both are extremely close to it
        // relative error is less meaningful here
        return diff < (epsilon * FLT_MIN);
    }
    return diff / std::min<float>(float(fabsf(left) + fabsf(right)), FLT_MAX) < epsilon;
}