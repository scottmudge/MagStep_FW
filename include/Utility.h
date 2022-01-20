#include <Arduino.h>
#include <float.h>
#include <ESPArduinoPins.h>

#include "Config.h"

//==============================================================================
// Pin definitions
#define STEP_IN_PIN ESP32_D26
#define STEP_OUT_PIN ESP32_D14
#define DIR_IN_PIN ESP32_D27
#define DIR_OUT_PIN ESP32_D12

// Very fast GPIO Reg Functions
#define GPIO_Set(x) (REG_WRITE(GPIO_OUT_W1TS_REG, 1 << (uint32_t)x))
#define GPIO_Clear(x) (REG_WRITE(GPIO_OUT_W1TC_REG, 1 << (uint32_t)x))
#define GPIO_IN_Read(x) (REG_READ(GPIO_IN_REG) & (1 << x))
#define GPIO_IN_Read_Byte(x) ((REG_READ(GPIO_IN_REG) & (1 << x)) ? 1U : 0U)
#define GPIO_IN_ReadAll() (REG_READ(GPIO_IN_REG))

// ~5 nanoseconds per clock
#define CLOCK_LEN_NS 5

// Clock cycles
#define STEP_PULSE_LEN 500
#define STEP_PULSE_LEN_MAX 5000

// Utility Functions
//==============================================================================
inline uint32_t IRAM_ATTR clocks() {
    uint32_t ccount;
    asm volatile("rsr %0, ccount"
                 : "=a"(ccount));
    return ccount;
}

uint64_t IRAM_ATTR inline _millis() { return esp_timer_get_time() / 1000l; }

uint64_t IRAM_ATTR inline _micros() { return esp_timer_get_time(); }

void IRAM_ATTR inline delayMicroseconds(uint32_t us) {
    if (us) {
        uint32_t m = _micros();
        while ((_micros() - m) < us) {
            asm(" nop");
        }
    }
}

void IRAM_ATTR inline delayClocks(uint32_t clks) {
    uint32_t c = clocks();
    while ((clocks() - c) < clks) {
        asm(" nop");
    }
}


//==============================================================================
inline void IRAM_ATTR stepDir(const uint8_t dir, const uint32_t _clock_per = STEP_PULSE_LEN){
    if (dir) GPIO_Set(DIR_OUT_PIN);
    else GPIO_Clear(DIR_OUT_PIN);

    GPIO_Set(STEP_OUT_PIN);
    delayClocks(_clock_per);
    GPIO_Clear(STEP_OUT_PIN);
}

inline void IRAM_ATTR stepOnce_Pos(){
    GPIO_Set(DIR_OUT_PIN);
    GPIO_Set(STEP_OUT_PIN);
    delayClocks(STEP_PULSE_LEN);
    GPIO_Clear(STEP_OUT_PIN);
}

inline void IRAM_ATTR stepOnce_Neg(){
    GPIO_Clear(DIR_OUT_PIN);
    GPIO_Set(STEP_OUT_PIN);
    delayClocks(STEP_PULSE_LEN);
    GPIO_Clear(STEP_OUT_PIN);
}

//==============================================================================

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

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
inline float stepsToDeg(const int32_t steps){
    return ((float)steps / (float)StepsPerRev) * 360.0f;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Return closest step to supplied angle
inline IRAM_ATTR uint16_t getClosestStep(const float v_in){
    return min(uint16_t(lroundf((v_in / 360.0f) * (float)StepsPerRev)), StepsPerRev);
}

IRAM_ATTR float getCalib(const uint16_t step);