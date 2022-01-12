/****************************************************
  Author: Scott Mudge
  Date: 09 Jan 2022
  File: Listener.h
  https://scottmudge.com
   
  Description: Class for interrupt-based listening 
    and filtering of dir/step signals.

  
  NOTES:
    
    A non-standard "ISR" is used here. The standard Arduino attachInterrupt()
    method produces response times which are too slow. Even on 240 MHz ESP32,
    the response times yield a maximum cycle time of ~260 kHz.

    Using this "manual" ISR method below (dedicating ALL of Core1 to monitoring
    the GPIO), a very quick response time is achieved, yielding a maximum cycle
    time of 5-10 MHz, an order of magnitude higher.

    Generally speaking, no step pulses are missed at standard frequencies.
***************************************************/

#include "Listener.h"

#include <ESPArduinoPins.h>
#include <soc/rtc_wdt.h>

#include "Utility.h"

static bool Attached = false;

// Very fast GPIO Reg Functions
#define GPIO_Set(x) (REG_WRITE(GPIO_OUT_W1TS_REG, 1 << (uint32_t)x))
#define GPIO_Clear(x) (REG_WRITE(GPIO_OUT_W1TC_REG, 1 << (uint32_t)x))
#define GPIO_IN_Read(x) (REG_READ(GPIO_IN_REG) & (1 << x))
#define GPIO_IN_Read_Byte(x) ((REG_READ(GPIO_IN_REG) & (1 << x)) ? 1U : 0U)
#define GPIO_IN_ReadAll() (REG_READ(GPIO_IN_REG))

// ~5 nanoseconds per clock
#define CLOCK_LEN_NS 5

// Task/Thread Vars
#define STACK_SIZE 8192
TaskHandle_t ListenTask;

// Pin definitions
#define STEP_IN_PIN ESP32_D26
#define STEP_OUT_PIN ESP32_D14
#define DIR_IN_PIN ESP32_D27
#define DIR_OUT_PIN ESP32_D12

#define USE_INTERRUPT_GUARD false

namespace Listener {

// Utility Functions
//==============================================================================
inline uint32_t IRAM_ATTR clocks() {
    uint32_t ccount;
    asm volatile("rsr %0, ccount"
                 : "=a"(ccount));
    return ccount;
}

uint64_t IRAM_ATTR millis() { return esp_timer_get_time() / 1000l; }

uint64_t IRAM_ATTR micros() { return esp_timer_get_time(); }

void IRAM_ATTR delayMicroseconds(uint32_t us) {
    if (us) {
        uint32_t m = micros();
        while ((micros() - m) < us) {
            asm(" nop");
        }
    }
}

void inline delayClocks(uint32_t clks) {
    uint32_t c = clocks();
    while ((clocks() - c) < clks) {
        asm(" nop");
    }
}
//==============================================================================

// ISR for dir pin
//==============================================================================
void IRAM_ATTR ListenerFunc(void* p) {
    // Initial delay
    vTaskDelay(1000);

    // Data
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Efficient data containers, storing levels
    register union {
        uint32_t _d = 0;
        struct {
            uint8_t b0;
            uint8_t b1;
            uint8_t b2;
            uint8_t b3;
        };
    } level;

    // Extra data container
    register union {
        uint32_t _d = 0;
        struct {
            uint8_t b0;
            uint8_t b1;
            uint8_t b2;
            uint8_t b3;
        };
    } data;

    uint32_t c1 = clocks();
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    // Main Loop
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    while (1) {  // the superloop

#if (USE_INTERRUPT_GUARD == true)
        portDISABLE_INTERRUPTS();
#endif

        // if (data.b0){
        //     data.b0 = 0;
        //     GPIO_Clear(STEP_OUT_PIN);
        //     //digitalWrite(STEP_OUT_PIN, LOW);
        // }

        level.b0 = GPIO_IN_Read_Byte(STEP_IN_PIN);

        if (level.b0 != level.b1) {  // only rising edges, adjust RTOS2 too !
            if (level.b1 == 0) {
                //if (level != old_level) {  // both edges

                GPIO_Set(STEP_OUT_PIN);
                delayClocks(20);
                GPIO_Clear(STEP_OUT_PIN);
                //delayClocks(50);
                //vTaskDelay(500);
                //GPIO_Clear(STEP_OUT_PIN);
                //Irqs++;
            }
            level.b1 = level.b0;
        }

#if (USE_INTERRUPT_GUARD == true)
        portENABLE_INTERRUPTS();
#endif
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
}
//==============================================================================

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void startListener(void) {
    
    xTaskCreatePinnedToCore(
        ListenerFunc,
        "Lst",
        STACK_SIZE,
        (void*)1,
        tskIDLE_PRIORITY + 2,
        &ListenTask,
        1);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void attach() {
    if (!Attached) {
        //disableCore0WDT();

        // Use pulldown since we're using RISING
        pinMode(STEP_IN_PIN, INPUT_PULLDOWN);
        pinMode(STEP_OUT_PIN, OUTPUT);

        pinMode(DIR_IN_PIN, INPUT);
        pinMode(DIR_OUT_PIN, OUTPUT);

        GPIO_Clear(STEP_OUT_PIN);

        const uint8_t level = GPIO_IN_Read(DIR_IN_PIN);
        if (level == 0)
            GPIO_Clear(DIR_OUT_PIN);
        else
            GPIO_Set(DIR_OUT_PIN);

        startListener();

        DbgLn("Core0 attached");
        Attached = true;
    }
}

}  // namespace Listener