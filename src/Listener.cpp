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

// Clock cycles
#define STEP_PULSE_LEN 500
#define STEP_PULSE_LEN_MAX 5000

// Task/Thread Vars
#define STACK_SIZE 8192
TaskHandle_t ListenTask;

// Pin definitions
#define STEP_IN_PIN ESP32_D26
#define STEP_OUT_PIN ESP32_D14
#define DIR_IN_PIN ESP32_D27
#define DIR_OUT_PIN ESP32_D12

#define STEP_IN_GPIO (gpio_num_t)STEP_IN_PIN

#define USE_INTERRUPT_GUARD false

// Queue stuff
static xQueueHandle StepIn_Queue = NULL;
static TaskHandle_t StepIn_Task = NULL;

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

// QUEUE HANDLER
//==============================================================================
void IRAM_ATTR gpioQueueHandler(void* p){
    uint8_t dir;
    for(;;) {
        if(xQueueReceive(StepIn_Queue, &dir, 1)) {
            if (dir)
                stepOnce_Pos();
            else
                stepOnce_Neg();
        }
    }
}

//==============================================================================
volatile uint32_t last_clock = 0;
volatile uint32_t last_time = STEP_PULSE_LEN;
void IRAM_ATTR stepISR(void* p){
    if (GPIO_IN_Read(STEP_IN_PIN)){
        const uint8_t dir = (uint8_t)GPIO_IN_Read_Byte(DIR_IN_PIN);
        stepDir(dir, last_time);
        last_clock = clocks();
    }
    else
        last_time =// min((max(((clocks() - last_clock) - CLOCK_OFFSET), (uint32_t)STEP_PULSE_LEN)), (uint32_t)STEP_PULSE_LEN_MAX);
         min(((clocks() - last_clock) / 2U), (uint32_t)STEP_PULSE_LEN_MAX);
}

void setupInterrupt(){
    //StepIn_Queue = xQueueCreate(10, sizeof(uint8_t));
    //xTaskCreate(gpioQueueHandler, "stepqh", 4096, NULL, 10, &StepIn_Task);

    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_pad_select_gpio(STEP_IN_GPIO);
    gpio_set_direction(STEP_IN_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(STEP_IN_GPIO, GPIO_PULLDOWN_ONLY);
    gpio_set_intr_type(STEP_IN_GPIO, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(STEP_IN_GPIO, stepISR, NULL);
    gpio_intr_enable(STEP_IN_GPIO);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void attach() {
    if (!Attached) {
        //disableCore0WDT();

        // Use pulldown since we're using RISING
        //pinMode(STEP_IN_PIN, INPUT_PULLDOWN);


        pinMode(STEP_OUT_PIN, OUTPUT);

        pinMode(DIR_IN_PIN, INPUT);
        pinMode(DIR_OUT_PIN, OUTPUT);

        GPIO_Clear(STEP_OUT_PIN);

        const uint8_t level = GPIO_IN_Read(DIR_IN_PIN);
        if (level == 0)
            GPIO_Clear(DIR_OUT_PIN);
        else
            GPIO_Set(DIR_OUT_PIN);

        //startListener();

        setupInterrupt();

        DbgLn("Core0 attached");
        Attached = true;
    }
}

}  // namespace Listener