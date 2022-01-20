/****************************************************
  Author: Scott Mudge
  Date: 09 Jan 2022
  File: Listener.h
  https://scottmudge.com
   
  Description: Class for interrupt-based listening 
    and filtering of dir/step signals.

  
  NOTES:
    
    How to compensate for lost steps while maintaining position?

    Max ticks in an int32 before roll-over = 51200 steps per rev
    -2147483647 - 2147483647 = max int32
    2147483647 / 51200 = **41943 Revs maximum** before int overflow!!

    200 steps per rev * 256 microsteps = 

    1. ISR performs two functions:

        1. Keep track of incoming steps, use this to estimate appropriate position. 
        2. 

    
***************************************************/

#include "Listener.h"

#include <soc/rtc_wdt.h>

#include "Utility.h"

static bool Attached = false;

// Task/Thread Vars
#define STACK_SIZE 8192
TaskHandle_t ListenTask;

#define STEP_IN_GPIO ((gpio_num_t)STEP_IN_PIN)
#define USE_INTERRUPT_GUARD false

// Queue stuff
//static xQueueHandle StepIn_Queue = NULL;
//static TaskHandle_t StepIn_Task = NULL;

namespace Listener {

// QUEUE HANDLER
//==============================================================================
// void IRAM_ATTR gpioQueueHandler(void* p){
//     uint8_t dir;
//     for(;;) {
//         if(xQueueReceive(StepIn_Queue, &dir, 1)) {
//             if (dir)
//                 stepOnce_Pos();
//             else
//                 stepOnce_Neg();
//         }
//     }
// }

//==============================================================================
volatile uint32_t g_LastClock = 0;
volatile uint32_t g_LastTime = STEP_PULSE_LEN;
volatile int32_t g_TotalSteps = 0;

void IRAM_ATTR stepISR(void* p){
    // Positive
    if (GPIO_IN_Read(STEP_IN_PIN)){
        const uint8_t dir = (uint8_t)GPIO_IN_Read_Byte(DIR_IN_PIN);
        if (dir) ++g_TotalSteps;
        else --g_TotalSteps;
        stepDir(dir, g_LastTime);
        g_LastClock = clocks();
    }
    else
        g_LastTime =// min((max(((clocks() - last_clock) - CLOCK_OFFSET), (uint32_t)STEP_PULSE_LEN)), (uint32_t)STEP_PULSE_LEN_MAX);
         min(((clocks() - g_LastClock) / 2U), (uint32_t)STEP_PULSE_LEN_MAX);
}

//==============================================================================
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

//==============================================================================
void suspendSteps(){ }

//==============================================================================
void resumeSteps(){ }

//==============================================================================
void attach() {
    if (!Attached) {
        //disableCore0WDT();
        pinMode(STEP_OUT_PIN, OUTPUT);

        pinMode(DIR_IN_PIN, INPUT);
        pinMode(DIR_OUT_PIN, OUTPUT);

        GPIO_Clear(STEP_OUT_PIN);

        const uint8_t level = GPIO_IN_Read(DIR_IN_PIN);
        if (level == 0)
            GPIO_Clear(DIR_OUT_PIN);
        else
            GPIO_Set(DIR_OUT_PIN);

        setupInterrupt();
        DbgLn("ISR attached");
        Attached = true;
    }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
int32_t totalSteps() {
    return g_TotalSteps;
}

}  // namespace Listener