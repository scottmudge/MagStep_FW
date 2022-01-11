#include "Listener.h"
#include "Utility.h"

static uint8_t DirPin_In = 0;
static uint8_t StepPin_In = 0;
static uint8_t DirPin_Out = 0;
static uint8_t StepPin_Out = 0;
static bool Attached = false;

namespace Listener {
    
// ISR for dir pin
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
static void IRAM_ATTR STEP_ISR() {
    // register uint8_t dir = (uint8_t)digitalRead(DirPin_In);
    // digitalWrite(DirPin_Out, dir);

    gpio_set_level((gpio_num_t)StepPin_Out, 1U);
    gpio_set_level((gpio_num_t)StepPin_Out, 0U);
    //digitalWrite(StepPin_Out, HIGH);
    //digitalWrite(StepPin_Out, LOW);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void attach(const uint8_t dir_pin_in, const uint8_t step_pin_in,
    const uint8_t dir_pin_out, const uint8_t step_pin_out) {
    if (!Attached) {
        DirPin_In = dir_pin_in;
        StepPin_In = step_pin_in;
        DirPin_Out = dir_pin_out;
        StepPin_Out = step_pin_out;

        // Use pulldown since we're using RISING
        pinMode(StepPin_In, INPUT_PULLDOWN);
        pinMode(DirPin_In, INPUT);

        pinMode(StepPin_Out, OUTPUT);
        pinMode(DirPin_Out, OUTPUT);

        digitalWrite(StepPin_Out, LOW);
        digitalWrite(DirPin_Out, digitalRead(DirPin_In));
        
        attachInterrupt(StepPin_In, STEP_ISR, RISING);

        DbgLn("ISRs attached");
        Attached = true;
    }
}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void detach() {
    if (Attached){
        detachInterrupt(StepPin_In);
        Attached = false;
    }
}

}  // namespace Listener