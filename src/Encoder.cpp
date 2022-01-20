#include <Arduino.h>
#include <Wire.h>

#include "Encoder.h"

#include "AS5600.h"
#include "Config.h"
#include "Utility.h"
#include "Listener.h"

/*
    Encoder 

    Read encoder count *before* I2C/Encoder update, then *after* update. Then
    average them together, and use that to compare against encoder angle.

*/

namespace EncoderMonitor {
// Variables
//==============================================================================
// Degrees required to trigger rotation
static constexpr float Encoder_AngleDiffThresh = 350.0f;

// How many cycles to determine if we need to check the magnet strength
static constexpr uint16_t Encoder_MagCheckCount = 64U;
// How many milliseconds between checking.
static constexpr uint16_t Encoder_MagCheckPeriodMillis = 1000U;
// How many errors before invalidating the wire
static constexpr uint8_t Encoder_Max_Errors = 3U;

DRAM_ATTR bool g_Initialized = false;
DRAM_ATTR float g_StartAngle = 0.0f;
DRAM_ATTR float g_LastAngle = 0.0f;
DRAM_ATTR float g_TotalAngle = 0.0f;
DRAM_ATTR float g_Dir = 1.0f;

DRAM_ATTR int32_t g_Revs = 0;

static TaskHandle_t g_EncoderTask = 0;
static AS5600 g_Magnet;
static TwoWire g_Wire = TwoWire(1);

//==============================================================================
IRAM_ATTR void coreTask( void * pvParameters ){
    // Flag to check the magnet strength
    bool mag_good = false;
    bool reinit_wire = true;
    uint32_t mag_millis = millis();
    uint16_t mag_count = 0U;
    uint8_t num_errors = 0U;

    // #TESTING
    //==============================================================================
    uint32_t TESTING_millis = millis();
    static constexpr uint32_t TESTING_dump_period = 1000U;
    //==============================================================================
    
    // Main loop
    for(;;){

        // Check to init wire
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        if (reinit_wire){
            if (!g_Wire.begin(ESP32_D5, ESP32_D4, ENCODER_I2C_CLOCK)){
                DbgLn("I2C init fail");
                delay(10);
                continue;
            }

            g_Magnet.setWire(&g_Wire);
            reinit_wire = false;
            continue;
        }
    
        // Magnet check is good :)
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        if (mag_good){
            // Get initial angle
            float expected_angle = stepsToDeg(Listener::totalSteps());
            // Update encoder
            float encoder_angle = g_Magnet.getRawAngleDegs(true);
            // Find after angle, and find average.
            // This, in theory. estimates the total encoder count during the middle of the encoder update.
            expected_angle = (expected_angle + stepsToDeg(Listener::totalSteps())) / 2.0f;

            // Everything is good with magnet read
            if (g_Magnet.getError(true) == AS5600_OK){

                // Do stuff w/ magnet data here
                if (num_errors) num_errors = 0U;

                // Main magnet data management here!!!
                // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
                // Update the encoder tracker
                update(encoder_angle);
                // Get the total angle out
                encoder_angle = EncoderMonitor::getTotal();

                // #TESTING
                //==============================================================================
                const uint32_t new_millis = millis();
                if ((new_millis - TESTING_millis) > TESTING_dump_period){
                    TESTING_millis = new_millis;
                    DbgLn("Int Ang: %.4f, Enc Ang: %.4f, Diff: %.4f", expected_angle, encoder_angle, (encoder_angle - expected_angle));
                }
                //==============================================================================
                // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
                
                // Check mag timer
                if (++mag_count >= Encoder_MagCheckCount) {
                    const uint32_t new_millis = millis();
                    // Check timer
                    if ((new_millis - mag_millis) >= Encoder_MagCheckPeriodMillis){
                        // Check the magnet
                        mag_good = false;
                        mag_millis = new_millis;
                    }
                    mag_count = 0U;
                }
            } 
            // Device conn. error
            else{
                if (++num_errors >= Encoder_Max_Errors) reinit_wire = true;
                continue;
            }
        } 
        // Need to check magnet
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        else{
            // Get mag strength
            const int mag_str = g_Magnet.getMagnetStrength();
            // If -1, error
            if (mag_str < 0) {
                // Check error threshold
                if (++num_errors >= Encoder_Max_Errors) reinit_wire = true;
                continue;
            }
            // Reset error counter
            if (num_errors) num_errors = 0U;
            // Check if magnet is present (non-zero)
            if (mag_str > 0) mag_good = true;
            // Otherwise keep flag false.
            continue;
        }
    } 
}

// Functions
//==============================================================================
void IRAM_ATTR startEncoderTask() {
    xTaskCreatePinnedToCore(coreTask, "enc", 20000, NULL,
     ENCODER_TASK_PRIORITY, &g_EncoderTask, ENCODER_TASK_CORE);
}

//==============================================================================
void IRAM_ATTR update(float angle) {
#if (ENABLE_CALIBRATION == 1)
    angle += getCalib(getClosestStep(angle));
#endif

    // Check if a rev has happened
    if (g_Initialized) {
        // If we're using a filter, process it here. This is optimized away if m_FltSize is set to <= 1
        // if (m_FltSize > 1) {
        //     m_MovAvg.push(angle);
        //     angle = m_MovAvg.get();
        // }
        const float dif = angle - g_LastAngle;

        g_Dir = (dif < 0.0f) ? -1.0f : (dif > 0.0f) ? 1.0f
                                                    : 0.0f;

        /*
                If diff goes from 359 -> 0.0 (diff == -359.0), that means we rotated once positively.
                If diff goes from 0.0 -> 359 (diff == 359.0), that means we rotated once negatively. 
            */

        // Revolution occured
        if (fabsf(dif) > Encoder_AngleDiffThresh) {
            // Positive revolution
            if (g_Dir < 0.0)
                ++g_Revs;
            else
                --g_Revs;
        }
    } else {
        g_StartAngle = angle;
        g_Initialized = true;
    }
    g_TotalAngle = (((float)g_Revs * 360.0f) + (angle - g_StartAngle));
    g_LastAngle = angle;
}

//==============================================================================
float IRAM_ATTR getTotal() {
    return g_TotalAngle;
}

//==============================================================================
float IRAM_ATTR getLastAngle() {
    return g_LastAngle;
}

//==============================================================================
int32_t IRAM_ATTR getTotalRevs() {
    return g_Revs;
}
}  // namespace EncoderMonitor