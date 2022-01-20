/****************************************************
  Firmware for MagStep closed-loop stepper motor.
  Author: Scott Mudge
  Date: 09 Jan 2022
  File: AS5600.cpp 
  https://scottmudge.com
   
  Description: Main file
***************************************************/
#pragma GCC optimize("O3")

#include <Arduino.h>
#include <ESPArduinoPins.h>
#include <MovingAverage.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include <float.h>
MovingAverage<float> g_AngFlt(3);

#define DISABLE_WIFI_EXPLICIT false

#if (DISABLE_WIFI_EXPLICIT == true)
#include <WiFi.h>
#include <esp_bt.h>
#include <esp_wifi.h>
#endif

#include "AS5600.h"
#include "Encoder.h"
#include "Listener.h"
#include "Utility.h"

uint32_t g_Millis = millis();
uint32_t g_Samp = 0U;
float g_Buf = 0.0f;
// uint8_t DegPulsed[360];
// uint8_t StepPulsed[200];
// uint8_t MicrostepPulsed[StepsPerRev];
uint32_t CurDeg = 0U;
uint32_t CurMicrostep = 0U;

//==============================================================================
void IRAM_ATTR setup() {
#ifdef DEBUG_OUTPUT
    Serial.begin(115200);
#endif
    delay(5);

#if (DISABLE_WIFI_EXPLICIT == true)
    //WiFi.mode(WIFI_OFF);
    esp_wifi_set_mode(WIFI_MODE_NULL);
    esp_wifi_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
#endif
    EncoderMonitor::startEncoderTask();
    delay(20);
    Listener::attach();
    delay(20);
    EncoderMonitor::startEncoderTask();

    // for (uint8_t& v : DegPulsed) v = 0U;
    // for (uint8_t& v : StepPulsed) v = 0U;
    // for (uint8_t& v : MicrostepPulsed) v = 0U;

    // auto start = micros();
    // for (int j = 0; j < 1000; j++){
    //     for (int i = 0; i < 200; i++){
    //         for (uint8_t& v : MicrostepPulsed) v = i;
    //     }
    // }
    // for (uint8_t& v : MicrostepPulsed) v = 0U;
    // auto elapsed = micros() - start;
    // Serial.printf("Elapsed: %.6f ms\n", ((float)(elapsed)/1000000.0f));

    delay(1000);
    // put your setup code here, to run once:
}

//==============================================================================
void IRAM_ATTR loop() {
    delay(10);

    // static uint32_t cur_mils = millis();
    // static uint32_t stable_millis = millis();
    // static int32_t stable_buf = 0;
    // static int8_t is_mag_lost = -1;
    // const uint32_t new_mils = millis();
    // const uint32_t elapsed_ms = (new_mils - cur_mils);

/*
    if (mag_str > 0) {
        if (magnet->getError() == AS5600_OK) {
            const float fl_ang = magnet->getRawAngleDegs(true);
            EncoderMonitor::update(fl_ang);

            const int32_t expected_steps = Listener::totalSteps();
            const float expected_angle = stepsToDeg(expected_steps);
            const uint32_t cur_microstep = (uint32_t)Listener::totalSteps();
            const float abs_angle = EncoderMonitor::getLastAngle();
            const float encoder_angle = EncoderMonitor::getTotal();

            static float last_expected_angle = 0.0f;
            static float last_abs_angle = 0.0f;
            static float last_encoder_angle = 0.0f;
            static uint32_t last_used_microstep = 0U;
*/

#ifdef CALIB_MODE
            bool stable = false;
            if (stable_buf != expected_steps) {
                stable_millis = new_mils;
                stable_buf = expected_steps;
            } else if ((new_mils - stable_millis) > 750) {
                stable = true;
            }

            //int32_t base_step = expected_steps / Microsteps;
            //if ((expected_steps % Microsteps) != 0) base_step = -1;

            //if (stable && base_step >= 0){
            if (stable) {
                // const uint32_t cur_deg = (uint32_t)(roundf(expected_angle));
                // const float absdif = fabsf(expected_angle - (float)cur_deg);
                // if (!DegPulsed[cur_deg] && (absdif < 0.05)){

                //if (!StepPulsed[base_step]) {
                if (cur_microstep < sizeof(MicrostepPulsed) && !MicrostepPulsed[cur_microstep]) {
                    //if (cur_deg == 0 && base_step == 0){
                    if (cur_microstep == 0) {
                        Serial.printf("Microstep, Encoder, Expected, Abs Angle, Exp-Abs Diff\n");
                    } else {
                        const uint32_t diff = (cur_microstep - last_used_microstep);
                        if (diff > 1) {
                            if (diff == 2) {
                                const float interp_abs_angle = (last_abs_angle + abs_angle) / 2.0f;
                                const float interp_expected_angle = (last_expected_angle + expected_angle) / 2.0f;
                                const float interp_encoder_angle = (last_encoder_angle + encoder_angle) / 2.0f;
                                const float interp_diff_angle = interp_expected_angle - interp_abs_angle;

                                Serial.printf("%u, %.4f, %.4f, %.4f, %.4f\n", cur_microstep - 1,
                                              interp_encoder_angle, interp_expected_angle, interp_abs_angle, interp_diff_angle);
                                MicrostepPulsed[cur_microstep - 1] = 1;
                            } else {
                                Serial.printf("ERROR -- Diff == %u\n", diff);
                            }
                        }
                    }

                    const float diff_angle = expected_angle - abs_angle;

                    Serial.printf("%u, %.4f, %.4f, %.4f, %.4f\n", cur_microstep,
                                  encoder_angle, expected_angle, abs_angle, diff_angle);

                    last_abs_angle = abs_angle;
                    last_expected_angle = expected_angle;
                    last_encoder_angle = encoder_angle;
                    last_used_microstep = cur_microstep;

                    ///DegPulsed[cur_deg] = true;
                    //StepPulsed[base_step] = true;
                    MicrostepPulsed[cur_microstep] = 1;
                }
            }
#endif

            // const float cur_deg = (float)CurDeg;
            // float absfd = fabsf(cur_deg - expected_angle);
            // if (absfd < 0.005)

    //         if (elapsed_ms > 500) {
    //             cur_mils = new_mils;
    //             Serial.printf("Ex St: %d, Enc Dg: %.4f, Ex Dg: %.4f, Diff: %.4f, Act Dg: %.4f, Raw Dg: %.4f, Calib: %.4f\n",
    //             expected_steps, encoder_angle, expected_angle, fabsf(encoder_angle - expected_angle), abs_angle, fl_ang, getCalib(getClosestStep(fl_ang)));
    //             //Serial.printf("TEST - Last Ang: %.4f\n", g_Encoder.getLastAngle());
    //         }
    //     }
    // }

    //delayMicroseconds(10);
}