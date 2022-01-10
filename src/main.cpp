/****************************************************
  Firmware for MagStep closed-loop stepper motor.
  Author: Scott Mudge
  Date: 09 Jan 2022
  File: AS5600.cpp 
  https://scottmudge.com
   
  Description: Main file
***************************************************/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPArduinoPins.h>
#include <MovingAverage.h>
#include <float.h>
MovingAverage<float> g_AngFlt(3);

#include <AS5600.h>

#include "Encoder.h"

AS5600 g_Magnet(ESP8266_D1, ESP8266_D2);
EncoderMonitor g_Enc;

uint32_t g_Millis = millis();
uint32_t g_Samp = 0U;
float g_Buf = 0.0f;

bool is_equal(const float& left, const float& right, const float& epsilon /*= 0.001*/) {
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

void setup() {
    Serial.begin(115200);
    // put your setup code here, to run once:
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();
}

void loop() {
    int mag_str = g_Magnet.getMagnetStrength();
    uint16_t ang = 0;
    static uint16_t ang_buf = UINT16_MAX;
    if (mag_str > 0){
        ang = g_Magnet.getRawAngle(true);
        const float fl_ang = g_Magnet.getRawAngleDegs();
        g_Enc.update(fl_ang);
        if (ang_buf != ang){
            ang_buf = ang;
            Serial.printf("%.3f\n", g_Enc.getTotal());
        }
    }

    // const float ang_f = ((float)ang / 4095.0f) * 360.0f;
    // g_AngFlt.push(ang_f);
    // ++g_Samp;

    // const uint32_t new_mils = millis();
    // const uint32_t elapsed = new_mils - g_Millis;
    // const uint32_t output_ms = 10;

    // if (elapsed > output_ms) {
    //     g_Millis = new_mils;
    //     if (mag_str > 0) {
    //         const float new_ang = g_AngFlt.get();
    //         if (!is_equal(new_ang, g_Buf, 0.001)) {
    //             g_Buf = new_ang;
    //             Serial.printf("%.2f - %u/s\n", g_Buf, (g_Samp * 1000 / output_ms));
    //         }
    //     }
    //     g_Samp = 0;
    // }

    // if (mag_str > 0){
    //     sprintf(buf, "%s - %u\n", mag_str == 1 ? "Weak" : mag_str == 2 ? "Good" : "Far", ang);
    //     Serial.printf(buf);
    // }
    // else {
    //     Serial.printf("No\n");
    //     delay(10);
    // }
}