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
AS5600* magnet = nullptr;
EncoderMonitor encoder;
TwoWire wire = TwoWire(1);

//==============================================================================
void setup() {
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
    wire.begin(ESP32_D5, ESP32_D4, 500000);
    magnet = new AS5600(&wire);
    delay(20);
    Listener::attach();

    // put your setup code here, to run once:
}

//==============================================================================
void loop() {
    static uint32_t cur_mils = millis();
    const uint32_t new_mils = millis();
    const uint32_t elapsed_ms = (new_mils - cur_mils);

    int mag_str = magnet->getMagnetStrength();
    if (mag_str < 0) {
        DbgLn("Err");
    }
    uint16_t ang = 0;
    if (mag_str > 0) {
        ang = magnet->getRawAngle(true);
        if (magnet->getError() == AS5600_OK) {
            const float fl_ang = magnet->getRawAngleDegs();
            encoder.update(fl_ang);

            if (elapsed_ms > 500) {
                cur_mils = new_mils;
                Serial.printf("%s - %.3f\n", mag_str == 0 ? "None" : (mag_str == 1 ? "Weak" : mag_str == 2 ? "Good"
                                                                                                           : "Far"),
                              encoder.getTotal());
            }
        }
    }
    delay(1);
}