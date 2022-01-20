/****************************************************
  Firmware for MagStep closed-loop stepper motor.
  Author: Scott Mudge
  Date: 09 Jan 2022
  File: AS5600.cpp 
  https://scottmudge.com
   
  Description: Configuration file
***************************************************/

#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <Arduino.h>

/*  The number of frames used in the moving average filter. If set to 0 or 1, filter is bypassed.
    Max size is 255U.    
*/
#define FILTER_FRAME_SIZE 0
#define ENABLE_CALIBRATION 1
#define ENCODER_TASK_CORE 1
#define ENCODER_I2C_CLOCK 100000
#define ENCODER_TASK_PRIORITY 4

static constexpr uint8_t Microsteps       = 8U;
static constexpr uint16_t BaseStepsPerRev = 200U;
static constexpr uint16_t StepsPerRev     = Microsteps * BaseStepsPerRev;
static constexpr float DegsPerStep        = 360.0f / (float)StepsPerRev;
static constexpr float StepsPerDeg        = 1.0f / DegsPerStep;

static constexpr float StepCorr_Thresh_deg      = 1.0f;
static constexpr uint32_t StepCorr_Thresh_steps = (uint32_t)(StepCorr_Thresh_deg * StepsPerDeg);

#endif // __CONFIG_H__