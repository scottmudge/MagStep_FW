/*
    Encoder monitor, monitors how rotations are handled.

    Author: Scott Mudge
            mail@scottmudge.com
            https://scottmudge.com

    Date: 07 Jan 2022
*/
#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <Arduino.h>
#include <MovingAverage.h>

#include "Config.h"

/* 
    This monitors an encoder which goes from 0 - 360.0 degrees, and then 
    ensures that the total angle is tracked and maintained.
*/
class EncoderMonitor{
public:
    EncoderMonitor();
    ~EncoderMonitor();

    // Update the current angle
    void update(float angle);

    // Get total angle
    float getTotal() const;

    int32_t getTotalRevs() const;

private:
    bool m_Initialized = false;

    float 
        m_Dir           = 0.0f,
        m_TotalAngle    = 0.0f,
        m_LastAngle     = 0.0f;

    int32_t m_Revs = 0U;

    static constexpr uint8_t m_FltSize = 
        (FILTER_FRAME_SIZE > 1 ? ((FILTER_FRAME_SIZE < UINT8_MAX) ? (uint8_t)FILTER_FRAME_SIZE : 255U) : 0U);

    MovingAverage<float> m_MovAvg;
};

#endif // __ENCODER_H__