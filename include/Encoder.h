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

/* 
    This monitors an encoder which goes from 0 - 360.0 degrees, and then 
    ensures that the total angle is tracked and maintained.
*/
class EncoderMonitor{
public:
    EncoderMonitor();
    ~EncoderMonitor();

    // Update the current angle
    void update(const float angle);
    float getTotal() const;

private:
    float m_TotalAngle = 0.0f;

};

#endif // __ENCODER_H__