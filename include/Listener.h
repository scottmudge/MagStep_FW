/****************************************************
  Author: Scott Mudge
  Date: 09 Jan 2022
  File: Listener.h
  https://scottmudge.com
   
  Description: Class for interrupt-based listening 
    and filtering of dir/step signals.
***************************************************/

#ifndef __LISTENER_H__
#define __LISTENER_H__

#include <Arduino.h>

namespace Listener{
    void attach(const uint8_t dir_pin, const uint8_t step_pin,
    const uint8_t dir_pin_out, const uint8_t step_pin_out);
    void detach();
}
#endif  // __LISTENER_H__