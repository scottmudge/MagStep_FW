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

/*  The number of frames used in the moving average filter. If set to 0 or 1, filter is bypassed.
    Max size is 255U.    
*/
#define FILTER_FRAME_SIZE 0

#endif // __CONFIG_H__