/****************************************************
  AS5600 class for Arduino platform, with configuration modes
  Author: Scott Mudge
  Date: 09 Jan 2022
  File: AS5600.h 
  Version 1.00
  https://scottmudge.com
   
  Description: This class is written to access the AS5600
    over I2C.
***************************************************/

#ifndef AMS_5600_h
#define AMS_5600_h

// Includes
#include <Arduino.h>

// Forward Declarations
class TwoWire;

// Configurations - Various configuration modes/parameters
//  Enum values correspond to the correct values used in the config registers.
//  DO NOT CHANGE THEM
//==============================================================================
/*  Power Mode
    Polling rate can be reduced to reduce power consumption    

    LPM Mode    | Polling Time (ms)     | Avg Current (mA)
    ----------------------------------------------------------
    Normal      | 0.0                       | 6.5
    LPM1        | 5.0                       | 3.4
    LPM2        | 20.0                      | 1.8
    LPM3        | 100.0                     | 1.5

*/
enum AS5600_PowerMode_t {
    AS5600_PowerMode_Normal = 0b00000000,
    AS5600_PowerMode_LPM1   = 0b00000001,
    AS5600_PowerMode_LPM2   = 0b00000010,
    AS5600_PowerMode_LPM3   = 0b00000011
};

/*  Hysteresis Mode

    To avoid any toggling of the output when the magnet is not
    moving, a 1 to 3 LSB hysteresis of the 12-bit resolution can be
    enabled.
*/
enum AS5600_HysteresisMode_t {
    AS5600_HysteresisMode_Off   = 0b00000000,
    AS5600_HysteresisMode_1LSB  = 0b00000100,
    AS5600_HysteresisMode_2LSB  = 0b00001000,
    AS5600_HysteresisMode_3LSB  = 0b00001100
};

/*  Output Mode

    - Analog Full Output - 0 to 100%, from GND to VDD
    - Analog Reduced Output - 10 - 90%, from GND to VDD
    - Digital PWM Output - See datasheet for how this works.
*/
enum AS5600_OutputMode_t {
    AS5600_OutputMode_Analog_Full       = 0b00000000,
    AS5600_OutputMode_Analog_Reduced    = 0b00010000,
    AS5600_OutputMode_PWM               = 0b00100000
};

// PWM Frequency
enum AS5600_PWMFrequency_t {
    AS5600_PWMFrequency_115Hz   = 0b00000000,
    AS5600_PWMFrequency_230Hz   = 0b01000000,
    AS5600_PWMFrequency_460Hz   = 0b10000000,
    AS5600_PWMFrequency_920Hz   = 0b11000000
};

/*  Slow Filter Speed
    
    This is the default filter, unless the fast filter threshold is set (see below).
    This filter performs linear interpolation between response periods, to even out noise.

    According to datasheet:

    Speed   | Response Delay (ms)   | Max RMS Output Noise (Deg)
    ----------------------------------------------------------
    16x     | 2.2                   | 0.015
    8x      | 1.1                   | 0.021
    4x      | 0.55                  | 0.030
    2x      | 0.286                 | 0.043
*/
enum AS5600_SlowFilterMode_t {
    AS5600_SlowFilter_16x   = 0b00000000,
    AS5600_SlowFilter_8x    = 0b00000001,
    AS5600_SlowFilter_4x    = 0b00000010,
    AS5600_SlowFilter_2x    = 0b00000011
};

/*  Fast Filter Threshold

    NOTE: Setting this to anything but Off will OVERRIDE the Slow Filter!
*/
enum AS5600_FastFilterMode_t {
    AS5600_FastFilter_Off           = 0b00000000,
    AS5600_FastFilterThresh_6LSBs   = 0b00000100,
    AS5600_FastFilterThresh_7LSBs   = 0b00001000,
    AS5600_FastFilterThresh_9LSBs   = 0b00001100,
    AS5600_FastFilterThresh_10LSBs  = 0b00011100,
    AS5600_FastFilterThresh_18LSBs  = 0b00010000,
    AS5600_FastFilterThresh_21LSBs  = 0b00010100,
    AS5600_FastFilterThresh_24LSBs  = 0b00011000
};

// Watchdog Mode 
enum AS5600_WatchdogMode_t {
    AS5600_Watchdog_Off = 0b00000000,
    AS5600_Watchdog_On  = 0b00100000
};

// Error Types
//==============================================================================
enum AS5600_Error_t {
    AS5600_Error_Unknown = -1,
    AS5600_OK = 0,
    AS5600_Error_General,
    AS5600_Error_NoBurnLeft,
    AS5600_Error_NoMagnet,
    AS5600_Error_InvalidDistance
} ;

// Constants
//==============================================================================
// Address of the chip
static constexpr uint8_t AS5600_I2C_Addr        = 0x36;
// Doesn't get larger than 4095, 0-4095 == 2^12 range
static constexpr uint16_t AS5600_Val_Range      = 4096U; 
static constexpr uint16_t AS5600_Max_Raw_Val    = AS5600_Val_Range - 1U; 


// Configuration Structure, used for storing and creating config register data
//==============================================================================
struct AS5600Config {
    AS5600_PowerMode_t      powerMode       = AS5600_PowerMode_Normal;
    AS5600_HysteresisMode_t hysteresisMode  = AS5600_HysteresisMode_Off;
    AS5600_OutputMode_t     outputMode      = AS5600_OutputMode_Analog_Full;
    AS5600_PWMFrequency_t   pwmFreq         = AS5600_PWMFrequency_115Hz;
    AS5600_SlowFilterMode_t slowFilterMode  = AS5600_SlowFilter_16x;
    AS5600_FastFilterMode_t fastFilterMode  = AS5600_FastFilter_Off;
    AS5600_WatchdogMode_t   watchdogMode    = AS5600_Watchdog_Off;

    // Returns both low and high bytes
    uint16_t getConfigData();
    // Returns low config byte
    uint8_t getLowByte();
    // Returns high config byte
    uint8_t getHighByte();
};

class AS5600
{
public:
    // Constructor, pass arbitrary TwoWire, or pass arbitrary SDA/SCL pins
    AS5600(TwoWire *wire = nullptr);
    AS5600(const uint8_t sda, const uint8_t scl);

    constexpr uint8_t getAddress();

    // Returns mutable config, be sure to use "updateConfig()" after editing.
    AS5600Config& getConfig();
    // Sets the configuration from the one supplied, and updates it on the connected device
    void setConfig(const AS5600Config& config);

    // Config setters, these auto-update on device
    void setPowerMode(AS5600_PowerMode_t power_mode);
    void setHysteresisMode(AS5600_HysteresisMode_t hyst_mode);
    void setOutputMode(AS5600_OutputMode_t output_mode);
    void setPWMFreq(AS5600_PWMFrequency_t pwm_freq);
    void setSlowFilterMode(AS5600_SlowFilterMode_t slow_flt_mode);
    void setFastFilterMode(AS5600_FastFilterMode_t fast_flt_mode);
    void setWatchdogMode(AS5600_WatchdogMode_t wd_mode);
    
    uint16_t setMaxAngle(uint16_t newMaxAngle = -1);
    uint16_t getMaxAngle();

    uint16_t setStartPosition(uint16_t startAngle = -1);
    uint16_t getStartPosition();

    uint16_t setEndPosition(uint16_t endAngle = -1);
    uint16_t getEndPosition();

    // Call this to update the angle from sensor. Or use one of the getters below and pass "true"
    AS5600_Error_t updateRawAngle();
    AS5600_Error_t updateScaledAngle();

    // Get raw angle, right out of sensor
    uint16_t getRawAngle(const bool update = false);
    // Get raw angle, and normalize it for entire range
    float getRawAngleNormalized(const bool update = false);
    // Get raw angle in degrees (0 - 360.0)
    float getRawAngleDegs(const bool update = false);
    // Get raw angle in radians
    float getRawAngleRads (const bool update = false);

    // Get angle scaled to configuration
    uint16_t getScaledAngle(const bool update = false);
    // Get scaled angle, and normalize it for entire range
    float getScaledAngleNormalized(const bool update = false);
    // Get scaled angle in degrees (0 - 360.0)
    float getScaledAngleDegs(const bool update = false);
    // Get scaled angle in radians
    float getScaledAngleRads(const bool update = false);

    int detectMagnet();
    int getMagnetStrength();
    int getAgc();
    uint16_t getMagnitude();
    void updateConfig();

    int getBurnCount();
    AS5600_Error_t burnAngle();
    AS5600_Error_t burnMaxAngleAndConfig();

private:
    TwoWire* _wire;

    uint16_t _rawStartAngle;
    uint16_t _zPosition;
    uint16_t _rawEndAngle;
    uint16_t _mPosition;
    uint16_t _maxAngle;

    AS5600Config _config;

    uint16_t 
        _curRawAngle = 0U,
        _curScaledAngle = 0U;

    void init();
    int readOneByte(int in_adr);
    uint16_t readTwoBytes(int in_adr_hi, int in_adr_lo);
    void writeOneByte(int adr_in, int dat_in);
};
#endif