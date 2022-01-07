/****************************************************
  AS5600 class for Arduino platform
  Author: Scott Mudge, based on code from Tom Denton
  Date: 09 Jan 2022
  File: AS5600.cpp 
  Version 1.00
  https://scottmudge.com
   
  Description: This class is written to access the AS5600
    over I2C.
***************************************************/

#include <Arduino.h>
#include <Wire.h>

#include "AS5600.h"

// Registers
//==============================================================================
// Inputs
static constexpr uint8_t AS5600_Reg_ZMco        = 0x00;
static constexpr uint8_t AS5600_Reg_ZPos_Hi     = 0x01;
static constexpr uint8_t AS5600_Reg_ZPos_Lo     = 0x02;
static constexpr uint8_t AS5600_Reg_MPos_Hi     = 0x03;
static constexpr uint8_t AS5600_Reg_MPos_Lo     = 0x04;
static constexpr uint8_t AS5600_Reg_MAng_Hi     = 0x05;
static constexpr uint8_t AS5600_Reg_MAng_Lo     = 0x06;
static constexpr uint8_t AS5600_Reg_Config_Hi   = 0x07;
static constexpr uint8_t AS5600_Reg_Config_Lo   = 0x08;

// Outputs
static constexpr uint8_t AS5600_Reg_RawAng_Hi   = 0x0C;
static constexpr uint8_t AS5600_Reg_RawAng_Lo   = 0x0D;
static constexpr uint8_t AS5600_Reg_Angle_Hi    = 0x0E;
static constexpr uint8_t AS5600_Reg_Angle_Lo    = 0x0F;

// Status
static constexpr uint8_t AS5600_Reg_Status      = 0x0B;
static constexpr uint8_t AS5600_Reg_Agc         = 0x1A;
static constexpr uint8_t AS5600_Reg_Magn_Hi     = 0x1B;
static constexpr uint8_t AS5600_Reg_Magn_Lo     = 0x1C;

//Burn
static constexpr uint8_t AS5600_Reg_Burn        = 0xFF;

// Config funcs
//==============================================================================
uint16_t AS5600Config::getConfigData(){
    return uint16_t((getHighByte() << 8) & getLowByte());
}

uint8_t AS5600Config::getLowByte(){
    return uint8_t(
        powerMode & 
        hysteresisMode & 
        outputMode & 
        pwmFreq 
    );
}

uint8_t AS5600Config::getHighByte(){
    return uint8_t(
        slowFilterMode &
        fastFilterMode &
        watchdogMode
    );
}

// Constructor class for AS5600, supplying arbitrary I2C Wire
//==============================================================================
AS5600::AS5600(TwoWire *wire /*= nullptr*/) {
    _wire = wire;
    init();
}

// Constructor with arbitrary SDA and SCL pins
//==============================================================================
AS5600::AS5600(const uint8_t sda, const uint8_t scl){
    _wire = new TwoWire();
    _wire->begin((int)sda, (int)scl);
    init();
}

// Shared initialization code
//==============================================================================
void AS5600::init(){
    _wire->setClock(100000U);
}

//==============================================================================
AS5600Config& AS5600::getConfig() {
    return _config;
}

//==============================================================================
void AS5600::setPowerMode(AS5600_PowerMode_t power_mode){
    _config.powerMode = power_mode;
    updateConfig();        
}

//==============================================================================
void AS5600::setHysteresisMode(AS5600_HysteresisMode_t hyst_mode){
    _config.hysteresisMode = hyst_mode;
    updateConfig();        
}

//==============================================================================
void AS5600::setOutputMode(AS5600_OutputMode_t output_mode){
    _config.outputMode = output_mode;
    updateConfig();        
}

//==============================================================================
void AS5600::setPWMFreq(AS5600_PWMFrequency_t pwm_freq){
    _config.pwmFreq = pwm_freq;
    updateConfig();        
}

//==============================================================================
void AS5600::setSlowFilterMode(AS5600_SlowFilterMode_t slow_flt_mode){
    _config.slowFilterMode = slow_flt_mode;
    updateConfig();        
}

//==============================================================================
void AS5600::setFastFilterMode(AS5600_FastFilterMode_t fast_flt_mode){
    _config.fastFilterMode = fast_flt_mode;
    updateConfig();        
}

//==============================================================================
void AS5600::setWatchdogMode(AS5600_WatchdogMode_t wd_mode){
    _config.watchdogMode = wd_mode;
    updateConfig();        
}

//==============================================================================
void AS5600::setConfig(const AS5600Config& config) {
    _config = config;
    updateConfig();
}

// Returns I2C Address of Device
//==============================================================================
constexpr uint8_t AS5600::getAddress() {
    return AS5600_I2C_Addr;
}

/* Sets a value in maximum angle register.
  If no value is provided, method will read position of
  magnet.  Setting this register zeros out max position
  register.*/
//==============================================================================
uint16_t AS5600::setMaxAngle(uint16_t newMaxAngle) {
    uint16_t retVal;
    if (newMaxAngle == -1)
    {
        _maxAngle = getRawAngle();
    }
    else
        _maxAngle = newMaxAngle;

    writeOneByte(AS5600_Reg_MAng_Hi, highByte(_maxAngle));
    delay(2);
    writeOneByte(AS5600_Reg_MAng_Lo, lowByte(_maxAngle));
    delay(2);

    retVal = readTwoBytes(AS5600_Reg_MAng_Hi, AS5600_Reg_MAng_Lo);
    return retVal;
}

/* Gets value of maximum angle register. */
//==============================================================================
uint16_t AS5600::getMaxAngle() {
    return readTwoBytes(AS5600_Reg_MAng_Hi, AS5600_Reg_MAng_Lo);
}

/* Sets a value in start position register.
  If no value is provided, method will read position of
  magnet. */
//==============================================================================
uint16_t AS5600::setStartPosition(uint16_t startAngle) {
    if (startAngle == -1)
    {
        _rawStartAngle = getRawAngle();
    }
    else
        _rawStartAngle = startAngle;

    writeOneByte(AS5600_Reg_ZPos_Hi, highByte(_rawStartAngle));
    delay(2);
    writeOneByte(AS5600_Reg_ZPos_Lo, lowByte(_rawStartAngle));
    delay(2);
    _zPosition = readTwoBytes(AS5600_Reg_ZPos_Hi, AS5600_Reg_ZPos_Lo);

    return (_zPosition);
}

/* Gets value of start position register. */
//==============================================================================
uint16_t AS5600::getStartPosition() {
    return readTwoBytes(AS5600_Reg_ZPos_Hi, AS5600_Reg_ZPos_Lo);
}

/* Sets a value in end position register.
  If no value is provided, method will read position of
  magnet. */
//==============================================================================
uint16_t AS5600::setEndPosition(uint16_t endAngle) {
    if (endAngle == -1)
        _rawEndAngle = getRawAngle();
    else
        _rawEndAngle = endAngle;

    writeOneByte(AS5600_Reg_MPos_Hi, highByte(_rawEndAngle));
    delay(2);
    writeOneByte(AS5600_Reg_MPos_Lo, lowByte(_rawEndAngle));
    delay(2);
    _mPosition = readTwoBytes(AS5600_Reg_MPos_Hi, AS5600_Reg_MPos_Lo);

    return (_mPosition);
}

/* Gets value of end position register. */
//==============================================================================
uint16_t AS5600::getEndPosition() {
    uint16_t retVal = readTwoBytes(AS5600_Reg_MPos_Hi, AS5600_Reg_MPos_Lo);
    return retVal;
}

/* Gets raw value of magnet position.
  start, end, and max angle settings do not apply */
//==============================================================================
uint16_t AS5600::getRawAngle() {
    return readTwoBytes(AS5600_Reg_RawAng_Hi, AS5600_Reg_RawAng_Lo);
}

/* Gets scaled value of magnet position.
  start, end, or max angle settings are used to 
  determine value */
//==============================================================================
uint16_t AS5600::getScaledAngle() {
    return readTwoBytes(AS5600_Reg_Angle_Hi, AS5600_Reg_Angle_Lo);
}

/* Out: 1 if magnet is detected, 0 if not
  Description: reads status register and examines the 
  MH bit */
//==============================================================================
int AS5600::detectMagnet() {
    int magStatus;
    int retVal = 0;
    /*0 0 MD ML MH 0 0 0*/
    /* MD high = magnet detected*/
    /* ML high = AGC Maximum overflow, magnet to weak*/
    /* MH high = AGC minimum overflow, Magnet to strong*/
    magStatus = readOneByte(AS5600_Reg_Status);

    if (magStatus & 0x20)
        retVal = 1;

    return retVal;
}

/* Out: 0 if no magnet is detected
       1 if magnet is to weak
       2 if magnet is just right
       3 if magnet is to strong
  Description: reads status register andexamins the MH,ML,MD bits */
//==============================================================================
int AS5600::getMagnetStrength() {
    int magStatus;
    int retVal = 0;
    /*0 0 MD ML MH 0 0 0*/
    /* MD high = magnet detected */
    /* ML high = AGC Maximum overflow, magnet to weak*/
    /* MH high = AGC minimum overflow, Magnet to strong*/
    magStatus = readOneByte(AS5600_Reg_Status);
    if (detectMagnet() == 1)
    {
        retVal = 2; /*just right */
        if (magStatus & 0x10)
            retVal = 1; /*to weak */
        else if (magStatus & 0x08)
            retVal = 3; /*to strong */
    }

    return retVal;
}

/* Gets value of AGC register. */
//==============================================================================
int AS5600::getAgc() {
    return readOneByte(AS5600_Reg_Agc);
}

/* Gets value of magnitude register. */
//==============================================================================
uint16_t AS5600::getMagnitude() {
    return readTwoBytes(AS5600_Reg_Magn_Hi, AS5600_Reg_Magn_Lo);
}

/* Sets value of CONF register. */
//==============================================================================
void AS5600::updateConfig() {
    writeOneByte(AS5600_Reg_Config_Hi, _config.getHighByte());
    delay(2);
    writeOneByte(AS5600_Reg_Config_Lo, _config.getLowByte());
    delay(2);
}

/* Determines how many times chip has been
  permanently written to. */
//==============================================================================
int AS5600::getBurnCount() {
    return readOneByte(AS5600_Reg_ZMco);
}

/* Burns start and end positions to chip.
  THIS CAN ONLY BE DONE 3 TIMES */
//==============================================================================
AS5600_Error_t AS5600::burnAngle() {
    AS5600_Error_t retVal = AS5600_OK;
    _zPosition = getStartPosition();
    _mPosition = getEndPosition();
    _maxAngle = getMaxAngle();

    if (detectMagnet() == 1)
    {
        if (getBurnCount() < 3)
        {
            if ((_zPosition == 0) && (_mPosition == 0))
                retVal = AS5600_Error_InvalidDistance;
            else
                writeOneByte(AS5600_Reg_Burn, 0x80);
        }
        else
            retVal = AS5600_Error_NoBurnLeft;
    }
    else
        retVal = AS5600_Error_NoMagnet;

    return retVal;
}

/* Burns max angle and config data to chip.
  THIS CAN ONLY BE DONE 1 TIME */
//==============================================================================
AS5600_Error_t AS5600::burnMaxAngleAndConfig() {
    AS5600_Error_t retVal = AS5600_OK;
    _maxAngle = getMaxAngle();

    if (getBurnCount() == 0)
    {
        if (_maxAngle * 0.087 < 18)
            retVal = AS5600_Error_InvalidDistance;
        else
            writeOneByte(AS5600_Reg_Burn, 0x40);
    }
    else
        retVal = AS5600_Error_NoBurnLeft;

    return retVal;
}

/* Reads one byte register from i2c */
//==============================================================================
int AS5600::readOneByte(int in_adr) {
    int retVal = -1;
    Wire.beginTransmission(getAddress());
    Wire.write(in_adr);
    Wire.endTransmission();
    Wire.requestFrom(getAddress(), 1U);
    while (Wire.available() == 0)
        ;
    retVal = Wire.read();

    return retVal;
}

/* Reads two bytes register from i2c */
//==============================================================================
uint16_t AS5600::readTwoBytes(int in_adr_hi, int in_adr_lo) {
    /* Read 2 Bytes */
    Wire.beginTransmission(getAddress());
    Wire.write(in_adr_hi);
    Wire.endTransmission();
    Wire.requestFrom(getAddress(), 2U);
    while (Wire.available() < 2)
        ;

    int high = Wire.read();
    int low = Wire.read();
    return (high << 8) | low;
}

/* Writes one byte to a i2c register */
//==============================================================================
void AS5600::writeOneByte(int adr_in, int dat_in) {
    Wire.beginTransmission(getAddress());
    Wire.write(adr_in);
    Wire.write(dat_in);
    Wire.endTransmission();
}

