#include <Arduino.h>
#include "Utility.h"
#include "calib.h"

IRAM_ATTR float getCalib(const uint16_t step){
    return CalibData[min(step, (uint16_t)(StepsPerRev - 1U))];
}