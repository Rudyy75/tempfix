// //libraries
#include <Arduino.h>
#include <vector>
#include "blitz.hpp"
#include "blitz_timer.cpp"
//#include "stdlib.h"

// pinmap
#include "pinmap/PinMap_base.h"
#include "pinmap/pinmap_wall.h"


// constants
//#include "constants/constants.cpp"
#include "constants/wall_const.cpp"


// read write

// functions
#include "functions/store_data.cpp"
#include "functions/timer_cb.cpp"
#include "functions/SetMotorSpeed.cpp"



#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <TFLI2C.h>