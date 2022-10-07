#pragma once

#include "../Configuration.hpp"
#include "inc/Globals.hpp"
PUSH_NO_WARNINGS
#include <AccelStepper.h>
POP_NO_WARNINGS

#include "Utility.hpp"
#include "Mount.hpp"
#include "MeadeCommandProcessor.hpp"

// TODO: we have to change driver type to DRIVER_TYPE_TMC2209 and add a new definition for the actual mode (e.g. DRIVER_MODE_UART)
#if (AZ_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART) || (ALT_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART)
PUSH_NO_WARNINGS
    #include <TMCStepper.h>
POP_NO_WARNINGS
#endif

#define AZmotorPin1 AZ_STEP_PIN
#define AZmotorPin2 AZ_DIR_PIN


#define ALTmotorPin1 ALT_STEP_PIN
#define ALTmotorPin2 ALT_DIR_PIN

// End Stepper Definitions //////////////
/////////////////////////////////////////

/////////////////////////////////////////
// Driver definitions ///////////////////
#if (RA_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART) || (DEC_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART)                                          \
    || (AZ_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART) || (ALT_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART)                                       \
    || (FOCUS_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART)
    #define R_SENSE 0.11f  // 0.11 for StepStick
#endif
// End Driver Definitions ///////////////
/////////////////////////////////////////