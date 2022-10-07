#include "../Configuration.hpp"
#include "Utility.hpp"
#include "Mount.hpp"
#include "libs/MappedDict/MappedDict.hpp"

PUSH_NO_WARNINGS
#include <AccelStepper.h>
#if (AZ_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART) || (ALT_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART)
    #include <TMCStepper.h>  // If you get an error here, download the TMCstepper library from "Tools > Manage Libraries"
#endif
POP_NO_WARNINGS

#define UART_CONNECTION_TEST_RETRIES 5

// Seconds per astronomical day (23h 56m 4.0905s)
const float secondsPerDay = 86164.0905f;

/////////////////////////////////
//
// CTOR
//
/////////////////////////////////
Mount::Mount()
{
}

/////////////////////////////////
//
// configureAZStepper / configureALTStepper
//
/////////////////////////////////
void Mount::configureAZStepper(byte pin1, byte pin2, int maxSpeed, int maxAcceleration)
{
    _stepperAZ = new AccelStepper(AccelStepper::DRIVER, pin1, pin2);
    _stepperAZ->setMaxSpeed(maxSpeed);
    _stepperAZ->setAcceleration(maxAcceleration);
    _maxAZSpeed        = maxSpeed;
    _maxAZAcceleration = maxAcceleration;
    #if AZ_INVERT_DIR == 1
    _stepperAZ->setPinsInverted(true, false, false);
    #endif
}
void Mount::configureALTStepper(byte pin1, byte pin2, int maxSpeed, int maxAcceleration)
{
    _stepperALT = new AccelStepper(AccelStepper::DRIVER, pin1, pin2);
    _stepperALT->setMaxSpeed(maxSpeed);
    _stepperALT->setAcceleration(maxAcceleration);
    _maxALTSpeed        = maxSpeed;
    _maxALTAcceleration = maxAcceleration;
    #if ALT_INVERT_DIR == 1
    _stepperALT->setPinsInverted(true, false, false);
    #endif
}


#if AZ_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART || ALT_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART
    #if UART_CONNECTION_TEST_TXRX == 1
bool Mount::connectToDriver(TMC2209Stepper *driver, const char *driverKind)
{
    LOG(DEBUG_STEPPERS, "[STEPPERS]: Testing UART Connection to %s driver...", driverKind);
    for (int i = 0; i < UART_CONNECTION_TEST_RETRIES; i++)
    {
        if (driver->test_connection() == 0)
        {
            LOG(DEBUG_STEPPERS, "[STEPPERS]: UART connection to %s driver successful.", driverKind);
            return true;
        }
        else
        {
            delay(500);
        }
    }
    LOG(DEBUG_STEPPERS, "[STEPPERS]: UART connection to %s driver failed.", driverKind);
    return false;
}
    #endif
#endif

/////////////////////////////////
//
// configureAZdriver
// TMC2209 UART only
/////////////////////////////////
#if AZ_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART
    #if SW_SERIAL_UART == 0
void Mount::configureAZdriver(Stream *serial, float rsense, byte driveraddress, int rmscurrent, int stallvalue)
{
    _driverAZ = new TMC2209Stepper(serial, rsense, driveraddress);
    _driverAZ->begin();
    bool UART_Rx_connected = false;
        #if UART_CONNECTION_TEST_TXRX == 1
    UART_Rx_connected = connectToDriver(_driverAZ, "AZ");
    if (!UART_Rx_connected)
    {
        digitalWrite(AZ_EN_PIN,
                     HIGH);  //Disable motor for safety reasons if UART connection fails to avoid operating at incorrect rms_current
    }
        #endif
    _driverAZ->toff(0);
        #if USE_VREF == 0  //By default, Vref is ignored when using UART to specify rms current.
    _driverAZ->I_scale_analog(false);
        #endif
    LOG(DEBUG_STEPPERS, "[MOUNT]: Requested AZ motor rms_current: %d mA", rmscurrent);
    _driverAZ->rms_current(rmscurrent, AZ_MOTOR_HOLD_SETTING / 100.0);  //holdMultiplier = 1 to set ihold = irun
    _driverAZ->toff(1);
    _driverAZ->en_spreadCycle(0);
    _driverAZ->blank_time(24);
    _driverAZ->microsteps(AZ_MICROSTEPPING == 1 ? 0 : AZ_MICROSTEPPING);  // If 1 then disable microstepping
    _driverAZ->TCOOLTHRS(0xFFFFF);                                        //xFFFFF);
    _driverAZ->semin(0);                                                  //disable CoolStep so that current is consistent
    _driverAZ->SGTHRS(stallvalue);
    if (UART_Rx_connected)
    {
        LOG(DEBUG_STEPPERS, "[MOUNT]: Actual AZ motor rms_current: %d mA", _driverAZ->rms_current());
        LOG(DEBUG_STEPPERS, "[MOUNT]: Actual AZ CS value: %d", _driverAZ->cs_actual());
        LOG(DEBUG_STEPPERS, "[MOUNT]: Actual AZ vsense: %d", _driverAZ->vsense());
    }
}

    #elif SW_SERIAL_UART == 1

void Mount::configureAZdriver(uint16_t AZ_SW_RX, uint16_t AZ_SW_TX, float rsense, byte driveraddress, int rmscurrent, int stallvalue)
{
    _driverAZ = new TMC2209Stepper(AZ_SW_RX, AZ_SW_TX, rsense, driveraddress);
    _driverAZ->beginSerial(19200);
    _driverAZ->mstep_reg_select(true);
    _driverAZ->pdn_disable(true);
    bool UART_Rx_connected = false;
        #if UART_CONNECTION_TEST_TXRX == 1
    UART_Rx_connected      = connectToDriver(_driverAZ, "AZ");
    if (!UART_Rx_connected)
    {
        digitalWrite(AZ_EN_PIN,
                     HIGH);  //Disable motor for safety reasons if UART connection fails to avoid operating at incorrect rms_current
    }
        #endif
    _driverAZ->toff(0);
        #if USE_VREF == 0  //By default, Vref is ignored when using UART to specify rms current.
    _driverAZ->I_scale_analog(false);
        #endif
    LOG(DEBUG_STEPPERS, "[MOUNT]: Requested AZ motor rms_current: %d mA", rmscurrent);
    _driverAZ->rms_current(rmscurrent, AZ_MOTOR_HOLD_SETTING / 100.0);  //holdMultiplier = 1 to set ihold = irun
    _driverAZ->toff(1);
    _driverAZ->en_spreadCycle(0);
    _driverAZ->blank_time(24);
    _driverAZ->microsteps(AZ_MICROSTEPPING == 1 ? 0 : AZ_MICROSTEPPING);  // If 1 then disable microstepping
    _driverAZ->TCOOLTHRS(0xFFFFF);                                        //xFFFFF);
    _driverAZ->semin(0);                                                  //disable CoolStep so that current is consistent
    _driverAZ->SGTHRS(stallvalue);
    if (UART_Rx_connected)
    {
        LOG(DEBUG_STEPPERS, "[MOUNT]: Actual AZ motor rms_current: %d mA", _driverAZ->rms_current());
        LOG(DEBUG_STEPPERS, "[MOUNT]: Actual AZ CS value: %d", _driverAZ->cs_actual());
        LOG(DEBUG_STEPPERS, "[MOUNT]: Actual AZ vsense: %d", _driverAZ->vsense());
    }
}
    #endif
#endif

/////////////////////////////////
//
// configureALTdriver
// TMC2209 UART only
/////////////////////////////////
#if ALT_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART
    #if SW_SERIAL_UART == 0
void Mount::configureALTdriver(Stream *serial, float rsense, byte driveraddress, int rmscurrent, int stallvalue)
{
    _driverALT = new TMC2209Stepper(serial, rsense, driveraddress);
    _driverALT->begin();
    bool UART_Rx_connected = false;
        #if UART_CONNECTION_TEST_TXRX == 1
    UART_Rx_connected = connectToDriver(_driverALT, "ALT");
    if (!UART_Rx_connected)
    {
        digitalWrite(ALT_EN_PIN,
                     HIGH);  //Disable motor for safety reasons if UART connection fails to avoid operating at incorrect rms_current
    }
        #endif
    _driverALT->toff(0);
        #if USE_VREF == 0  //By default, Vref is ignored when using UART to specify rms current.
    _driverALT->I_scale_analog(false);
        #endif
    LOG(DEBUG_STEPPERS, "[MOUNT]: Requested ALT motor rms_current: %d mA", rmscurrent);
    _driverALT->rms_current(rmscurrent, ALT_MOTOR_HOLD_SETTING / 100.0);  //holdMultiplier = 1 to set ihold = irun
    _driverALT->toff(1);
    _driverALT->en_spreadCycle(0);
    _driverALT->blank_time(24);
    _driverALT->microsteps(ALT_MICROSTEPPING == 1 ? 0 : ALT_MICROSTEPPING);  // If 1 then disable microstepping
    _driverALT->TCOOLTHRS(0xFFFFF);                                          //xFFFFF);
    _driverALT->semin(0);                                                    //disable CoolStep so that current is consistent
    _driverALT->SGTHRS(stallvalue);
    if (UART_Rx_connected)
    {
        LOG(DEBUG_STEPPERS, "[MOUNT]: Actual ALT motor rms_current: %d mA", _driverALT->rms_current());
        LOG(DEBUG_STEPPERS, "[MOUNT]: Actual ALT CS value: %d", _driverALT->cs_actual());
        LOG(DEBUG_STEPPERS, "[MOUNT]: Actual ALT vsense: %d", _driverALT->vsense());
    }
}

    #elif SW_SERIAL_UART == 1

void Mount::configureALTdriver(uint16_t ALT_SW_RX, uint16_t ALT_SW_TX, float rsense, byte driveraddress, int rmscurrent, int stallvalue)
{
    _driverALT = new TMC2209Stepper(ALT_SW_RX, ALT_SW_TX, rsense, driveraddress);
    _driverALT->beginSerial(19200);
    _driverALT->mstep_reg_select(true);
    _driverALT->pdn_disable(true);
        #if UART_CONNECTION_TEST_TXRX == 1
    bool UART_Rx_connected = false;
    UART_Rx_connected      = connectToDriver(_driverALT, "ALT");
    if (!UART_Rx_connected)
    {
        digitalWrite(ALT_EN_PIN,
                     HIGH);  //Disable motor for safety reasons if UART connection fails to avoid operating at incorrect rms_current
    }
        #endif
    _driverALT->toff(0);
        #if USE_VREF == 0  //By default, Vref is ignored when using UART to specify rms current.
    _driverALT->I_scale_analog(false);
        #endif
    LOG(DEBUG_STEPPERS, "[MOUNT]: Requested ALT motor rms_current: %d mA", rmscurrent);
    _driverALT->rms_current(rmscurrent, ALT_MOTOR_HOLD_SETTING / 100.0);  //holdMultiplier = 1 to set ihold = irun
    _driverALT->toff(1);
    _driverALT->en_spreadCycle(0);
    _driverALT->blank_time(24);
    _driverALT->microsteps(ALT_MICROSTEPPING == 1 ? 0 : ALT_MICROSTEPPING);  // If 1 then disable microstepping
    _driverALT->TCOOLTHRS(0xFFFFF);                                          //xFFFFF);
    _driverALT->semin(0);                                                    //disable CoolStep so that current is consistent
    _driverALT->SGTHRS(stallvalue);
        #if UART_CONNECTION_TEST_TXRX == 1
    if (UART_Rx_connected)
    {
        LOG(DEBUG_STEPPERS, "[MOUNT]: Actual ALT motor rms_current: %d mA", _driverALT->rms_current());
        LOG(DEBUG_STEPPERS, "[MOUNT]: Actual ALT CS value: %d", _driverALT->cs_actual());
        LOG(DEBUG_STEPPERS, "[MOUNT]: Actual ALT vsense: %d", _driverALT->vsense());
    }
        #endif
}
    #endif
#endif

/////////////////////////////////
//
// getBacklashCorrection
//
/////////////////////////////////
int Mount::getBacklashCorrection()
{
    return _backlashCorrectionSteps;
}

/////////////////////////////////
//
// getMountHardwareInfo
//
/////////////////////////////////
String Mount::getMountHardwareInfo()
{
    String ret = F("Unknown,");
#if defined(ESP32)
    ret = F("ESP32,");
#elif defined(__AVR_ATmega2560__)
    ret = F("Mega,");
#endif

    ret += F("AUTO_AZ_ALT,");

    return ret;
}

/////////////////////////////////
//
// setSpeed
//
/////////////////////////////////
void Mount::setSpeed(StepperAxis which, float speedDegsPerSec)
{
    if (which == AZIMUTH_STEPS)
    {
        float stepsPerSec = speedDegsPerSec * AZIMUTH_STEPS_PER_DEGREE;  // deg/sec * u-steps/deg = u-steps/sec
        LOG(DEBUG_STEPPERS, "[STEPPERS]: setSpeed: Set AZ speed %f degs/s, which is %f steps/s", speedDegsPerSec, stepsPerSec);
        _stepperAZ->setSpeed(stepsPerSec);
    }
    else if (which == ALTITUDE_STEPS)
    {
        float stepsPerSec = speedDegsPerSec * ALTITUDE_STEPS_PER_DEGREE;  // deg/sec * u-steps/deg = u-steps/sec
        LOG(DEBUG_STEPPERS, "[STEPPERS]: setSpeed: Set ALT speed %f degs/s, which is %f steps/s", speedDegsPerSec, stepsPerSec);
        _stepperALT->setSpeed(stepsPerSec);
    }

}

/////////////////////////////////
//
// isRunningAZ
//
/////////////////////////////////
bool Mount::isRunningAZ() const
{
    return _stepperAZ->isRunning();
}

/////////////////////////////////
//
// isRunningALT
//
/////////////////////////////////
bool Mount::isRunningALT() const
{
    return _stepperALT->isRunning();
}

/////////////////////////////////
//
// moveBy
//
/////////////////////////////////
void Mount::moveBy(int direction, float arcMinutes)
{
    if (direction == AZIMUTH_STEPS)
    {
        enableAzAltMotors();
        int stepsToMove = arcMinutes * AZIMUTH_STEPS_PER_ARC_MINUTE;
        _stepperAZ->move(stepsToMove);
    }
    if (direction == ALTITUDE_STEPS)
    {
        enableAzAltMotors();
        int stepsToMove = arcMinutes * ALTITUDE_STEPS_PER_ARC_MINUTE;
        _stepperALT->move(stepsToMove);
    }
}

/////////////////////////////////
//
// disableAzAltMotors
//
/////////////////////////////////
void Mount::disableAzAltMotors()
{
    _stepperAZ->stop();
    _stepperALT->stop();

    while (_stepperAZ->isRunning())
    {
        loop();
    }
    while (_stepperALT->isRunning())
    {
        loop();
    }

    #if AZ_ALWAYS_ON == 0
    digitalWrite(AZ_EN_PIN, HIGH);  // Logic HIGH to disable driver
    #endif

    #if ALT_ALWAYS_ON == 0
    digitalWrite(ALT_EN_PIN, HIGH);  // Logic HIGH to disable driver
    #endif
}

/////////////////////////////////
//
// enableAzAltMotors
//
/////////////////////////////////
void Mount::enableAzAltMotors()
{
    digitalWrite(AZ_EN_PIN, LOW);  // Logic LOW to enable driver
    digitalWrite(ALT_EN_PIN, LOW);  // Logic LOW to enable driver
}


/////////////////////////////////
//
// getStatusString
//
/////////////////////////////////
String Mount::getStatusString()
{
    String disp = "------,";
    if (_stepperAZ->isRunning())
        disp[3] = _stepperAZ->speed() < 0 ? 'Z' : 'z';
    if (_stepperALT->isRunning())
        disp[4] = _stepperALT->speed() < 0 ? 'A' : 'a';
    return disp;
}

/////////////////////////////////
//
// interruptLoop()
//
// This function is run in an ISR. It needs to be fast and do little work.
/////////////////////////////////
void Mount::interruptLoop()
{
    _stepperAZ->run();
    _stepperALT->run();
}

/////////////////////////////////
//
// loop
//
// Process any stepper changes.
/////////////////////////////////
void Mount::loop()
{
    bool oneIsRunning = false;
    #if (AZ_STEPPER_TYPE != STEPPER_TYPE_NONE)
    oneIsRunning |= _stepperAZ->isRunning();
    #endif
    #if (ALT_STEPPER_TYPE != STEPPER_TYPE_NONE)
    oneIsRunning |= _stepperALT->isRunning();
    #endif

    if (!oneIsRunning && _azAltWasRunning)
    {
        // One of the motors was running last time through the loop, but not anymore, so shutdown the outputs.
        disableAzAltMotors();
        _azAltWasRunning = false;
    }

    oneIsRunning = false;
    #if (AZ_STEPPER_TYPE != STEPPER_TYPE_NONE)
    oneIsRunning |= _stepperAZ->isRunning();
    #endif
    #if (ALT_STEPPER_TYPE != STEPPER_TYPE_NONE)
    oneIsRunning |= _stepperALT->isRunning();
    #endif

    if (oneIsRunning)
    {
        _azAltWasRunning = true;
    }

}

/////////////////////////////////
//
// bootComplete
//
/////////////////////////////////
void Mount::bootComplete()
{
    _bootComplete = true;
#if USE_HALL_SENSOR_RA_AUTOHOME == 1
    _homing.pinState = _homing.lastPinState = digitalRead(RA_HOMING_SENSOR_PIN);
#endif
}

/////////////////////////////////
//
// isBootComplete
//
/////////////////////////////////
bool Mount::isBootComplete()
{
    return _bootComplete;
}


/////////////////////////////////
//
// moveStepperBy
//
/////////////////////////////////
void Mount::moveStepperBy(StepperAxis direction, long steps)
{
    LOG(DEBUG_STEPPERS, "[STEPPERS]: moveStepperBy: %l", steps);
    switch (direction)
    {
        case AZIMUTH_STEPS:
            enableAzAltMotors();
            LOG(DEBUG_STEPPERS,
                "[STEPPERS]: moveStepperBy: AZ from %l to %l",
                _stepperAZ->currentPosition(),
                _stepperAZ->currentPosition() + steps);
            _stepperAZ->moveTo(_stepperAZ->currentPosition() + steps);
            break;
        case ALTITUDE_STEPS:
            enableAzAltMotors();
            _stepperALT->moveTo(_stepperALT->currentPosition() + steps);
            break;
    }
}
