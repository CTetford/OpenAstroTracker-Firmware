#ifndef _MOUNT_HPP_
#define _MOUNT_HPP_

// Forward declarations
class AccelStepper;
class TMC2209Stepper;

enum StepperAxis
{
    AZIMUTH_STEPS,
    ALTITUDE_STEPS
};

//////////////////////////////////////////////////////////////////
//
// Class that represent the OpenAstroTracker mount, with all its parameters, motors, etc.
//
//////////////////////////////////////////////////////////////////
class Mount
{
  public:
    Mount();

    static Mount instance();

// Configure the AZ stepper motors.
    void configureAZStepper(byte pin1, byte pin2, int maxSpeed, int maxAcceleration);

// Configure the ALT stepper motors.
    void configureALTStepper(byte pin1, byte pin2, int maxSpeed, int maxAcceleration);


#if AZ_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART || ALT_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART
    bool connectToDriver(TMC2209Stepper *driver, const char *driverKind);
#endif

// Configure the AZ driver.
    #if AZ_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART
        #if SW_SERIAL_UART == 0
    void configureAZdriver(Stream *serial, float rsense, byte driveraddress, int rmscurrent, int stallvalue);
        #elif SW_SERIAL_UART == 1
    void configureAZdriver(uint16_t AZ_SW_RX, uint16_t AZ_SW_TX, float rsense, byte driveraddress, int rmscurrent, int stallvalue);
        #endif
    #endif

// Configure the ALT driver.
    #if ALT_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART
        #if SW_SERIAL_UART == 0
    void configureALTdriver(Stream *serial, float rsense, byte driveraddress, int rmscurrent, int stallvalue);
        #elif SW_SERIAL_UART == 1
    void configureALTdriver(uint16_t AlT_SW_RX, uint16_t ALT_SW_TX, float rsense, byte driveraddress, int rmscurrent, int stallvalue);
        #endif
    #endif
    
    bool isRunningAZ() const;
    bool isRunningALT() const;

    // Process any stepper movement.
    void loop();

    // Low-leve process any stepper movement on interrupt callback.
    void interruptLoop();

    // Returns a comma-delimited string with all the mounts' information
    String getStatusString();

    // Set the speed of the given motor
    void setSpeed(StepperAxis which, float speedDegsPerSec);

    // Support for moving the mount in azimuth and altitude (requires extra hardware)
    void moveBy(int direction, float arcMinutes);
    void disableAzAltMotors();
    void enableAzAltMotors();

    // Move the given stepper motor by the given amount of steps.
    void moveStepperBy(StepperAxis which, long steps);

    // Set the number of steps to use for backlash correction
    void setBacklashCorrection(int steps);

    // Get the number of steps to use for backlash correction
    int getBacklashCorrection();

    // Get Mount configuration data
    String getMountHardwareInfo();

    // Returns a flag indicating whether the mount is fully booted.
    bool isBootComplete();

    // Let the mount know that the system has finished booting
    void bootComplete();

  private:
    // Reads values from EEPROM that configure the mount (if previously stored)
    void readPersistentData();

    int _maxAZSpeed;
    int _maxALTSpeed;
    int _maxAZAcceleration;
    int _maxALTAcceleration;
    int _backlashCorrectionSteps;
    // Stepper control for RA, DEC and TRK.

    bool _azAltWasRunning;
    AccelStepper *_stepperAZ;
        #if AZ_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART
    TMC2209Stepper *_driverAZ;
        #endif
    AccelStepper *_stepperALT;
        #if ALT_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART
    TMC2209Stepper *_driverALT;
        #endif

    bool _bootComplete;

    long _localStartTimeSetMillis;
};

#endif