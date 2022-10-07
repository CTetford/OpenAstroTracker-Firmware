// Create the LCD menu variable and initialize the LCD (16x2 characters)

#pragma once

#if defined(OAT_DEBUG_BUILD)
PUSH_NO_WARNINGS
    #if BOARD < 1000
        #include "avr8-stub.h"
    #else
        #error "Debugging not supported on this platform"
    #endif
POP_NO_WARNINGS
#endif

#include "InterruptCallback.hpp"

#include "Utility.hpp"
#include "a_inits.hpp"


#ifdef ESP32
DRAM_ATTR Mount mount(&);
#else
Mount mount;
#endif

#if (WIFI_ENABLED == 1)
    #include "WifiControl.hpp"
WifiControl wifiControl(&mount);
#endif

/////////////////////////////////
//   Interrupt handling
/////////////////////////////////
/* There are Two possible configurations for periodically servicing the stepper drives:
 * 1) If ESP32 is #defined then a periodic task is assigned to Core 0 to service the steppers.
 *    stepperControlTask() is scheduled to run every 1 ms (1 kHz rate). On ESP32 the default Arduino 
 *    loop() function runs on Core 1, therefore serial and UI activity also runs on Core 1. 
 *    Note that Wifi drivers will be sharing Core 0 with stepperControlTask().
 *    This configuration decouples stepper servicing from other OAT activities by using both cores.
 * 2) By default (e.g. for ATmega2560) a periodic timer is configured for a 500 us (2 kHz rate interval).
 *    This timr generates interrupts which are handled by stepperControlCallback(). The stepper 
 *    servicing therefore suspends loop() to generate motion, ensuring smooth tracking.
 */
#if defined(ESP32)

TaskHandle_t StepperTask;

// This is the task for simulating periodic interrupts on ESP32 platforms.
// It should do very minimal work, only calling Mount::interruptLoop() to step the stepper motors as needed.
// This task function is run on Core 0 of the ESP32 and never returns
void IRAM_ATTR stepperControlTask(void *payload)
{
    Mount *mountCopy = reinterpret_cast<Mount *>(payload);
    for (;;)
    {
        mountCopy->interruptLoop();
        vTaskDelay(1);  // 1 ms 	// This will limit max stepping rate to 1 kHz
    }
}

#else
// This is the callback function for the timer interrupt on ATMega platforms.
// It should do very minimal work, only calling Mount::interruptLoop() to step the stepper motors as needed.
// It is called every 500 us (2 kHz rate)
void stepperControlTimerCallback(void *payload)
{
    Mount *mountCopy = reinterpret_cast<Mount *>(payload);
    if (mountCopy)
        mountCopy->interruptLoop();
}

#endif

/////////////////////////////////
//
// Main program setup
//
/////////////////////////////////
void setup()
{
#if defined(OAT_DEBUG_BUILD)
    #if BOARD < 1000
    debug_init();  // Setup avr-stub
    breakpoint();  // Set a breakpoint as soon as possible
    #else
        #error "Debugging not supported on this platform"
    #endif
#else
    Serial.begin(SERIAL_BAUDRATE);
    #if DEBUG_LEVEL > 0 && DEBUG_SEPARATE_SERIAL == 1
    DEBUG_SERIAL_PORT.begin(DEBUG_SERIAL_BAUDRATE);
    #endif
#endif

    LOG(DEBUG_ANY, "[SYSTEM]: Hello, universe, this is OAT %s!", VERSION);

    /////////////////////////////////
    //   Microstepping/driver pins
    /////////////////////////////////
    pinMode(AZ_EN_PIN, OUTPUT);
    digitalWrite(AZ_EN_PIN, HIGH);  // Logic HIGH to disable the driver initally
    #if AZ_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART
    // include TMC2209 UART pins
    pinMode(AZ_DIAG_PIN, INPUT);
        #ifdef AZ_SERIAL_PORT
    AZ_SERIAL_PORT.begin(57600);  // Start HardwareSerial comms with driver
        #endif
    #endif

    pinMode(ALT_EN_PIN, OUTPUT);
    digitalWrite(ALT_EN_PIN, HIGH);  // Logic HIGH to disable the driver initally
    #if ALT_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART
    // include TMC2209 UART pins
    pinMode(ALT_DIAG_PIN, INPUT);
        #ifdef ALT_SERIAL_PORT
    ALT_SERIAL_PORT.begin(57600);  // Start HardwareSerial comms with driver
        #endif
    #endif
    // end microstepping -------------------

    LOG(DEBUG_ANY, "[SYSTEM]: Hardware: %s", mount.getMountHardwareInfo().c_str());

    // Create the command processor singleton
    LOG(DEBUG_ANY, "[SYSTEM]: Initialize LX200 handler...");
    MeadeCommandProcessor::createProcessor(&mount);

#if (WIFI_ENABLED == 1)
    LOG(DEBUG_ANY, "[SYSTEM]: Setup Wifi...");
    wifiControl.setup();
#endif

    // Configure the mount
    // Delay for a while to get UARTs booted...
    delay(1000);

    LOG(DEBUG_ANY, "[SYSTEM]: Configure steppers...");

// Set the stepper motor parameters

    LOG(DEBUG_ANY, "[STEPPERS]: Configure AZ stepper...");
    mount.configureAZStepper(AZmotorPin1, AZmotorPin2, AZ_STEPPER_SPEED, AZ_STEPPER_ACCELERATION);
    #if AZ_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART
    LOG(DEBUG_ANY, "[STEPPERS]: Configure AZ driver...");
        #if SW_SERIAL_UART == 0
    mount.configureAZdriver(&AZ_SERIAL_PORT, R_SENSE, AZ_DRIVER_ADDRESS, AZ_RMSCURRENT, AZ_STALL_VALUE);
        #elif SW_SERIAL_UART == 1
    mount.configureAZdriver(AZ_SERIAL_PORT_RX, AZ_SERIAL_PORT_TX, R_SENSE, AZ_DRIVER_ADDRESS, AZ_RMSCURRENT, AZ_STALL_VALUE);
        #endif
    #endif

    LOG(DEBUG_ANY, "[STEPPERS]: Configure Alt stepper...");
    mount.configureALTStepper(ALTmotorPin1, ALTmotorPin2, ALT_STEPPER_SPEED, ALT_STEPPER_ACCELERATION);
    #if ALT_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART
    LOG(DEBUG_ANY, "[STEPPERS]: Configure ALT driver...");
        #if SW_SERIAL_UART == 0
    mount.configureALTdriver(&ALT_SERIAL_PORT, R_SENSE, ALT_DRIVER_ADDRESS, ALT_RMSCURRENT, ALT_STALL_VALUE);
        #elif SW_SERIAL_UART == 1
    mount.configureALTdriver(ALT_SERIAL_PORT_RX, ALT_SERIAL_PORT_TX, R_SENSE, ALT_DRIVER_ADDRESS, ALT_RMSCURRENT, ALT_STALL_VALUE);
        #endif
    #endif

// Setup service to periodically service the steppers.
#if defined(ESP32)

    disableCore0WDT();
    xTaskCreatePinnedToCore(stepperControlTask,  // Function to run on this core
                            "StepperControl",    // Name of this task
                            32767,               // Stack space in bytes
                            &mount,              // payload
                            2,                   // Priority (2 is higher than 1)
                            &StepperTask,        // The location that receives the thread id
                            0);                  // The core to run this on

#else
    // 2 kHz updates (higher frequency interferes with serial communications and complete messes up OATControl communications)
    if (!InterruptCallback::setInterval(0.5f, stepperControlTimerCallback, &mount))
    {
        LOG(DEBUG_MOUNT, "[SYSTEM]: CANNOT setup interrupt timer!");
    }
#endif

    mount.bootComplete();
    LOG(DEBUG_ANY, "[SYSTEM]: Boot complete!");
}
