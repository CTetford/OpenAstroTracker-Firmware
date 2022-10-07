#pragma once

/**
 * This file contains advanced configurations. Edit values here only if you know what you are doing. Invalid values
 * can lead to OAT misbehaving very bad and in worst case could even lead to hardware damage. The default values here
 * were chosen after many tests and can are currently concidered to work the best.
 * 
 * 
 *         YOU SHOULD NOT NEED TO EDIT THIS FILE!
 *         --------------------------------------
 **/

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                            ////////
// Misc Configuration         ////////
//                            ////////
//////////////////////////////////////
#if !defined(USE_DUMMY_EEPROM)
    #define USE_DUMMY_EEPROM false
#endif
#if !defined(BUFFER_LOGS)
    #define BUFFER_LOGS false
#endif


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                        ////////
// MECHANICS SETTINGS     ////////
//                        ////////
//////////////////////////////////
//

// the pitch of the GT2 timing belt
#define GT2_BELT_PITCH 2.0f  // mm

////////////////////////////
//
// INVERT AXIS
// Set to 1 or 0 to invert motor directions
#ifndef AZ_INVERT_DIR
    #define AZ_INVERT_DIR 0
#endif
#ifndef ALT_INVERT_DIR
    #define ALT_INVERT_DIR 0
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     ///
// HARDWARE EXTENSIONS SUPPORT SECTION ///
//                                     ///
//////////////////////////////////////////

//////////////////////////////////////////
// AutoPA AZIMUTH support
//////////////////////////////////////////
// Enable Azimuth motor functionality in your local Configuration. Do not edit here!
#ifndef AZ_MICROSTEPPING
    #define AZ_MICROSTEPPING 64.0f
#endif
#ifndef AZ_STEPPER_SPR
    #define AZ_STEPPER_SPR 400  // NEMA 0.9° = 400  |  NEMA 1.8° = 200
#endif
#ifndef AZ_STEPPER_SPEED
    #define AZ_STEPPER_SPEED                                                                                                           \
        100 * AZ_MICROSTEPPING  // You can change the speed and acceleration of the steppers here. Max. Speed = 3000.
#endif
#ifndef AZ_STEPPER_ACCELERATION
    #define AZ_STEPPER_ACCELERATION 100 * AZ_MICROSTEPPING
#endif

// the Circumference of the AZ rotation. 808mm dia.
#define AZ_CIRCUMFERENCE 2538.4f
#ifndef AZIMUTH_STEPS_PER_REV
    #define AZIMUTH_STEPS_PER_REV                                                                                                      \
        (AZ_CORRECTION_FACTOR * (AZ_CIRCUMFERENCE / (AZ_PULLEY_TEETH * GT2_BELT_PITCH)) * AZ_STEPPER_SPR                               \
            * AZ_MICROSTEPPING)  // Actually u-steps/rev
#endif
#define AZIMUTH_STEPS_PER_DEGREE (AZIMUTH_STEPS_PER_REV / (360.0f))
#define AZIMUTH_STEPS_PER_ARC_MINUTE (AZIMUTH_STEPS_PER_REV / (60.0f))  // Used to determine move distance in steps


// AZ TMC2209 UART settings
// These settings work only with TMC2209 in UART connection (single wire to TX)
#if (AZ_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART)
    #ifndef USE_VREF
        #define USE_VREF 0  //By default Vref is ignored when using UART to specify rms current. Only enable if you know what you are doing.
    #endif

    #define AZ_RMSCURRENT AZ_MOTOR_CURRENT_RATING *(AZ_OPERATING_CURRENT_SETTING / 100.0f) / 1.414f

    #ifndef AZ_MOTOR_HOLD_SETTING
        #define AZ_MOTOR_HOLD_SETTING 100
    #endif

    #define AZ_STALL_VALUE 10  // adjust this value if the RA autohoming sequence often false triggers, or triggers too late

    #ifndef USE_VREF
        #define USE_VREF                                                                                                               \
            0  //By default Vref is ignored when using UART to specify rms current. Only enable if you know what you are doing.
    #endif
#endif
#ifndef AZ_ALWAYS_ON
    #define AZ_ALWAYS_ON 0
#endif

//////////////////////////////////////////
// AutoPA ALTITUDE support
//////////////////////////////////////////
// Enable Altitude motor functionality in your local configuration. Do not edit here!
#ifndef ALT_MICROSTEPPING
    #define ALT_MICROSTEPPING 4.0f
#endif
#ifndef ALT_STEPPER_SPR
    #define ALT_STEPPER_SPR 400  // NEMA 0.9° = 400  |  NEMA 1.8° = 200
#endif
#ifndef ALT_STEPPER_SPEED
    #define ALT_STEPPER_SPEED                                                                                                          \
        3000 * ALT_MICROSTEPPING  // You can change the speed and acceleration of the steppers here. Max. Speed = 3000.
#endif
#ifndef ALT_STEPPER_ACCELERATION
    #define ALT_STEPPER_ACCELERATION 100 * ALT_MICROSTEPPING
#endif

// the Circumference of the AZ rotation. 770mm dia.
#define ALT_CIRCUMFERENCE 2419.0f

#ifndef AUTOPA_VERSION
    #define AUTOPA_VERSION 1
#endif

#if AUTOPA_VERSION == 1
    // the ratio of the ALT gearbox for AutoPA V1 (40:3)
    #define ALT_WORMGEAR_RATIO (40.0f / 3.0f)
#else
    // the ratio of the ALT gearbox for AutoPA V2 (40:1)
    #define ALT_WORMGEAR_RATIO (40.0f)
#endif

#define ALTITUDE_STEPS_PER_REV                                                                                                         \
    (ALT_CORRECTION_FACTOR * (ALT_CIRCUMFERENCE / (ALT_PULLEY_TEETH * GT2_BELT_PITCH)) * ALT_STEPPER_SPR * ALT_MICROSTEPPING           \
        * ALT_WORMGEAR_RATIO)                                                      // Actually u-steps/rev
#define ALTITUDE_STEPS_PER_DEGREE (ALTITUDE_STEPS_PER_REV / (360.0f))
#define ALTITUDE_STEPS_PER_ARC_MINUTE (ALTITUDE_STEPS_PER_REV / (60.0f))  // Used to determine move distance in steps

// ALT TMC2209 UART settings
// These settings work only with TMC2209 in UART connection (single wire to TX)
#if (ALT_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART)
    #define ALT_RMSCURRENT ALT_MOTOR_CURRENT_RATING *(ALT_OPERATING_CURRENT_SETTING / 100.0f) / 1.414f

    #ifndef ALT_MOTOR_HOLD_SETTING
        #define ALT_MOTOR_HOLD_SETTING 100
    #endif

    #define ALT_STALL_VALUE 10  // adjust this value if the RA autohoming sequence often false triggers, or triggers too late

    #ifndef USE_VREF
        #define USE_VREF                                                                                                               \
            0  //By default Vref is ignored when using UART to specify rms current. Only enable if you know what you are doing.
    #endif
#endif
#ifndef ALT_ALWAYS_ON
    #define ALT_ALWAYS_ON 0
#endif

// Enable Meade protocol communication over serial
#if !defined(SUPPORT_SERIAL_CONTROL)
    #define SUPPORT_SERIAL_CONTROL 1
#endif

// The port number to access OAT control over WiFi (ESP32 only)
#define WIFI_PORT 4030

////////////////////////////
//
// DEBUG OUTPUT
//
// #define DEBUG_LEVEL (DEBUG_STEPPERS|DEBUG_MOUNT)
// #define DEBUG_LEVEL (DEBUG_INFO|DEBUG_MOUNT|DEBUG_GENERAL)
// #define DEBUG_LEVEL (DEBUG_SERIAL|DEBUG_WIFI|DEBUG_INFO|DEBUG_MOUNT|DEBUG_GENERAL)
// #define DEBUG_LEVEL (DEBUG_ANY)
// #define DEBUG_LEVEL (DEBUG_INFO|DEBUG_MOUNT|DEBUG_GENERAL)
//
// Bit Name                 Output
//  0  DEBUG_INFO           General output, like startup variables and status
//  1  DEBUG_SERIAL         Serial commands and replies
//  2  DEBUG_WIFI           Wifi related output
//  3  DEBUG_MOUNT          Mount processing output
//  4  DEBUG_MOUNT_VERBOSE  Verbose mount processing (coordinates, etc)
//  5  DEBUG_GENERAL        Other misc. output
//  6  DEBUG_MEADE          Meade command handling output
// Set this to specify the amount of debug output OAT should send to the serial port.
// Note that if you use an app to control OAT, ANY debug output will likely confuse that app.
// Debug output is useful if you are using Wifi to control the OAT or if you are issuing
// manual commands via a terminal only.
//
#ifndef DEBUG_LEVEL
    #define DEBUG_LEVEL (DEBUG_NONE)
#endif

#ifndef DEBUG_SEPARATE_SERIAL
    #define DEBUG_SEPARATE_SERIAL 0
#endif

#ifndef DEBUG_SERIAL_BAUDRATE
    #define DEBUG_SERIAL_BAUDRATE 115200
#endif

#if defined(OAT_DEBUG_BUILD)
    // AVR based boards have numbers < 1000
    #if BOARD < 1000
        /*
     * Debugging on the mega2560 using avr-stub dissallows application-code from
     * using the normal (USB) serial port
     */
        // Disable debug output
        #if defined(DEBUG_LEVEL)
            #undef DEBUG_LEVEL
        #endif
        #define DEBUG_LEVEL (DEBUG_NONE)

        // Disable serial control
        #if defined(SUPPORT_SERIAL_CONTROL)
            #undef SUPPORT_SERIAL_CONTROL
        #endif
        #define SUPPORT_SERIAL_CONTROL 0
    #else
        #error "Debugging not supported on this platform"
    #endif
#endif
