#pragma once

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                            ////////
// VALIDATE CONFIGURATION     ////////
//                            ////////
//////////////////////////////////////

// Platform
#if defined(ESP32) || defined(__AVR_ATmega2560__)
// Valid platform
#else
    #error Unsupported platform configuration. Use at own risk.
#endif

#if (AZ_STEPPER_TYPE == STEPPER_TYPE_NONE)
    // Baseline configuration without azimuth control is valid
    #if (AZ_DRIVER_TYPE == DRIVER_TYPE_NONE)
    // Valid ALT stepper and driver combination
    #else
        #error Defined an AZ driver, but no AZ stepper.
    #endif
#elif defined(__AVR_ATmega2560__)
    // Azimuth configuration
    #if (AZ_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART)
        #ifndef AZ_DRIVER_ADDRESS
            // Serial bus address must be specified for TMC2209 in UART mode
            #error AZ driver address for DRIVER_TYPE_TMC2209_UART not specified.
        #endif
    #endif

#else
    #error Configuration does not support AZ. Use at own risk.
#endif

#if (ALT_STEPPER_TYPE == STEPPER_TYPE_NONE)
    // Baseline configuration without altitude control is valid
    #if (ALT_DRIVER_TYPE == DRIVER_TYPE_NONE)
    // Valid ALT stepper and driver combination
    #else
        #error Defined an ALT driver, but no ALT stepper.
    #endif
#elif defined(__AVR_ATmega2560__)
    // Altitude configuration
    #if (ALT_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART)
        #ifndef ALT_DRIVER_ADDRESS
            // Serial bus address must be specified for TMC2209 in UART mode
            #error ALT driver address for DRIVER_TYPE_TMC2209_UART not specified.
        #endif
    #endif

#else
    #warning Configuration does not support ALT. Use at own risk.
#endif

#if (WIFI_ENABLED == 0)
// Baseline configuration without WiFi is valid
#elif defined(ESP32)
    // Wifi is only supported on ESP32
    #if !defined(WIFI_HOSTNAME)
        #error Wifi hostname must be provided for infrastructure and AP modes
    #endif
    #if (WIFI_MODE == WIFI_MODE_DISABLED)
    // Baseline configuration with disabled WiFi is valid
    #endif
    #if (WIFI_MODE == WIFI_MODE_INFRASTRUCTURE) || (WIFI_MODE == WIFI_MODE_ATTEMPT_INFRASTRUCTURE_FAIL_TO_AP)
        #if !defined(WIFI_INFRASTRUCTURE_MODE_SSID) || !defined(WIFI_INFRASTRUCTURE_MODE_WPAKEY)
            #error Wifi SSID and WPA key must be provided for infrastructure mode
        #endif
    #elif (WIFI_MODE == WIFI_MODE_AP_ONLY) || (WIFI_MODE == WIFI_MODE_ATTEMPT_INFRASTRUCTURE_FAIL_TO_AP)
        #if !defined(WIFI_AP_MODE_WPAKEY)
            #error Wifi WPA key must be provided for AP mode
        #endif
    #else
        #error Unsupported WiFi configuration. Use at own risk.
    #endif
#else
    #error Unsupported WiFi configuration (WiFI only supported on ESP32). Use at own risk.
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                            ////////
// VALIDATE PIN ASSIGNMENTS   ////////
//                            ////////
//////////////////////////////////////

// Motor & driver configurations
#if (AZ_STEPPER_TYPE != STEPPER_TYPE_NONE)
    #if (AZ_DRIVER_TYPE == DRIVER_TYPE_A4988_GENERIC) || (AZ_DRIVER_TYPE == DRIVER_TYPE_TMC2209_STANDALONE)
        #if !defined(AZ_STEP_PIN) || !defined(AZ_DIR_PIN) || !defined(AZ_EN_PIN) || !defined(AZ_DIAG_PIN)
            // Required pin assignments missing
            #error Missing pin assignments for configured AZ DRIVER_TYPE_A4988_GENERIC or DRIVER_TYPE_TMC2209_STANDALONE driver
        #endif
    #elif (AZ_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART)
        #if !defined(AZ_STEP_PIN) || !defined(AZ_DIR_PIN) || !defined(AZ_EN_PIN) || !defined(AZ_DIAG_PIN)
            // Required pin assignments missing (ATmega uses SoftwareSerial for this driver)
            #error Missing pin assignments for configured AZ DRIVER_TYPE_TMC2209_UART driver
        #endif
        #if !((defined(AZ_SERIAL_PORT_TX) && defined(AZ_SERIAL_PORT_RX)) || defined(AZ_SERIAL_PORT))
            // Required pin assignments missing for UART serial
            #error Missing pin assignments for configured AZ DRIVER_TYPE_TMC2209_UART driver serial connection
        #endif
    #endif
#endif

#if (ALT_STEPPER_TYPE != STEPPER_TYPE_NONE)
    #if (ALT_DRIVER_TYPE == DRIVER_TYPE_A4988_GENERIC) || (ALT_DRIVER_TYPE == DRIVER_TYPE_TMC2209_STANDALONE)
        #if !defined(ALT_STEP_PIN) || !defined(ALT_DIR_PIN) || !defined(ALT_EN_PIN) || !defined(ALT_DIAG_PIN)
            // Required pin assignments missing
            #error Missing pin assignments for configured AZ DRIVER_TYPE_A4988_GENERIC or DRIVER_TYPE_TMC2209_STANDALONE driver
        #endif
    #elif (ALT_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART)
        #if !defined(ALT_STEP_PIN) || !defined(ALT_DIR_PIN) || !defined(ALT_EN_PIN) || !defined(ALT_DIAG_PIN)
            // Required pin assignments missing (ATmega uses SoftwareSerial for this driver)
            #error Missing pin assignments for configured ALT DRIVER_TYPE_TMC2209_UART driver
        #endif
        #if !((defined(ALT_SERIAL_PORT_TX) && defined(ALT_SERIAL_PORT_RX)) || defined(ALT_SERIAL_PORT))
            // Required pin assignments missing for UART serial
            #error Missing pin assignments for configured ALT DRIVER_TYPE_TMC2209_UART driver serial connection
        #endif
    #endif
#endif

// Debugging
#if DEBUG_SEPARATE_SERIAL == 1
    #if !defined(DEBUG_SERIAL_PORT)
        #error Missing serial port assignment for external debugging
    #endif
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                ////////
// VALIDATE CRITICAL PARAMETERS   ////////
//                                ////////
//////////////////////////////////////////

#if (AZ_STEPPER_TYPE != STEPPER_TYPE_NONE)
    #if !defined(AZ_MICROSTEPPING)
        #error "Azimuth microstepping must be defined. Default is 4.0f for 28BYJ steppers, 64.0f for NEMA."
    #endif
    #if (AZ_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART)
        #if defined(AZ_MOTOR_CURRENT_RATING)
            #if (AZ_MOTOR_CURRENT_RATING > 2000)
                #error "The TMC2209 driver is only rated up to 2A output. Delete this error if you know what you're doing"
            #endif
            #if (AZ_MOTOR_CURRENT_RATING == 0)
                #error                                                                                                                     \
                    "AZ current rating/setting cannot be zero. Please configure the current rating of your motor in you local configuration file using the AZ_MOTOR_CURRENT_RATING keyword."
            #endif
        #else
            #error                                                                                                                         \
                "AZ_MOTOR_CURRENT_RATING is not defined. Please define the current rating of your motor in you local configuration file using the AZ_MOTOR_CURRENT_RATING keyword."
        #endif
        #if defined(AZ_OPERATING_CURRENT_SETTING)
            #if (AZ_OPERATING_CURRENT_SETTING <= 0) || (AZ_OPERATING_CURRENT_SETTING > 100)
                #error "AZ_OPERATING_CURRENT_SETTING is not within acceptable range (0-100)"
            #endif
        #else
            #error                                                                                                                         \
                "AZ_OPERATING_CURRENT_SETTING is not defined. Please define the operating percentage of your motor in you local configuration file using the AZ_OPERATING_CURRENT_SETTING keyword."
        #endif
    #endif
#endif

#if (ALT_STEPPER_TYPE != STEPPER_TYPE_NONE)
    #if !defined(ALT_MICROSTEPPING)
        #error "Altitude microstepping must be defined. Default is 4.0f for 28BYJ steppers, 4.0f for NEMA."
    #endif
    #if (ALT_DRIVER_TYPE == DRIVER_TYPE_TMC2209_UART)
        #if defined(ALT_MOTOR_CURRENT_RATING)
            #if (ALT_MOTOR_CURRENT_RATING > 2000)
                #error "The TMC2209 driver is only rated up to 2A output. Delete this error if you know what you're doing"
            #endif
            #if (ALT_MOTOR_CURRENT_RATING == 0)
                #error                                                                                                                     \
                    "ALT current rating/setting cannot be zero. Please configure the current rating of your motor in you local configuration file using the ALT_MOTOR_CURRENT_RATING keyword."
            #endif
        #else
            #error                                                                                                                         \
                "ALT_MOTOR_CURRENT_RATING is not defined. Please define the current rating of your motor in you local configuration file using the ALT_MOTOR_CURRENT_RATING keyword."
        #endif
        #if defined(ALT_OPERATING_CURRENT_SETTING)
            #if (ALT_OPERATING_CURRENT_SETTING <= 0) || (ALT_OPERATING_CURRENT_SETTING > 100)
                #error "ALT_OPERATING_CURRENT_SETTING is not within acceptable range (0-100)"
            #endif
        #else
            #error                                                                                                                         \
                "ALT_OPERATING_CURRENT_SETTING is not defined. Please define the operating percentage of your motor in you local configuration file using the ALT_OPERATING_CURRENT_SETTING keyword."
        #endif
    #endif
#endif
