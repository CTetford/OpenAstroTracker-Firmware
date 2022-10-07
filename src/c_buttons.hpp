#pragma once

#include "b_setup.hpp"

#if SUPPORT_SERIAL_CONTROL == 1
    #include "f_serial.hpp"
#endif

void loop()
{
    #ifdef ESP32
    serialLoop();
    #endif
}
