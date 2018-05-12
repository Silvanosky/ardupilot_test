// -*- Mode: C++; c-basic-offset: 8; indent-tabs-mode: nil -*-

#include <string.h>

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_FLYMAPLE/AP_HAL_FLYMAPLE.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void test_snprintf() {
    char test[40];
    memset(test,0,40);
    hal.util->snprintf(test, 40, "hello %d from prog %f %S\r\n",
            10, 1.2345, "progmem");
    hal.console->write((const uint8_t*)test, strlen(test));

}

void test_snprintf() {
    char test[40];
    memset(test,0,40);
    hal.util->snprintf(test, 40, "hello %d world %f %s\r\n",
            20, 2.3456, "sarg");
    hal.console->write((const uint8_t*)test, strlen(test));
}

void setup(void)
{
        //
        // HAL will start serial port at 115200.
        //

        //
        // Test printing things
        //
    hal.console->println("Utility String Library Test");
    hal.console->println("Test snprintf:");
    
    test_snprintf(); 
    
    hal.console->println("Test snprintf:");

    test_snprintf();

    hal.console->println("done");
}

void loop(void) { }

AP_HAL_MAIN();
