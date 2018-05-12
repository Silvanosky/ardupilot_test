
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_FLYMAPLE/AP_HAL_FLYMAPLE.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void multiread(AP_HAL::RCInput* in, uint16_t* channels) {
    /* Multi-channel read method: */
    uint8_t valid;
    valid = in->read(channels, 8);
    hal.console->printf(
            "multi      read %d: %d %d %d %d %d %d %d %d\r\n",
            (int) valid, 
            channels[0], channels[1], channels[2], channels[3],
            channels[4], channels[5], channels[6], channels[7]);
}

void individualread(AP_HAL::RCInput* in, uint16_t* channels) {
    /* individual channel read method: */
    bool valid;
    valid = in->new_input();
    for (int i = 0; i < 8; i++) {
        channels[i] = in->read(i);
    }
    hal.console->printf(
            "individual read %d: %d %d %d %d %d %d %d %d\r\n",
            (int) valid, 
            channels[0], channels[1], channels[2], channels[3],
            channels[4], channels[5], channels[6], channels[7]);
}


void individualwrite(AP_HAL::RCOutput* out, uint16_t* channels) {
    for (int ch = 0; ch < 8; ch++) {
        out->write(ch, channels[ch]); 
        /* Upper channels duplicate lower channels*/
        out->write(ch+8, channels[ch]);
    }
}

void loop (void) {
    static int ctr = 0;
    uint16_t channels[8];

//    hal.gpio->write(13, 1);

    /* Cycle between using the individual read method
     * and the multi read method*/
    if (ctr < 500) {
        multiread(hal.rcin, channels);
    } else {
        individualread(hal.rcin, channels);
        if (ctr > 1000)  ctr = 0;
    }

    individualwrite(hal.rcout, channels);

//    hal.gpio->write(13, 0);
    hal.scheduler->delay(4);
    ctr++;
}

void setup (void) {
//    hal.scheduler->delay(5000);
    hal.gpio->pinMode(13, HAL_GPIO_OUTPUT);
    hal.gpio->write(13, 0);
    for (uint8_t i=0; i<16; i++) {
        hal.rcout->enable_ch(i);
    }

    /* Bottom 4 channels at 400hz (like on a quad) */
    hal.rcout->set_freq(0x0000000F, 400);
    for(int i = 0; i < 12; i++) {
        hal.console->printf("rcout ch %d has frequency %d\r\n",
                i, hal.rcout->get_freq(i));
    }
    /* Delay to let the user see the above printouts on the terminal */
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
