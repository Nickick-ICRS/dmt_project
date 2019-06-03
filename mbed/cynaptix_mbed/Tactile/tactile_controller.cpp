#include "tactile_controller.h"

TactileController::TactileController() 
        :i2c_(PB_7, PB_6), 
         multi_2_(&i2c_, MULTI2ADDR, true), 
         multi_8_(&i2c_, MULTI8ADDR), 
         lra_(&i2c_) {
    // Set each LRA to receive realtime controls
    multi_2_.select(0);
    lra_.init();
    lra_.set_mode(DRV2605_MODE_REALTIME);
    multi_2_.disable();

    for(char i = 0; i < 8; i++) {
        multi_8_.select(i);
        lra_.init();
        lra_.set_mode(DRV2605_MODE_REALTIME);
        multi_8_.disable();
    }
};

TactileController::~TactileController() {
    // dtor
};

void TactileController::set_motor_power(char motor_power, 
                                        VibrationMotor motor) {
    switch(motor) {
    // Thumb left is the '0th' motor (it's on the 2nd multiplexer)
    case TMB_L:
        multi_2_.select(0);
        break;
    default:
        // Multi 8 channel 0 is motor 1, channel 7 is motor 8...
        multi_8_.select(motor-1);
        break;
    }
    // Motor power ranges from 0 to 127, but the driver only
    // responds when the input is from 128 to 255
    lra_.set_realtime_value(motor_power+128);
    switch(motor) {
    case TMB_L:
        multi_2_.disable();
        break;
    default:
        multi_8_.disable();
        break;
    }
}
