#include "drv2605.h"

Drv2605::Drv2605(I2C *i2c) :p_i2c(i2c) {}

void Drv2605::init(bool lra, float period_ms) {
    char id = read_register(DRV2605_REG_STATUS);

    // Reset to powerup state
    set_mode(DRV2605_MODE_INTTRIG);

    // Set real-time playback to off
    write_register(DRV2605_REG_RTPIN, 0x00);

    // Set real-time playback to unsigned
    write_register(DRV2605_REG_CONTROL3,
                   read_register(DRV2605_REG_CONTROL3) | 0x08);

    // Strong click
    write_register(DRV2605_REG_WAVESEQ1, 1);
    write_register(DRV2605_REG_WAVESEQ2, 0);

    // no overdrive
    write_register(DRV2605_REG_OVERDRIVE, 0);

    write_register(DRV2605_REG_SUSTAINPOS, 0);
    write_register(DRV2605_REG_SUSTAINNEG, 0);
    write_register(DRV2605_REG_BREAK, 0);
    write_register(DRV2605_REG_AUDIOMAXIN, 0x64);

    if(lra) {
        setup_lra(period_ms);
    }
    else {
        setup_erm(period_ms);
    }
}

void Drv2605::set_waveform(char slot, char waveform) {
    write_register(DRV2605_REG_WAVESEQ1+slot, waveform);
}

void Drv2605::select_library(char lib) {
    write_register(DRV2605_REG_LIBRARY, lib);
}

void Drv2605::go() {
    set_mode(DRV2605_MODE_INTTRIG);
    write_register(DRV2605_REG_GO, 1);
}

void Drv2605::stop() {
    write_register(DRV2605_REG_GO, 0);
}

void Drv2605::set_mode(char mode) {
    if(m_mode != mode) {
        m_mode = mode;
        write_register(DRV2605_REG_MODE, mode);
    }
}

void Drv2605::set_realtime_value(char value) {
    if(value < 128)
        set_mode(DRV2605_MODE_INTTRIG);
    else {
        set_mode(DRV2605_MODE_REALTIME);
        write_register(DRV2605_REG_RTPIN, value);
    }
}

char Drv2605::read_register(char reg) {
    // 1 byte buffer
    char data[1];
    data[0] = reg;
    
    // Tell the chip which register we want
    p_i2c->write(DRV2605_ADDR << 1, data, 1);
    // Read the register data
    p_i2c->read(DRV2605_ADDR << 1, data, 1);

    return data[0];
}

void Drv2605::write_register(char reg, char val) {
    // 2 byte buffer
    char data[2];
    // First send the register we are writing
    data[0] = reg;
    // Then send the value to be written
    data[1] = val;

    // Send both bytes of data via i2c note that mbed takes 8bit 
    // addresses so shift the address by 1
    p_i2c->write(DRV2605_ADDR << 1, data, 2);
}

void Drv2605::setup_lra(float period_ms) {
    // Turn on LRA feedback
    write_register(DRV2605_REG_FEEDBACK,
                   read_register(DRV2605_REG_FEEDBACK) | 0x80);
    
    // Calculate the estimated DRIVE_TIME given the period of oscillation
    char drive_time = char((0.5f * period_ms - 0.5f) * 10) & 0x1F;
    write_register(DRV2605_REG_FEEDBACK,
                   read_register(DRV2605_REG_FEEDBACK) | drive_time);

    // Set LRA amplitude update to occur twice per cycle
    write_register(DRV2605_REG_CONTROL3,
                   read_register(DRV2605_REG_CONTROL3) | 0x04);

    // Set LRA to auto-resonance mode
    write_register(DRV2605_REG_CONTROL3,
                   read_register(DRV2605_REG_CONTROL3) & 0xFE);
}

void Drv2605::setup_erm(float period_ms) {
    // Turn on ERM feedback
    write_register(DRV2605_REG_FEEDBACK,
                   read_register(DRV2605_REG_FEEDBACK) & 0x7F);
    
    // Calculate required DRIVE_TIME
    char drive_time = char((period_ms - 1.0f) * 5) & 0x1F;
    write_register(DRV2605_REG_FEEDBACK,
                   read_register(DRV2605_REG_FEEDBACK) | drive_time);

    // turn on ERM open loop control
    write_register(DRV2605_REG_CONTROL3,
                   read_register(DRV2605_REG_CONTROL3) | 0x20);
}
