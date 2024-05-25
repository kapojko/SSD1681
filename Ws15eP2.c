#include "MinUnit.h"
#include "Ws15eP2.h"

#define WS15EP2_CMD_DRIVER_OUTPUT 0x01
#define WS15EP2_CMD_GATE_DRIVING_VOLTAGE 0x03
#define WS15EP2_CMD_SOURCE_DRIVING_VOLTAGE 0x04
#define WS15EP2_CMD_DEEP_SLEEP_MODE 0x10
#define WS15EP2_CMD_DATA_ENTRY_MODE 0x11
#define WS15EP2_CMD_SW_RESET 0x12
#define WS15EP2_CMD_MASTER_ACTIVATION 0x20
#define WS15EP2_CMD_DISPLAY_UPDATE_CONTROL_1 0x21
#define WS15EP2_CMD_DISPLAY_UPDATE_CONTROL_2 0x22
#define WS15EP2_CMD_WRITE_RAM_BW 0x24
#define WS15EP2_CMD_WRITE_RAM_RED 0x26
#define WS15EP2_CMD_VCOM_SENSE 0x28
#define WS15EP2_CMD_PROGRAM_VCOM_OTP 0x2A
#define WS15EP2_CMD_VCOM_CONTROL 0x2B
#define WS15EP2_CMD_VCOM_WRITE 0x2C
#define WS15EP2_CMD_READ_DISPLAY_OPTION_OTP 0x2D
#define WS15EP2_CMD_READ_USER_ID 0x2E
#define WS15EP2_CMD_PROGRAM_WS_OTP 0x30
#define WS15EP2_CMD_LOAD_WS_OTP 0x31
#define WS15EP2_CMD_WRITE_LUT 0x32
#define WS15EP2_CMD_PROGRAM_OTP_SELECTION 0x36
#define WS15EP2_CMD_WRITE_USER_ID 0x38
#define WS15EP2_CMD_OTP_PROGRAM_MODE 0x39
#define WS15EP2_CMD_SET_RAM_X 0x44
#define WS15EP2_CMD_SET_RAM_Y 0x45
#define WS15EP2_CMD_SET_RAM_X_COUNTER 0x4E
#define WS15EP2_CMD_SET_RAM_Y_COUNTER 0x4F

static struct Ws15eP2_Platform *platform;

// see https://github.com/waveshareteam/e-Paper/blob/master/Arduino/epd1in54_V2/epd1in54_V2.cpp
// the same values also in https://github.com/waveshareteam/e-Paper/blob/master/STM32/STM32-F103ZET6/User/e-Paper/EPD_1in54_V2.c
// it could be not that since the voltage config is in last bytes
uint8_t Ws15eP2_WF_Full_1IN54[159] = {
    0x80,	0x48,	0x40,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
    0x40,	0x48,	0x80,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
    0x80,	0x48,	0x40,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
    0x40,	0x48,	0x80,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
    0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
    0xA,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
    0x8,	0x1,	0x0,	0x8,	0x1,	0x0,	0x2,
    0xA,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
    0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
    0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
    0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
    0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
    0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
    0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
    0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
    0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
    0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
    0x22,	0x22,	0x22,	0x22,	0x22,	0x22,	0x0,	0x0,	0x0,
    0x22,	0x17,	0x41,	0x0,	0x32,	0x20
};

// waveform partial refresh(fast)
// see https://github.com/waveshareteam/e-Paper/blob/master/Arduino/epd1in54_V2/epd1in54_V2.cpp
// the same values also in https://github.com/waveshareteam/e-Paper/blob/master/STM32/STM32-F103ZET6/User/e-Paper/EPD_1in54_V2.c
// it could be not that since the voltage config is in last bytes
uint8_t Ws15eP2_WF_PARTIAL_1IN54_0[159] = {
    0x0,0x40,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
    0x80,0x80,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
    0x40,0x40,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
    0x0,0x80,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
    0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
    0xF,0x0,0x0,0x0,0x0,0x0,0x0,
    0x1,0x1,0x0,0x0,0x0,0x0,0x0,
    0x0,0x0,0x0,0x0,0x0,0x0,0x0,
    0x0,0x0,0x0,0x0,0x0,0x0,0x0,
    0x0,0x0,0x0,0x0,0x0,0x0,0x0,
    0x0,0x0,0x0,0x0,0x0,0x0,0x0,
    0x0,0x0,0x0,0x0,0x0,0x0,0x0,
    0x0,0x0,0x0,0x0,0x0,0x0,0x0,
    0x0,0x0,0x0,0x0,0x0,0x0,0x0,
    0x0,0x0,0x0,0x0,0x0,0x0,0x0,
    0x0,0x0,0x0,0x0,0x0,0x0,0x0,
    0x0,0x0,0x0,0x0,0x0,0x0,0x0,
    0x22,0x22,0x22,0x22,0x22,0x22,0x0,0x0,0x0,
    0x02,0x17,0x41,0xB0,0x32,0x28,
};

static bool sendCommand(uint8_t command) {
    int res;

    platform->gpioSet(platform->pinDc, 0);

    platform->gpioSet(platform->pinCs, 0);
    res = platform->spiSend(&command, 1);
    platform->gpioSet(platform->pinCs, 1);

    if (res != 0) {
        platform->debugPrint("sendCommand failed, spiSend returned %d\r\n", res);
        return false;
    }

    return true;
}

static bool sendData(const uint8_t *data, int len) {
    int res;

    platform->gpioSet(platform->pinDc, 1);

    platform->gpioSet(platform->pinCs, 0);
    res = platform->spiSend(data, len);
    platform->gpioSet(platform->pinCs, 1);

    if (res != 0) {
        platform->debugPrint("sendData failed, spiSend returned %d\r\n", res);
        return false;
    }

    return true;
}

static uint16_t convertTemperature(int tempDegC) {
    // Apply LSB
    int16_t tempSigned = tempDegC * 16;

    // Convert to 2's supplement (12 bits only)
    uint16_t temp = (*(uint16_t *)&tempSigned) & 0xFFF;

    return temp;
}

void Ws15eP2_Init(struct Ws15eP2_Platform *platformPtr) {
    platform = platformPtr;
}

bool Ws15eP2_DefInit(bool reversed) {
    bool ok = true;

    // Perform hardware reset
    Ws15eP2_HardwareReset();

    // Software reset
    ok = ok & Ws15eP2_SoftwareReset();

    // Driver output control
    ok = ok & Ws15eP2_SetDriverOutput(WS15EP2_MUX_GATE_LINES_DEF, WS15EP2_GATE_DEF, WS15EP2_SCAN_DEF,
        (reversed ? WS15EP2_TOP_BOTTOM_REVERSE : WS15EP2_TOP_BOTTOM));

    // Data entry mode
    ok = ok & Ws15eP2_SetDataEntryMode(reversed ? WS15EP2_DATA_ENTRY_YDEC_XINC : WS15EP2_DATA_ENTRY_YINC_XINC,
        WS15EP2_DATA_DIRECTION_X);
}

bool Ws15eP2_SoftwareReset(void) {
    bool ok = true;

    ok = ok & sendCommand(WS15EP2_CMD_SW_RESET);
    Ws15eP2_WaitBusy();

    return ok;
}

bool Ws15eP2_SetDriverOutput(int muxGateLines, int firstGate, int scanningOrder, int topBottom) {
    bool ok = true;

    // send command
    ok = ok & sendCommand(WS15EP2_CMD_DRIVER_OUTPUT);

    // send command data
    uint8_t data[3] = {
        (muxGateLines - 1) & 0xFF,
        ((muxGateLines - 1) >> 8) & 0b1,
        ((firstGate & 0b1) << 2) | ((scanningOrder & 0b1) << 1) | (topBottom & 0b1),
    };
    ok = ok & sendData(data, 3);

    return ok;
}

bool Ws15eP2_DeepSleep(void) {
    bool ok = true;

    // send command
    ok = ok & sendCommand(WS15EP2_CMD_DEEP_SLEEP_MODE);

    // send command data
    uint8_t data = 0x01; // Enter Deep Sleep Mode 1
    ok = ok & sendData(&data, 1);

    // NOTE from datasheet: After this command initiated, the chip will
    // enter Deep Sleep Mode, BUSY pad will keep
    // output high.
    // Remark:
    // To Exit Deep Sleep mode, User required to
    // send HWRESET to the driver

    // NOTE: the following is from Arduino example
    // platform->delayMs(200);
    // platform->gpioSet(platform->pinRst, 0);

    return ok;
}

bool Ws15eP2_SetDataEntryMode(int dataEntryMode, int dataDirection) {
    bool ok = true;

    // send command
    ok = ok & sendCommand(WS15EP2_CMD_DATA_ENTRY_MODE);

    // send command data
    uint8_t data = (dataEntryMode & 0b11) | ((dataDirection & 0b1) << 2);
    ok = ok & sendData(&data, 1);

    return ok;
}

bool Ws15eP2_SetLut(const uint8_t *lut) {
    bool ok = true;

    // NOTE: taken from https://github.com/waveshareteam/e-Paper/blob/master/Arduino/epd1in54_V2/epd1in54_V2.cpp

    // Write LUT (153 bytes)
    ok = ok & sendCommand(WS15EP2_CMD_WRITE_LUT);
    ok = ok & sendData(lut, 153);

    Ws15eP2_WaitBusy();

    // NOTE: command 0x3f is not given in datasheet but persist in example
    ok = ok & sendCommand(0x3f);
    ok = ok & sendData(&lut[153], 1);

    // Setup gate driving voltage
    ok = ok & sendCommand(WS15EP2_CMD_GATE_DRIVING_VOLTAGE);
    ok = ok & sendData(&lut[154], 1);

    // Setup source driving voltage
    ok = ok & sendCommand(WS15EP2_CMD_SOURCE_DRIVING_VOLTAGE);
    ok = ok & sendData(&lut[155], 3);

    // Write VCOM
    ok = ok & sendCommand(WS15EP2_CMD_VCOM_WRITE);
    ok = ok & sendData(&lut[158], 1);

    return ok;
}

bool Ws15eP2_SetTemperature(int tempDegC) {
    bool ok = true;

    // Convert temperature
    uint16_t temp = convertTemperature(tempDegC);

    // Send command
    // NOTE: this command is mentioned in datasheet but not described in command reference table
    ok = ok & sendCommand(0x1A);

    // Send command data
    uint8_t data[2] = {
        (temp & 0xFF),
        ((temp >> 8) & 0xFF),
    };
    ok = ok & sendData(data, 2);

    return ok; 
}

void Ws15eP2_HardwareReset(void) {
    // NOTE:  module reset. often used to awaken the module in deep sleep
    platform->gpioSet(platform->pinRst, 1);
    platform->delayMs(20);

    platform->gpioSet(platform->pinRst, 0); // module reset
    platform->delayMs(5);

    platform->gpioSet(platform->pinRst, 1);
    platform->delayMs(20);

    Ws15eP2_WaitBusy();
}

void Ws15eP2_WaitBusy(void) {
    // NOTE: timings taken from Arduino example

    while (platform->gpioGet(platform->pinBusy) != 0) {
        platform->delayMs(100);
    }

    platform->delayMs(200);
}

const char *Ws15eP2_UnitTest(void) {
    // Test temperature conversion
    mu_assert("convertTemperature(127) == 0x7F0", convertTemperature(127) == 0x7F0);
    mu_assert("convertTemperature(25) == 0x190", convertTemperature(25) == 0x190);
    mu_assert("convertTemperature(0) == 0x0", convertTemperature(0) == 0x0);
    mu_assert("convertTemperature(-25) == 0xE70", convertTemperature(-25) == 0xE70);
    mu_assert("convertTemperature(-55) == 0xC90", convertTemperature(-55) == 0xC90);

    return 0;
}
