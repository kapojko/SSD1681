#include "MinUnit.h"
#include "SSD1681.h"

#define SSD1681_CMD_DRIVER_OUTPUT 0x01
#define SSD1681_CMD_GATE_DRIVING_VOLTAGE 0x03
#define SSD1681_CMD_SOURCE_DRIVING_VOLTAGE 0x04
#define SSD1681_CMD_DEEP_SLEEP_MODE 0x10
#define SSD1681_CMD_DATA_ENTRY_MODE 0x11
#define SSD1681_CMD_SW_RESET 0x12
#define SSD1681_CMD_TEMP_SENSOR 0x18 // NOTE: taken from SSD1681 datasheet
#define SSD1681_CMD_WRITE_TEMP_REGISTER 0x1A // NOTE: taken from SSD1681 datasheet
#define SSD1681_CMD_READ_TEMP_REGISTER 0x1B // NOTE: taken from SSD1681 datasheet
#define SSD1681_CMD_MASTER_ACTIVATION 0x20
#define SSD1681_CMD_DISPLAY_UPDATE_CONTROL_1 0x21
#define SSD1681_CMD_DISPLAY_UPDATE_CONTROL_2 0x22
#define SSD1681_CMD_WRITE_RAM_BW 0x24
#define SSD1681_CMD_WRITE_RAM_RED 0x26
#define SSD1681_CMD_VCOM_SENSE 0x28
#define SSD1681_CMD_PROGRAM_VCOM_OTP 0x2A
#define SSD1681_CMD_VCOM_CONTROL 0x2B
#define SSD1681_CMD_VCOM_WRITE 0x2C
#define SSD1681_CMD_READ_DISPLAY_OPTION_OTP 0x2D
#define SSD1681_CMD_READ_USER_ID 0x2E
#define SSD1681_CMD_PROGRAM_WS_OTP 0x30
#define SSD1681_CMD_LOAD_WS_OTP 0x31
#define SSD1681_CMD_WRITE_LUT 0x32
#define SSD1681_CMD_PROGRAM_OTP_SELECTION 0x36
#define SSD1681_CMD_WRITE_DISPLAY_OPTION 0x37 // NOTE: taken from SSD1681 datasheet
#define SSD1681_CMD_WRITE_USER_ID 0x38
#define SSD1681_CMD_OTP_PROGRAM_MODE 0x39
#define SSD1681_CMD_BORDER_WAVEFORM 0x3C // NOTE: taken from SSD1681 datasheet, missed in original one
#define SSD1681_CMD_SET_RAM_X 0x44
#define SSD1681_CMD_SET_RAM_Y 0x45
#define SSD1681_CMD_SET_RAM_X_COUNTER 0x4E
#define SSD1681_CMD_SET_RAM_Y_COUNTER 0x4F

#define CHECK_X(x8x, width8x) (x8x >= 0 && width8x > 0 && x8x + width8x <= SSD1681_WIDTH && x8x % 8 == 0 && width8x % 8 == 0)
#define CHECK_Y(y, height) (y >= 0 && height > 0 && y + height <= SSD1681_HEIGHT)
#define DATA_SIZE(width8x, height) (width8x / 8 * height)

static struct SSD1681_Platform *platform;

// see https://github.com/waveshareteam/e-Paper/blob/master/Arduino/epd1in54_V2/epd1in54_V2.cpp
// the same values also in https://github.com/waveshareteam/e-Paper/blob/master/STM32/STM32-F103ZET6/User/e-Paper/EPD_1in54_V2.c
// it could be not that since the voltage config is in last bytes
uint8_t SSD1681_WF_Full_1IN54[159] = {
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
uint8_t SSD1681_WF_PARTIAL_1IN54_0[159] = {
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

    // Check busy
    if (platform->gpioGet(platform->pinBusy)) {
        platform->debugPrint("sendCommand failed, busy\r\n");
        return false;
    }

    // Select command using DC
    platform->gpioSet(platform->pinDc, 0);

    // Send data with CS
    res = platform->spiSendWithCs(&command, 1, false);

    if (res != 0) {
        platform->debugPrint("sendCommand failed, spiSendWithCs returned %d\r\n", res);
        return false;
    }

    return true;
}

static bool sendData(const uint8_t *data, int len) {
    int res;

    // Check busy
    if (platform->gpioGet(platform->pinBusy)) {
        platform->debugPrint("sendData failed, busy\r\n");
        return false;
    }

    // Select data using DC
    platform->gpioSet(platform->pinDc, 1);

    // Send data with CS
    res = platform->spiSendWithCs(data, len, false);

    if (res != 0) {
        platform->debugPrint("sendData failed, spiSendWithCs returned %d\r\n", res);
        return false;
    }

    return true;
}

static bool readCommandData(uint8_t command, uint8_t *data, int len) {
    int res = 0;

    // Check busy
    if (platform->gpioGet(platform->pinBusy)) {
        platform->debugPrint("readCommandData failed, busy\r\n");
        return false;
    }

    // Begin SPI transaction
    platform->spiBegin();

    // Select command using DC
    platform->gpioSet(platform->pinDc, 0);

    // Write command (keep CS active after that)
    res |= platform->spiSendWithCs(&command, 1, true);

    // Select data using DC
    platform->gpioSet(platform->pinDc, 1);

    // Receive data using CS (dont't keep CS active)
    res |= platform->spiRecvWithCs(data, len, false);

    // End SPI transaction
    platform->spiEnd();

    if (res != 0) {
        platform->debugPrint("readCommandData failed, spiRecvWithCs returned %d\r\n", res);
        return false;
    }

    return true;
}

static uint16_t encodeTemperature(int tempDegC) {
    // Apply LSB
    int16_t tempSigned = tempDegC * 16;

    // Convert to 2's supplement (12 bits only)
    uint16_t temp = (*(uint16_t *)&tempSigned) & 0xFFF;

    return temp;
}

static int decodeTemperature(uint16_t temp) {
    // Fill last 4 bits with the same value as 12 bit
    uint16_t temp16Bit = temp;
    int negativeSign = temp16Bit & 0x800;
    if (negativeSign) {
        temp16Bit |= 0xF000;
    }

    // Convert to signed
    int16_t tempSigned = *(int16_t *)&temp16Bit;

    // Convert to Celsius
    int tempC = tempSigned / 16;

    return tempC;
}

void SSD1681_Init(struct SSD1681_Platform *platformPtr) {
    platform = platformPtr;
}

bool SSD1681_SendGenericCommand(uint8_t command, const uint8_t *data, int len) {
    bool ok = true;

    // send command
    ok = ok & sendCommand(command);

    // send data (if provided)
    if (data) {
        ok = ok & sendData(data, len);
    }

    return ok;
}

bool SSD1681_SetDriverOutput(int muxGateLines, int firstGate, int scanningOrder, int topBottom) {
    bool ok = true;

    // send command
    ok = ok & sendCommand(SSD1681_CMD_DRIVER_OUTPUT);

    // send command data
    // NOTE: datasheet states that DC# bit should be 1 only for first data byte, this must be an error
    uint8_t data[3] = {
        (muxGateLines - 1) & 0xFF,
        ((muxGateLines - 1) >> 8) & 0b1,
        ((firstGate & 0b1) << 2) | ((scanningOrder & 0b1) << 1) | (topBottom & 0b1),
    };
    ok = ok & sendData(data, sizeof(data));

    return ok;
}

bool SSD1681_DeepSleep(void) {
    bool ok = true;

    // send command
    ok = ok & sendCommand(SSD1681_CMD_DEEP_SLEEP_MODE);

    // send command data
    uint8_t data = 0x01; // Enter Deep Sleep Mode 1
    ok = ok & sendData(&data, sizeof(data));

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

bool SSD1681_SetDataEntryMode(int dataEntryMode, int dataDirection) {
    bool ok = true;

    // send command
    ok = ok & sendCommand(SSD1681_CMD_DATA_ENTRY_MODE);

    // send command data
    uint8_t data = (dataEntryMode & 0b11) | ((dataDirection & 0b1) << 2);
    ok = ok & sendData(&data, sizeof(data));

    return ok;
}

bool SSD1681_SoftwareReset(void) {
    bool ok = true;

    ok = ok & sendCommand(SSD1681_CMD_SW_RESET);
    SSD1681_WaitBusy();

    return ok;
}

bool SSD1681_SetTemperatureSensor(int tempSensor) {
    // NOTE: the command is from SSD1681 datasheet

    bool ok = true;

    ok = ok & sendCommand(SSD1681_CMD_TEMP_SENSOR);

    uint8_t data = tempSensor & 0xFF;
    ok = ok & sendData(&data, sizeof(data));

    return ok;
}

bool SSD1681_WriteTemperature(int tempDegC) {
    // NOTE: the command is from SSD1681 datasheet
    // (it is mentioned in the original datasheet but not described in command reference table)

    bool ok = true;

    // Convert temperature
    uint16_t temp = encodeTemperature(tempDegC);

    // Send command
    ok = ok & sendCommand(SSD1681_CMD_WRITE_TEMP_REGISTER);

    // Send command data
    uint8_t data[2] = {
        ((temp >> 4) & 0xFF),
        ((temp << 4) & 0xF0),
    };
    ok = ok & sendData(data, sizeof(data));

    return ok; 
}

bool SSD1681_ReadTemperature(int *tempDegC) {
    // NOTE: the command is from SSD1681 datasheet

    bool ok = true;

    // Send command and read 2 bytes
    uint8_t data[2];
    ok = ok & readCommandData(SSD1681_CMD_READ_TEMP_REGISTER, data, 2);

    // Extract and convert temperature
    uint16_t tempEncoded = (data[0] << 4) | (data[1] >> 4);
    *tempDegC = decodeTemperature(tempEncoded);

    return ok;
}


bool SSD1681_MasterActivation(void) {
    bool ok = true;

    ok = ok & sendCommand(SSD1681_CMD_MASTER_ACTIVATION);
    SSD1681_WaitBusy();

    return ok;
}

bool SSD1681_SetRamContentOption(int redRamOption, int bwRamOption) {
    bool ok = true;

    // send command
    ok = ok & sendCommand(SSD1681_CMD_DISPLAY_UPDATE_CONTROL_1);

    // send command data
    uint8_t data[2] = {
        ((redRamOption & 0b1111) << 4) | ((bwRamOption & 0b1111) << 0),
        0
    };
    ok = ok & sendData(data, sizeof(data));

    return ok;
}

bool SSD1681_SetDisplayUpdateSequence(int displayUpdateSequence) {
    bool ok = true;

    // send command
    ok = ok & sendCommand(SSD1681_CMD_DISPLAY_UPDATE_CONTROL_2);

    // send command data
    uint8_t data = displayUpdateSequence;
    ok = ok & sendData(&data, sizeof(data));

    return ok;
}

bool SSD1681_WriteRAMbw(const uint8_t *data, int len) {
    bool ok = true;

    // send command
    ok = ok & sendCommand(SSD1681_CMD_WRITE_RAM_BW);

    // send data
    ok = ok & sendData(data, len);

    return ok;
}

bool SSD1681_WriteRAMred(const uint8_t *data, int len) {
    bool ok = true;

    // send command
    ok = ok & sendCommand(SSD1681_CMD_WRITE_RAM_RED);

    // send data
    ok = ok & sendData(data, len);

    return ok;
}

bool SSD1681_ReadUserID(uint8_t *data10bytes) {
    bool ok = true;

    ok &= readCommandData(SSD1681_CMD_READ_USER_ID, data10bytes, 10);

    return ok;
}

bool SSD1681_SetLutAndVoltage(const uint8_t *lut) {
    bool ok = true;

    // NOTE: taken from https://github.com/waveshareteam/e-Paper/blob/master/Arduino/epd1in54_V2/epd1in54_V2.cpp

    // Write LUT (153 bytes)
    ok = ok & sendCommand(SSD1681_CMD_WRITE_LUT);
    ok = ok & sendData(lut, 153);

    SSD1681_WaitBusy();

    // NOTE: command 0x3f is not given in datasheet but persist in example
    ok = ok & sendCommand(0x3f);
    ok = ok & sendData(&lut[153], 1);

    // Setup gate driving voltage
    ok = ok & sendCommand(SSD1681_CMD_GATE_DRIVING_VOLTAGE);
    ok = ok & sendData(&lut[154], 1);

    // Setup source driving voltage
    ok = ok & sendCommand(SSD1681_CMD_SOURCE_DRIVING_VOLTAGE);
    ok = ok & sendData(&lut[155], 3);

    // Write VCOM
    ok = ok & sendCommand(SSD1681_CMD_VCOM_WRITE);
    ok = ok & sendData(&lut[158], 1);

    return ok;
}

bool SSD1681_WriteUserID(const uint8_t *data10bytes) {
    bool ok = true;

    // send command
    ok = ok & sendCommand(SSD1681_CMD_WRITE_USER_ID);

    // send data
    ok = ok & sendData(data10bytes, 10);

    return ok;
}

bool SSD1681_SetRAMxAddress(int xStart, int xEnd) {
    bool ok = true;

    // send command
    ok = ok & sendCommand(SSD1681_CMD_SET_RAM_X);

    // send command data
    uint8_t data[2] = {
        xStart & 077,
        xEnd & 077
        
    };
    ok = ok & sendData(data, sizeof(data));

    return ok;
}

bool SSD1681_SetRAMyAddress(int yStart, int yEnd) {
    bool ok = true;

    // send command
    ok = ok & sendCommand(SSD1681_CMD_SET_RAM_Y);

    // send command data
    uint8_t data[4] = {
        yStart & 0xFF,
        (yStart >> 8) & 0b1,
        yEnd & 0xFF,
        (yEnd >> 8) & 0b1
    };
    ok = ok & sendData(data, sizeof(data));

    return ok;
}

bool SSD1681_SetRAMxCounter(int xCounter) {
    bool ok = true;

    // send command
    ok = ok & sendCommand(SSD1681_CMD_SET_RAM_X_COUNTER);

    // send command data
    uint8_t data = xCounter & 077;
    ok = ok & sendData(&data, sizeof(data));

    return ok;
}

bool SSD1681_SetRAMyCounter(int yCounter) {
    bool ok = true;

    // send command
    ok = ok & sendCommand(SSD1681_CMD_SET_RAM_Y_COUNTER);

    // send command data
    uint8_t data[2] = {
        yCounter & 0xFF,
        (yCounter >> 8) & 0b1
    };
    ok = ok & sendData(data, sizeof(data));

    return ok;
}

void SSD1681_HardwareReset(void) {
    // NOTE:  module reset. often used to awaken the module in deep sleep
    // NOTE: timings in example: 20, 5, 20
    // timings in datasheet: 10, 10, 10

    platform->gpioSet(platform->pinRst, 1);
    platform->delayMs(10);

    platform->gpioSet(platform->pinRst, 0); // module reset
    platform->delayMs(10);

    platform->gpioSet(platform->pinRst, 1);
    platform->delayMs(10);

    SSD1681_WaitBusy();
}

void SSD1681_WaitBusy(void) {
    // NOTE: timings taken from Arduino example

    while (platform->gpioGet(platform->pinBusy) != 0) {
        platform->delayMs(100);
    }

    platform->delayMs(200);
}

bool SSD1681_DefInitFull() {
    // Based on SSD1681 datasheet plus Arduino example, and keeping in mind the original Waveshare datasheet

    bool ok = true;

    // INITIAL CONFIGURATION

    // Perform hardware reset
    SSD1681_HardwareReset();

    // Software reset
    ok = ok & SSD1681_SoftwareReset();

    // Wait 10 ms
    platform->delayMs(10);

    // SEND INITIALIZATION CODE

    // Driver output control
    ok = ok & SSD1681_SetDriverOutput(SSD1681_MUX_GATE_LINES_DEF, SSD1681_GATE_DEF, SSD1681_SCAN_DEF, SSD1681_TOP_BOTTOM_REVERSE);

    // Data entry mode
    ok = ok & SSD1681_SetDataEntryMode(SSD1681_DATA_ENTRY_YDEC_XINC, SSD1681_DATA_DIRECTION_X);

    // RAM x and y address start/end position
    ok = ok & SSD1681_SetRAMxAddress(0, (SSD1681_WIDTH / 8) - 1);
    ok = ok & SSD1681_SetRAMyAddress(SSD1681_HEIGHT - 1, 0);

    // Border waveform
    uint8_t borderWaveformData = 0x01;
    ok = ok & SSD1681_SendGenericCommand(SSD1681_CMD_BORDER_WAVEFORM, &borderWaveformData, sizeof(borderWaveformData));

    // LOAD WAVEFORM LUT

    // Temperature sensor control
    ok = ok & SSD1681_SetTemperatureSensor(SSD1681_TEMP_SENSOR_INTERNAL);

    // Setup display update sequence - load Temperature and waveform setting
    ok = ok & SSD1681_SetDisplayUpdateSequence(SSD1681_UPD_SEQ_EC_LT_L1_DC);

    // Master activation
    ok = ok & SSD1681_MasterActivation();

    // Set LUT
    ok = ok & SSD1681_SetLutAndVoltage(SSD1681_WF_Full_1IN54);

    if (!ok) {
        platform->debugPrint("SSD1681_DefInitFull failed\r\n");
        return false;
    }

    return true;
}

#if 0
bool SSD1681_DefInitFull_Arduino() {
    // NOTE: from Arduino example

    bool ok = true;

    // Perform hardware reset
    SSD1681_HardwareReset();

    // Software reset
    ok = ok & SSD1681_SoftwareReset();

    // Driver output control
    ok = ok & SSD1681_SetDriverOutput(SSD1681_MUX_GATE_LINES_DEF, SSD1681_GATE_DEF, SSD1681_SCAN_DEF, SSD1681_TOP_BOTTOM_REVERSE);

    // Data entry mode
    ok = ok & SSD1681_SetDataEntryMode(SSD1681_DATA_ENTRY_YDEC_XINC, SSD1681_DATA_DIRECTION_X);

    // RAM x address start/end position (in address units of 8 bits)
    ok = ok & SSD1681_SetRAMxAddress(0, (200 / 8) - 1);
    
    // RAM y address start/end position
    ok = ok & SSD1681_SetRAMyAddress(199, 0);

    // Set BorderWavefrom
    // NOTE: the command missed in the original datasheet, taken from Arduino example and SSD1681 datasheet
    uint8_t borderWaveformData = 0x01;
    ok = ok & SSD1681_SendGenericCommand(SSD1681_CMD_BORDER_WAVEFORM, &borderWaveformData, sizeof(borderWaveformData));

    // Command 0x18
    // NOTE: the command missed in the datasheet but occurs in Arduino example
    uint8_t command18Data = 0x80;
    ok = ok & SSD1681_SendGenericCommand(SSD1681_CMD_TEMP_SENSOR, &command18Data, sizeof(command18Data));

    // Setup display update sequence - load Temperature and waveform setting
    ok = ok & SSD1681_SetDisplayUpdateSequence(SSD1681_UPD_SEQ_EC_LT_L1_DC);

    // set RAM x address count to 0
    ok = ok & SSD1681_SetRAMxCounter(0);

    // set RAM y address count to 199
    ok = ok & SSD1681_SetRAMyCounter(199);

    SSD1681_WaitBusy();

    // Set LUT
    ok = ok & SSD1681_SetLutAndVoltage(SSD1681_WF_Full_1IN54);

    // Hardware initialized
    return ok;
}
#endif

#if 0
bool SSD1681_DefInitFullPartial_Arduino() {
    // NOTE: from Arduino example

    bool ok = true;

    // Perform hardware reset
    SSD1681_HardwareReset();

    // Software reset
    ok = ok & SSD1681_SoftwareReset();

    // Set LUT for partial display
    ok = ok & SSD1681_SetLutAndVoltage(SSD1681_WF_PARTIAL_1IN54_0);

    // Send command 0x37
    uint8_t command37Data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00};
    ok = ok & SSD1681_SendGenericCommand(SSD1681_CMD_WRITE_DISPLAY_OPTION, command37Data, sizeof(command37Data));

    // Set BorderWavefrom
    // NOTE: the command missed in the datasheet but occurs in Arduino example
    uint8_t borderWaveformData = 0x80;
    ok = ok & SSD1681_SendGenericCommand(SSD1681_CMD_BORDER_WAVEFORM, &borderWaveformData, sizeof(borderWaveformData));

    // Setup display update sequence - just enable clock and analog
    ok = ok & SSD1681_SetDisplayUpdateSequence(SSD1681_UPD_SEQ_EC_EA);

    // Master activation
    ok = ok & SSD1681_MasterActivation();

    return ok;
}
#endif

bool SSD1681_RefreshDisplay(void) {
    bool ok = true;

    // Setup display update sequence
    ok = ok & SSD1681_SetDisplayUpdateSequence(SSD1681_UPD_SEQ_EC_EA_D1_DA_DO);

    // Master activation
    ok = ok & SSD1681_MasterActivation();

    return ok;
}

bool SSD1681_ClearScreen(bool clearBW, bool clearRed) {
    bool ok = true;

    int widthBytes = SSD1681_WIDTH / 8;
    int heightBytes = SSD1681_HEIGHT;
    uint8_t data = 0xFF;

    // Set RAM start/end address
    ok = ok & SSD1681_SetRAMxAddress(0, widthBytes - 1);
    ok = ok & SSD1681_SetRAMyAddress(heightBytes - 1, 0);

    // Set RAM counter
    ok = ok & SSD1681_SetRAMxCounter(0);
    ok = ok & SSD1681_SetRAMyCounter(heightBytes - 1);

    // Clear BW RAM (write ones)
    if (clearBW) {
        ok = ok & sendCommand(SSD1681_CMD_WRITE_RAM_BW);
        for (int y = 0; y < heightBytes; y++) {
            for (int x = 0; x < widthBytes; x++) {
                ok = ok & sendData(&data, sizeof(data));
            }
        }
    }

    // Clear RED RAM (write ones)
    if (clearRed) {
        ok = ok & sendCommand(SSD1681_CMD_WRITE_RAM_RED);
        for (int y = 0; y < heightBytes; y++) {
            for (int x = 0; x < widthBytes; x++) {
                ok = ok & sendData(&data, sizeof(data));
            }
        }
    }

    // Refresh display
    ok = ok & SSD1681_MasterActivation();

    return ok;
}

bool SSD1681_OutputBitmap(int x8x, int y, int width8x, int height, const uint8_t *data, int dataSize) {
    bool ok = true;

    // Check x and y
    if (!CHECK_X(x8x, width8x)) {
        platform->debugPrint("OutputBitmap failed, invalid x8x or width8x: %d, %d\r\n", x8x, width8x);
        return false;
    }

    if (!CHECK_Y(y, height)) {
        platform->debugPrint("OutputBitmap failed, invalid y or height: %d, %d\r\n", y, height);
        return false;
    }

    // Check data size
    if (dataSize != DATA_SIZE(width8x, height)) {
        platform->debugPrint("OutputBitmap failed, invalid dataSize: %d, expected %d\r\n", dataSize, DATA_SIZE(width8x, height));
        return false;
    }

    // Set start/end position
    ok = ok & SSD1681_SetRAMxAddress(x8x / 8, (x8x + width8x) / 8 - 1);
    ok = ok & SSD1681_SetRAMyAddress(y + height - 1, y);

    // Set address counters
    ok = ok & SSD1681_SetRAMxCounter(x8x / 8);
    ok = ok & SSD1681_SetRAMyCounter(y + height - 1);

    // Write BW RAM
    ok = ok & SSD1681_WriteRAMbw(data, dataSize);

    if (!ok) {
        platform->debugPrint("OutputBitmap failed, errors from SPI or commands\r\n");
        return false;
    }

    return true;
}

bool SSD1681_FillArea(int x8x, int y, int width8x, int height, uint8_t value) {
    bool ok = true;

    // Check x and y
    if (!CHECK_X(x8x, width8x)) {
        platform->debugPrint("FillArea failed, invalid x8x or width8x: %d, %d\r\n", x8x, width8x);
        return false;
    }

    if (!CHECK_Y(y, height)) {
        platform->debugPrint("FillArea failed, invalid y or height: %d, %d\r\n", y, height);
        return false;
    }

    // Set start/end position
    ok = ok & SSD1681_SetRAMxAddress(x8x / 8, (x8x + width8x) / 8 - 1);
    ok = ok & SSD1681_SetRAMyAddress(y + height - 1, y);

    // Set address counters
    ok = ok & SSD1681_SetRAMxCounter(x8x / 8);
    ok = ok & SSD1681_SetRAMyCounter(y + height - 1);

    // Write BW RAM
    ok = ok & sendCommand(SSD1681_CMD_WRITE_RAM_BW);

    int dataSize = DATA_SIZE(width8x, height);
    for (int i = 0; i < dataSize; i++) {
        ok = ok & sendData(&value, sizeof(value));
    }

    if (!ok) {
        platform->debugPrint("FillArea failed, errors from SPI or commands\r\n");
        return false;
    }

    return true;
}

const char *SSD1681_UnitTest(void) {
    // Test temperature conversion
    mu_assert("encodeTemperature(127) == 0x7F0", encodeTemperature(127) == 0x7F0);
    mu_assert("encodeTemperature(25) == 0x190", encodeTemperature(25) == 0x190);
    mu_assert("encodeTemperature(0) == 0x0", encodeTemperature(0) == 0x0);
    mu_assert("encodeTemperature(-25) == 0xE70", encodeTemperature(-25) == 0xE70);
    mu_assert("encodeTemperature(-55) == 0xC90", encodeTemperature(-55) == 0xC90);

    // Test temperature decoding
    uint16_t encoded25C = encodeTemperature(25);
    mu_assert("decodeTemperature(encoded25C) == 25", decodeTemperature(encoded25C) == 25);
    uint16_t encoded0C = encodeTemperature(0);
    mu_assert("decodeTemperature(encoded0C) == 0", decodeTemperature(encoded0C) == 0);
    uint16_t encodedNeg25C = encodeTemperature(-25);
    mu_assert("decodeTemperature(encodedNeg25C) == -25", decodeTemperature(encodedNeg25C) == -25);

    return 0;
}
