#ifndef WS15EP2_H
#define WS15EP2_H

// Based on https://github.com/waveshareteam/e-Paper/tree/master/Arduino/epd1in54_V2
// Note, that original datasheet contains a lot of errors
// But it seems, that module uses SSD1681 chip which datasheet is much more accurate

#include <stdbool.h>
#include <stdint.h>

#define WS15EP2_WIDTH 200
#define WS15EP2_HEIGHT 200

#define WS15EP2_DPI 188

#define WS15EP2_MAX_DATA_SIZE (WS15EP2_WIDTH / 8 * WS15EP2_HEIGHT)

// SPI config: MSB first
#define WS15EP2_SPI_MAX_WRITE_FREQ 20000000 // 20 MHz
#define WS15EP2_SPI_MAX_READ_FREQ 2500000 // 2.5 MHz

#define WS15EP2_MUX_GATE_LINES_DEF 200

#define WS15EP2_GATE_0 0 // G0 is the 1st gate output channel, gate output sequence is G0,G1, G2, G3, …
#define WS15EP2_GATE_1 1 // G1 is the 1st gate output channel, gate output sequence is G1, G0, G3, G2, …
#define WS15EP2_GATE_DEF WS15EP2_GATE_0

#define WS15EP2_SCAN_SEQ 0 // G0, G1, G2, G3…G199
#define WS15EP2_SCAN_INTERLACED 1 // G0, G2, G4 …G198, G1, G3, …G199
#define WS15EP2_SCAN_DEF WS15EP2_SCAN_SEQ

#define WS15EP2_TOP_BOTTOM 0 // scan from G0 to G199
#define WS15EP2_TOP_BOTTOM_REVERSE 1 // scan from G199 to G0
#define WS15EP2_TOP_BOTTOM_DEF WS15EP2_TOP_BOTTOM

#define WS15EP2_DATA_ENTRY_YDEC_XDEC 0b00
#define WS15EP2_DATA_ENTRY_YDEC_XINC 0b01
#define WS15EP2_DATA_ENTRY_YINC_XDEC 0b10
#define WS15EP2_DATA_ENTRY_YINC_XINC 0b11
#define WS15EP2_DATA_ENTRY_DEF WS15EP2_DATA_ENTRY_YINC_XINC

#define WS15EP2_DATA_DIRECTION_X 0
#define WS15EP2_DATA_DIRECTION_Y 1
#define WS15EP2_DATA_DIRECTION_DEF WS15EP2_DATA_DIRECTION_X

#define WS15EP2_RAM_OPTION_NORMAL 0
#define WS15EP2_RAM_OPTION_BYPASS_0 0b100
#define WS15EP2_RAM_OPTION_INVERSE 0b1000

#define WS15EP2_TEMP_SENSOR_EXTERNAL 0x48
#define WS15EP2_TEMP_SENSOR_INTERNAL 0x80
#define WS15EP2_TEMP_SENSOR_DEF WS15EP2_TEMP_SENSOR_EXTERNAL

// Display update sequence abbreviations:
// EC - enable clock signal
// DC - disable clock signal
// EA - enable analog
// DA - disable analog
// L1 - load LUT with Display Mode 1
// L2 - load LUT with Display Mode 2
// D1 - display with Display Mode 1
// D2 - display with Display Mode 2
// LT - load temperature value
// DO - disable OSC
#define WS15EP2_UPD_SEQ_EC 0x80
#define WS15EP2_UPD_SEQ_DC 0x01
#define WS15EP2_UPD_SEQ_EC_EA 0xC0
#define WS15EP2_UPD_SEQ_DA_DC 0x03
#define WS15EP2_UPD_SEQ_EC_L1_DC 0x91
#define WS15EP2_UPD_SEQ_EC_L2_DC 0x99
#define WS15EP2_UPD_SEQ_EC_LT_L1_DC 0xB1
#define WS15EP2_UPD_SEQ_EC_LT_L2_DC 0xB9
#define WS15EP2_UPD_SEQ_EC_EA_D1_DA_DO 0xC7
#define WS15EP2_UPD_SEQ_EC_EA_D2_DA_DO 0xCF
#define WS15EP2_UPD_SEQ_EC_EA_LT_D1_DA_DO 0xF7
#define WS15EP2_UPD_SEQ_EC_EA_LT_D2_DA_DO 0xFF

struct Ws15eP2_Platform {
    int (*gpioGet)(int pin);
    void (*gpioSet)(int pin, int state);

    void (*spiBegin)(void);
    int (*spiSendWithCs)(const uint8_t *data, int len, bool keepCS);
    int (*spiRecvWithCs)(uint8_t *data, int len, bool keepCS);
    void (*spiEnd)(void);

    void (*delayMs)(int ms);
    void (*debugPrint)(const char *fmt, ...);

    uint8_t pinRst;
    uint8_t pinDc;
    uint8_t pinBusy;
};

// waveform full refresh
extern uint8_t Ws15eP2_WF_Full_1IN54[159];

// waveform partial refresh
extern uint8_t Ws15eP2_WF_PARTIAL_1IN54_0[159];

void Ws15eP2_Init(struct Ws15eP2_Platform *platform);

bool Ws15eP2_SetDriverOutput(int muxGateLines, int firstGate, int scanningOrder, int topBottom);
bool Ws15eP2_DeepSleep(void);
bool Ws15eP2_SetDataEntryMode(int dataEntryMode, int dataDirection);
bool Ws15eP2_SoftwareReset(void);
bool Ws15eP2_SetTemperatureSensor(int tempSensor);
bool Ws15eP2_WriteTemperature(int tempDegC);
bool Ws15eP2_ReadTemperature(int *tempDegC);
bool Ws15eP2_MasterActivation(void);
bool Ws15eP2_SetRamContentOption(int redRamOption, int bwRamOption);
bool Ws15eP2_SetDisplayUpdateSequence(int displayUpdateSequence);
bool Ws15eP2_WriteRAMbw(const uint8_t *data, int len);
bool Ws15eP2_WriteRAMred(const uint8_t *data, int len);
bool Ws15eP2_ReadUserID(uint8_t *data10bytes);
bool Ws15eP2_SetLutAndVoltage(const uint8_t *lut);
bool Ws15eP2_WriteUserID(const uint8_t *data10bytes);
bool Ws15eP2_SetRAMxAddress(int xStart, int xEnd);
bool Ws15eP2_SetRAMyAddress(int yStart, int yEnd);
bool Ws15eP2_SetRAMxCounter(int xCounter);
bool Ws15eP2_SetRAMyCounter(int yCounter);

void Ws15eP2_HardwareReset(void);
void Ws15eP2_WaitBusy(void);

bool Ws15eP2_DefInitFull();
// NOTE: no default partial initialization for now

bool Ws15eP2_RefreshDisplay(void);
bool Ws15eP2_ClearScreen(bool clearBW, bool clearRed);
bool Ws15eP2_OutputBitmap(int x8x, int y, int width8x, int height, const uint8_t *data, int dataSize);
bool Ws15eP2_FillArea(int x8x, int y, int width8x, int height, uint8_t value);

const char *Ws15eP2_UnitTest(void);

#endif // WS15EP2_H
