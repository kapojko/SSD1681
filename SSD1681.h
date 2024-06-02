#ifndef SSD1681_H
#define SSD1681_H

// Based on https://github.com/waveshareteam/e-Paper/tree/master/Arduino/epd1in54_V2
// Note, that original datasheet contains a lot of errors
// But the module uses SSD1681 chip which datasheet is much more accurate

#include <stdbool.h>
#include <stdint.h>

#define SSD1681_WIDTH 200
#define SSD1681_HEIGHT 200

#define SSD1681_DPI 188

#define SSD1681_MAX_DATA_SIZE (SSD1681_WIDTH / 8 * SSD1681_HEIGHT)

// SPI config: MSB first
#define SSD1681_SPI_MAX_WRITE_FREQ 20000000 // 20 MHz
#define SSD1681_SPI_MAX_READ_FREQ 2500000 // 2.5 MHz

#define SSD1681_MUX_GATE_LINES_DEF 200

#define SSD1681_GATE_0 0 // G0 is the 1st gate output channel, gate output sequence is G0,G1, G2, G3, …
#define SSD1681_GATE_1 1 // G1 is the 1st gate output channel, gate output sequence is G1, G0, G3, G2, …
#define SSD1681_GATE_DEF SSD1681_GATE_0

#define SSD1681_SCAN_SEQ 0 // G0, G1, G2, G3…G199
#define SSD1681_SCAN_INTERLACED 1 // G0, G2, G4 …G198, G1, G3, …G199
#define SSD1681_SCAN_DEF SSD1681_SCAN_SEQ

#define SSD1681_TOP_BOTTOM 0 // scan from G0 to G199
#define SSD1681_TOP_BOTTOM_REVERSE 1 // scan from G199 to G0
#define SSD1681_TOP_BOTTOM_DEF SSD1681_TOP_BOTTOM

#define SSD1681_DATA_ENTRY_YDEC_XDEC 0b00
#define SSD1681_DATA_ENTRY_YDEC_XINC 0b01
#define SSD1681_DATA_ENTRY_YINC_XDEC 0b10
#define SSD1681_DATA_ENTRY_YINC_XINC 0b11
#define SSD1681_DATA_ENTRY_DEF SSD1681_DATA_ENTRY_YINC_XINC

#define SSD1681_DATA_DIRECTION_X 0
#define SSD1681_DATA_DIRECTION_Y 1
#define SSD1681_DATA_DIRECTION_DEF SSD1681_DATA_DIRECTION_X

#define SSD1681_RAM_OPTION_NORMAL 0
#define SSD1681_RAM_OPTION_BYPASS_0 0b100
#define SSD1681_RAM_OPTION_INVERSE 0b1000

#define SSD1681_TEMP_SENSOR_EXTERNAL 0x48
#define SSD1681_TEMP_SENSOR_INTERNAL 0x80
#define SSD1681_TEMP_SENSOR_DEF SSD1681_TEMP_SENSOR_EXTERNAL

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
#define SSD1681_UPD_SEQ_EC 0x80
#define SSD1681_UPD_SEQ_DC 0x01
#define SSD1681_UPD_SEQ_EC_EA 0xC0
#define SSD1681_UPD_SEQ_DA_DC 0x03
#define SSD1681_UPD_SEQ_EC_L1_DC 0x91
#define SSD1681_UPD_SEQ_EC_L2_DC 0x99
#define SSD1681_UPD_SEQ_EC_LT_L1_DC 0xB1
#define SSD1681_UPD_SEQ_EC_LT_L2_DC 0xB9
#define SSD1681_UPD_SEQ_EC_EA_D1_DA_DO 0xC7
#define SSD1681_UPD_SEQ_EC_EA_D2_DA_DO 0xCF
#define SSD1681_UPD_SEQ_EC_EA_LT_D1_DA_DO 0xF7
#define SSD1681_UPD_SEQ_EC_EA_LT_D2_DA_DO 0xFF

struct SSD1681_Platform {
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
extern uint8_t SSD1681_WF_Full_1IN54[159];

// waveform partial refresh
extern uint8_t SSD1681_WF_PARTIAL_1IN54_0[159];

void SSD1681_Init(struct SSD1681_Platform *platform);

bool SSD1681_SetDriverOutput(int muxGateLines, int firstGate, int scanningOrder, int topBottom);
bool SSD1681_DeepSleep(void);
bool SSD1681_SetDataEntryMode(int dataEntryMode, int dataDirection);
bool SSD1681_SoftwareReset(void);
bool SSD1681_SetTemperatureSensor(int tempSensor);
bool SSD1681_WriteTemperature(int tempDegC);
bool SSD1681_ReadTemperature(int *tempDegC);
bool SSD1681_MasterActivation(void);
bool SSD1681_SetRamContentOption(int redRamOption, int bwRamOption);
bool SSD1681_SetDisplayUpdateSequence(int displayUpdateSequence);
bool SSD1681_WriteRAMbw(const uint8_t *data, int len);
bool SSD1681_WriteRAMred(const uint8_t *data, int len);
bool SSD1681_ReadUserID(uint8_t *data10bytes);
bool SSD1681_SetLutAndVoltage(const uint8_t *lut);
bool SSD1681_WriteUserID(const uint8_t *data10bytes);
bool SSD1681_SetRAMxAddress(int xStart, int xEnd);
bool SSD1681_SetRAMyAddress(int yStart, int yEnd);
bool SSD1681_SetRAMxCounter(int xCounter);
bool SSD1681_SetRAMyCounter(int yCounter);

void SSD1681_HardwareReset(void);
void SSD1681_WaitBusy(void);

bool SSD1681_DefInitFull();
// NOTE: no default partial initialization for now

bool SSD1681_RefreshDisplay(void);
bool SSD1681_ClearScreen(bool clearBW, bool clearRed);
bool SSD1681_OutputBitmap(int x8x, int y, int width8x, int height, const uint8_t *data, int dataSize);
bool SSD1681_FillArea(int x8x, int y, int width8x, int height, uint8_t value);

const char *SSD1681_UnitTest(void);

#endif // SSD1681_H
