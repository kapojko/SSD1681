#ifndef WS15EP2_H
#define WS15EP2_H

// Based on https://github.com/waveshareteam/e-Paper/tree/master/Arduino/epd1in54_V2

#include <stdbool.h>
#include <stdint.h>

#define WS15EP2_WIDTH 200
#define WS15EP2_HEIGHT 200

#define WS15EP2_DPI 188

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

struct Ws15eP2_Platform {
    int (*gpioGet)(int pin);
    void (*gpioSet)(int pin, int state);

    int (*spiSend)(uint8_t *data, int len);
    int (*spiRecv)(uint8_t *data, int len);
    
    void (*delayMs)(int ms);
    void (*debugPrint)(const char *fmt, ...);

    uint8_t pinCs;
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
bool Ws15eP2_SetLut(const uint8_t *lut);

const char *Ws15eP2_UnitTest(void);

#endif // WS15EP2_H
