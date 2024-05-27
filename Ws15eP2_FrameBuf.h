#ifndef WS15EP2_FRAMEBUF_H
#define WS15EP2_FRAMEBUF_H

#include <stdint.h>
#include "Ws15eP2.h"

#define WS15EP2_FRAMEBUF_SIZE(width8x, height) (width8x / 8 * height)

bool Ws15eP2_FrameBuf_Init(int width8x, int height, uint8_t *fb, int fbSize);

bool Ws15eP2_FrameBuf_Clear(void);
bool Ws15eP2_FrameBuf_Fill(int value);
bool Ws15eP2_FrameBuf_SetPixel(int x, int y, int value);

bool Ws15eP2_FrameBuf_FillArea(int x, int y, int width, int height, int value);
bool Ws15eP2_FrameBuf_OutputBitmap(int x, int y, int width, int height, const uint8_t *data, int dataSize);

#endif // WS15EP2_FRAMEBUF_H
