#ifndef SSD1681_FRAMEBUF_H
#define SSD1681_FRAMEBUF_H

#include <stdint.h>
#include "SSD1681.h"

#define SSD1681_FRAMEBUF_SIZE(width8x, height) (width8x / 8 * height)

bool SSD1681_FrameBuf_Init(int width8x, int height, uint8_t *fb, int fbSize);

bool SSD1681_FrameBuf_Clear(void);
bool SSD1681_FrameBuf_Fill(int value);
bool SSD1681_FrameBuf_SetPixel(int x, int y, int value);

bool SSD1681_FrameBuf_FillArea(int x, int y, int width, int height, int value);
bool SSD1681_FrameBuf_FillArea_R90(int x, int y, int width, int height, int value);

bool SSD1681_FrameBuf_OutputBitmap(int x, int y, int width, int height, const uint8_t *data, int dataSize);
bool SSD1681_FrameBuf_OutputBitmap_R90(int x, int y, int width, int height, const uint8_t *data, int dataSize);

#endif // SSD1681_FRAMEBUF_H
