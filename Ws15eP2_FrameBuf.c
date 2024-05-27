#include <string.h>
#include <stddef.h>
#include "Ws15eP2_FrameBuf.h"

#define CHECK_X(x) (x >= 0 && x < WS15EP2_WIDTH)
#define CHECK_Y(y) (y >= 0 && y < WS15EP2_HEIGHT)
#define CHECK_X_WIDTH(x, width) (x >= 0 && x + width <= WS15EP2_WIDTH)
#define CHECK_Y_HEIGHT(y, height) (y >= 0 && y + height <= WS15EP2_HEIGHT)
#define CHECK_FB(fbData) (fbData != NULL)

static int fbWidth;
static int fbHeight;
static uint8_t *fbData = NULL;
static int fbSize;

static int getDataPixel(int x, int y, int width, int height, const uint8_t *data) {
    // Data is sequenced left to right, top to bottom, MSB first; the last byte in row may contain padding bits
    int widthBytes = (width % 8) ? (width / 8) + 1 : width / 8;
    int offset = y * widthBytes + x / 8;
    int bitOffset = 7 - (x % 8);
    return (data[offset] >> bitOffset) & 0b1;
}

static void setDataPixel(int x, int y, int width, int height, uint8_t *data, int value) {
    // Data is sequenced left to right, top to bottom, MSB first; the last byte in row may contain padding bits
    int widthBytes = (width % 8) ? (width / 8) + 1 : width / 8;
    int offset = y * widthBytes + x / 8;
    int bitOffset = 7 - (x % 8);
    if (value) {
        data[offset] |= (1 << bitOffset);
    } else {
        data[offset] &= ~(1 << bitOffset);
    }
}

bool Ws15eP2_FrameBuf_Init(int width8x_, int height_, uint8_t *fb_, int fbSize_) {
    // Check FB size
    if (width8x_ % 8 != 0) {
        return false;
    }

    int expectedSize = WS15EP2_FRAMEBUF_SIZE(width8x_, height_);
    if (fbSize_ != expectedSize) {
        return false;
    }

    // Save parameters
    fbWidth = width8x_;
    fbHeight = height_;
    fbData = fb_;
    fbSize = fbSize_;

    // Clear FB
    memset(fbData, 0xFF, fbSize);

    return true;
}

bool Ws15eP2_FrameBuf_Clear(void) {
    return Ws15eP2_FrameBuf_Fill(1);
}

bool Ws15eP2_FrameBuf_Fill(int value) {
    if (!CHECK_FB(fbData)) {
        return false;
    }

    // Prepare value to set
    uint8_t valueData = value ? 0xFF : 0x00;

    // Set all pixels
    memset(fbData, valueData, fbSize);

    return true;
}

bool Ws15eP2_FrameBuf_SetPixel(int x, int y, int value) {
    if (!CHECK_X(x) || !CHECK_Y(y) || !CHECK_FB(fbData)) {
        return false;
    }

    setDataPixel(x, y, fbWidth, fbHeight, fbData, value);

    return true;
}

bool Ws15eP2_FrameBuf_FillArea(int x, int y, int width, int height, int value) {
    if (!CHECK_X_WIDTH(x, width) || !CHECK_Y_HEIGHT(y, height) || !CHECK_FB(fbData)) {
        return false;
    }

    // FIXME: optimize
    for (int y1 = y; y1 < y + height; y1++) {
        for (int x1 = x; x1 < x + width; x1++) {
            setDataPixel(x1, y1, fbWidth, fbHeight, fbData, value);
        }
    }

    return true;
}

bool Ws15eP2_FrameBuf_OutputBitmap(int x, int y, int width, int height, const uint8_t *data, int dataSize) {
    if (!CHECK_X_WIDTH(x, width) || !CHECK_Y_HEIGHT(y, height) || !CHECK_FB(fbData)) {
        return false;
    }

    // Check data size
    int widthBytes = (width % 8) ? (width / 8) + 1 : width / 8;
    int expectedSize = widthBytes * height;
    if (dataSize != expectedSize) {
        return false;
    }

    // FIXME: optimize
    for (int y1 = y; y1 < y + height; y1++) {
        for (int x1 = x; x1 < x + width; x1++) {
            int value = getDataPixel(x1 - x, y1 - y, width, height, data);
            setDataPixel(x1, y1, fbWidth, fbHeight, fbData, value);
        }
    }

    return true;
}
