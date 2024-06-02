# SSD1681

Platform-agnostic library for SSD1681 E-Ink displays for embedded applications. Example hardware:

* Waveshare 1.5inch e-Paper V2.

## Output modes

Library provedes two output modes:

* Direct output to e-Paper RAM. Data and bitmaps are transmitted as is, so output *x* coordinate and *width* must be byte aligned.
* Framebuffer output. No alignment requirements for *x* and *width* (notwithstandingly, each row must be byte aligned using padding bits in the last byte). After paining all data to framebuffer, it can be outputted to e-Paper using direct output mode.

## Bitmap format and generation

Bitmaps should have the following format:

* Every byte bits from left to right (MSB on the left).
* Bytes from left to right forming image row, the last byte in a row may contain padding bits (row must be byte aligned).
* Rows from top to bottom forming image.
* Direct monochrome output (no inversion), background is 1, foreground is 0.

Fonts and bitmaps can be generated using *lcd-image-converter* is used ( [Github](https://github.com/riuson/lcd-image-converter), [Sourceforge](https://sourceforge.net/projects/lcd-image-converter/) ). Conversion rules are saved in `SSD1681.xml` file.

## Testing

Integrated unit-testing is implemented using *MinUnit.h* and *SR1612Z1_Test.c* files. Tests are compiled and run by *CTest* and automated using GitHub Actions.

## Precautions

***WARNING: The following precautions should be taken when using the module, see official [wiki](https://www.waveshare.com/wiki/1.54inch_e-Paper_Module_Manual#FAQ).***

* Refresh mode
    * Full refresh: The e-paper screen will flicker several times during the refresh process (the number of flickers depends on the refresh time), and the flicker is to remove the afterimage to achieve the best display effect.
    * Partial refresh: The e-paper screen has no flickering effect during the refresh process. Users who use the partial brushing function note that after refreshing several times, a full refresh operation should be performed to remove the residual image, otherwise the residual image problem will become more and more serious, or even damage the screen (currently only some black and white e-paper screens support partial refreshing, please refer to product page description).
* Refresh rate
    * During use, it is recommended that customers set the refresh interval of the e-paper screen to at least 180 seconds (except for products that support the local brush function).
    * During the standby process (that is, after the refresh operation), it is recommended that the customer set the e-paper screen to sleep mode, or power off (the power supply part of the e-paper screen can be disconnected with an analog switch) to reduce power consumption and prolong the life of the e-paper screen. (If some e-paper screens are powered on for a long time, the screen will be damaged beyond repair.)
    * During the use of the multi-color e-paper screen, it is recommended that customers update the display screen at least once every 24 hours. (If the screen keeps the same picture for a long time, the screen will burn and it is difficult to repair.)
* Use Environment
    * The e-paper screen is recommended for indoor use. If it is used outdoors, it is necessary to avoid direct sunlight on the e-paper screen, and at the same time, take UV protection measures, because charged particles will dry out under strong light for a long time, resulting in loss of activity and failure to refresh. This situation is irreversible. When designing e-paper screen products, customers should pay attention to determining whether the use environment meets the requirements of an e-paper screen.
* Power
    * Power on the development board for a long time, after each refresh operation, it is recommended to set the screen to sleep mode or directly power off processing, otherwise, the screen may burn out when the screen is in a high voltage state for a long time.

## Author

Yuriy Kapoyko - ykapoyko@vk.com
