#ifdef TESTING

#include <stdio.h>
#include "SSD1681.h"

int main(void) {
    const char *unitTestResult = SSD1681_UnitTest();
    if (unitTestResult == 0) {
        printf("UNIT TEST PASSED\n");
        return 0;
    } else {
        printf("UNIT TEST FAILED: %s\n", unitTestResult);
        return 1;
    }
}

#endif
