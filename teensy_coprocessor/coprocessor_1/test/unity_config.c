
#include <stdio.h>
#include <unity_config.h>

void unityOutputStart() {}

void unityOutputChar(char c) {
    // Output character to Serial for test result reporting
    // Serial.write(c);  // Uncomment if Serial output is available in test environment

    putchar(c);  // Use standard output for test result reporting
}

void unityOutputFlush() {
    fflush(stdout);  // Ensure all output is sent immediately
}

void unityOutputComplete() {}
