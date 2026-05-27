
#ifdef __IMXRT1062__
  #include <Arduino.h>
#else
  #include <stdio.h>
#endif
#include <unity_config.h>

void unityOutputStart() {

#ifdef __IMXRT1062__
    Serial.begin(115200);
    while (!Serial && millis() < 4000) ;
#endif

}

void unityOutputChar(char c) {
    
#ifdef __IMXRT1062__
    Serial.write(c);
#else
    putchar(c); // Use standard output for test result reporting
#endif

}

void unityOutputFlush() {
    
#ifdef __IMXRT1062__
    Serial.flush();
#else
    fflush(stdout);  // Ensure all output is sent immediately
#endif

}

void unityOutputComplete() {}
