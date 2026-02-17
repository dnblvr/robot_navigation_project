/**
 * @file Arduino_mock.h
 * @brief Mock Arduino functions for native PC simulation
 */

#ifndef ARDUINO_MOCK_H
#define ARDUINO_MOCK_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

// Mock Arduino types
typedef uint8_t byte;
typedef bool boolean;

// Mock Arduino functions
inline void delay(unsigned long ms) { /* no-op */ }
inline unsigned long millis() { return 0; }
inline unsigned long micros() { return 0; }

// Math functions
#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

#ifndef HALF_PI
#define HALF_PI 1.5707963267948966192313216916398
#endif

#ifndef TWO_PI
#define TWO_PI 6.283185307179586476925286766559
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295769236907684886
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105
#endif

// Bit manipulation
#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif

#ifndef bit
#define bit(b) (1UL << (b))
#endif

#ifndef bitRead
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#endif

#ifndef bitSet
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#endif

#ifndef bitClear
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#endif

#ifndef bitWrite
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
#endif

// Min/Max
#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif

#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif

#ifndef abs
#define abs(x) ((x)>0?(x):-(x))
#endif

#ifndef constrain
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

#ifndef sq
#define sq(x) ((x)*(x))
#endif

// Mock Serial for printf-style debugging
class MockSerial {
public:
    void begin(unsigned long baud) {}
    void print(const char* str) { printf("%s", str); }
    void println(const char* str) { printf("%s\n", str); }
    void print(int val) { printf("%d", val); }
    void println(int val) { printf("%d\n", val); }
    void print(float val, int decimals = 2) { printf("%.*f", decimals, val); }
    void println(float val, int decimals = 2) { printf("%.*f\n", decimals, val); }
    
    template<typename... Args>
    void printf(const char* format, Args... args) {
        ::printf(format, args...);
    }
};

extern MockSerial Serial;

#endif // ARDUINO_MOCK_H
