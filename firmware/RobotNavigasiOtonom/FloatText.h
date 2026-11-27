#ifndef FLOAT_TEXT_H
#define FLOAT_TEXT_H

// Float <-> text conversion that works on the Arduino Uno.
//
// The AVR libc linked by the Arduino IDE ships printf/scanf WITHOUT float
// support: snprintf with a %.2f conversion prints "?" and sscanf with %f matches nothing.
// The desktop test build uses a full libc, so tests cannot catch this.
// Never use a float conversion in a printf/scanf format in the firmware
// (enforced by `make lint`); format and parse floats through these helpers.

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

namespace FloatText {

// Big enough for any format() output: "-2000000.000" plus the terminator.
static const size_t BUFFER_SIZE = 16;

// Writes `value` with `decimals` (0-3) fractional digits, rounded half away
// from zero, e.g. 25.4f/2 -> "25.40". Values too large for the fixed-point
// range become "inf"/"-inf" and NaN becomes "nan", which Python's float()
// still parses. Returns `buf` so it can be used inline as a "%s" argument.
inline const char* format(char* buf, size_t size, float value, uint8_t decimals) {
    static const long SCALE[] = {1L, 10L, 100L, 1000L};
    if (decimals > 3) {
        decimals = 3;
    }
    if (isnan(value)) {
        snprintf(buf, size, "nan");
        return buf;
    }

    float scaled = value * (float)SCALE[decimals];
    if (fabsf(scaled) >= 2.0e9f) {
        snprintf(buf, size, (value < 0.0f) ? "-inf" : "inf");
        return buf;
    }

    long rounded = (long)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
    const char* sign = (rounded < 0) ? "-" : "";
    unsigned long magnitude = (unsigned long)((rounded < 0) ? -rounded : rounded);
    unsigned long whole = magnitude / (unsigned long)SCALE[decimals];
    unsigned long frac = magnitude % (unsigned long)SCALE[decimals];

    switch (decimals) {
        case 0:  snprintf(buf, size, "%s%lu", sign, whole); break;
        case 1:  snprintf(buf, size, "%s%lu.%01lu", sign, whole, frac); break;
        case 2:  snprintf(buf, size, "%s%lu.%02lu", sign, whole, frac); break;
        default: snprintf(buf, size, "%s%lu.%03lu", sign, whole, frac); break;
    }
    return buf;
}

// Parses one number at `cursor` that must be followed immediately by
// `terminator` (':' between fields, '\0' for the last one). On success stores
// it in `out`, moves `cursor` past the terminator and returns true; on failure
// leaves `cursor` and `out` untouched.
inline bool parseFloatField(const char*& cursor, char terminator, float& out) {
    char* end = NULL;
    double value = strtod(cursor, &end);
    if (end == cursor || *end != terminator) {
        return false;
    }
    out = (float)value;
    cursor = (terminator == '\0') ? end : end + 1;
    return true;
}

// Integer counterpart of parseFloatField(); rejects "2.5" and other non-integers.
inline bool parseLongField(const char*& cursor, char terminator, long& out) {
    char* end = NULL;
    long value = strtol(cursor, &end, 10);
    if (end == cursor || *end != terminator) {
        return false;
    }
    out = value;
    cursor = (terminator == '\0') ? end : end + 1;
    return true;
}

} // namespace FloatText

#endif // FLOAT_TEXT_H
