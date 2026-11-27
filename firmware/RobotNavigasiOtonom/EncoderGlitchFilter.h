#ifndef ENCODER_GLITCH_FILTER_H
#define ENCODER_GLITCH_FILTER_H

#include "Config.h"
#include <stdint.h>

// Counts encoder slots while rejecting motor-PWM noise.
//
// A plain "ignore edges for N ms" debounce cannot reject PWM noise: the
// spikes repeat every 1-2 ms, so every Nth spike still gets counted. What
// separates noise from a slot is how long a level LASTS: spikes last
// microseconds, a slot holds HIGH/LOW for milliseconds. So the filter only
// accepts a level change once the new level has persisted for
// ENCODER_MIN_LEVEL_US, and counts one slot per accepted LOW -> HIGH.
//
// Feed it from a CHANGE interrupt. A level's duration is known only at the
// edge that ends it, so a slot is counted when its HIGH level ends (half a
// slot later than a RISING counter); the count is identical once moving.
struct EncoderGlitchFilter {
    volatile uint8_t rawLevel;      // pin level since the most recent edge
    volatile uint8_t stableLevel;   // last level that persisted long enough
    volatile uint32_t lastEdgeUs;   // time of the most recent edge

    void reset(uint8_t level, uint32_t nowUs) {
        rawLevel = level ? 1 : 0;
        stableLevel = rawLevel;
        lastEdgeUs = nowUs;
    }

    // Call on every edge with the pin level read after it. Returns true when
    // this edge confirms a new slot.
    bool onEdge(uint8_t level, uint32_t nowUs) {
        bool slotConfirmed = false;
        if (nowUs - lastEdgeUs >= ENCODER_MIN_LEVEL_US && rawLevel != stableLevel) {
            stableLevel = rawLevel;
            slotConfirmed = (stableLevel == 1);
        }
        rawLevel = level ? 1 : 0;
        lastEdgeUs = nowUs;
        return slotConfirmed;
    }
};

#endif // ENCODER_GLITCH_FILTER_H
