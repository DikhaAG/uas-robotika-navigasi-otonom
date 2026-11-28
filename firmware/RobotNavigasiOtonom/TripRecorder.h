#ifndef TRIP_RECORDER_H
#define TRIP_RECORDER_H

#include <stdint.h>

// Untethered distance measurement for wheel-diameter calibration.
//
// Opening the USB serial port resets the Uno (DTR auto-reset), which wipes
// the RAM pulse counters. TripRecorder adds up the encoder pulses made while
// the robot is IDLE since power-on and, once the wheels have been still for
// TRIP_SAVE_IDLE_MS, stores the running total in EEPROM. So the robot can be
// pushed on battery power with USB unplugged, and the total read afterwards
// with !TRIP:READ. Trips shorter than TRIP_MIN_SAVE_PULSES are never saved, so
// nudging the robot after a reboot (plugging the cable in, tidying up) cannot
// overwrite a measured push. Pauses shorter than TRIP_SAVE_IDLE_MS just continue the
// trip; a longer pause saves the total so far and later motion extends it.
// A new trip starts at the next power-on/reset.

#define TRIP_EEPROM_START_ADDR 64     // after the CalibrationProfile record at 0
#define TRIP_SAVE_IDLE_MS      1000UL
#define TRIP_MIN_SAVE_PULSES   20L    // ~20 cm with 65 mm wheels and 20 slots

class TripRecorder {
public:
    // Call after Odometry::init(): starts a new, empty trip.
    static void init();
    // Call every control tick while IDLE.
    static void update(uint32_t nowMs);
    // Last saved trip; false if none was saved or the record is corrupt.
    static bool readSaved(long& pulsesLeft, long& pulsesRight);

private:
    static void save();

    static long tripLeft;
    static long tripRight;
    static long lastLeft;
    static long lastRight;
    static uint32_t lastMotionMs;
    static bool unsaved;
};

#endif // TRIP_RECORDER_H
