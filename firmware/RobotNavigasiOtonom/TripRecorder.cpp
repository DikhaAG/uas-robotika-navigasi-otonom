#include "TripRecorder.h"
#include "Hal.h"
#include "Odometry.h"

// EEPROM record: magic, left pulses (4 bytes LE), right pulses (4 bytes LE), checksum
static const uint8_t TRIP_MAGIC = 0x7A;
static const uint8_t TRIP_RECORD_SIZE = 10;

long TripRecorder::tripLeft = 0;
long TripRecorder::tripRight = 0;
long TripRecorder::lastLeft = 0;
long TripRecorder::lastRight = 0;
uint32_t TripRecorder::lastMotionMs = 0;
bool TripRecorder::unsaved = false;

static uint8_t recordChecksum(const uint8_t* bytes, uint8_t count) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < count; ++i) {
        sum = (uint8_t)(sum + bytes[i]);
    }
    return (uint8_t)~sum;
}

static void putLong(uint8_t* out, long value) {
    uint32_t v = (uint32_t)value;
    for (uint8_t i = 0; i < 4; ++i) {
        out[i] = (uint8_t)(v >> (8 * i));
    }
}

static long getLong(const uint8_t* in) {
    uint32_t v = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        v |= (uint32_t)in[i] << (8 * i);
    }
    return (long)v;
}

// Pulses added since the previous reading; a smaller reading means the
// counter was reset (e.g. by a !DIAG or !STEP command) and restarted from 0.
static long pulsesSince(long previous, long current) {
    return (current >= previous) ? (current - previous) : current;
}

void TripRecorder::init() {
    tripLeft = 0;
    tripRight = 0;
    lastLeft = Odometry::getPulsaKiri();
    lastRight = Odometry::getPulsaKanan();
    lastMotionMs = Hal::millis();
    unsaved = false;
}

void TripRecorder::update(uint32_t nowMs) {
    long curLeft = Odometry::getPulsaKiri();
    long curRight = Odometry::getPulsaKanan();
    long dLeft = pulsesSince(lastLeft, curLeft);
    long dRight = pulsesSince(lastRight, curRight);
    lastLeft = curLeft;
    lastRight = curRight;

    if (dLeft != 0 || dRight != 0) {
        tripLeft += dLeft;
        tripRight += dRight;
        lastMotionMs = nowMs;
        unsaved = true;
    } else if (unsaved && nowMs - lastMotionMs >= TRIP_SAVE_IDLE_MS) {
        if (tripLeft >= TRIP_MIN_SAVE_PULSES || tripRight >= TRIP_MIN_SAVE_PULSES) {
            save();
        }
        unsaved = false;
    }
}

void TripRecorder::save() {
    uint8_t record[TRIP_RECORD_SIZE];
    record[0] = TRIP_MAGIC;
    putLong(&record[1], tripLeft);
    putLong(&record[5], tripRight);
    record[9] = recordChecksum(record, TRIP_RECORD_SIZE - 1);
    for (uint8_t i = 0; i < TRIP_RECORD_SIZE; ++i) {
        Hal::eepromUpdate(TRIP_EEPROM_START_ADDR + i, record[i]);
    }
}

bool TripRecorder::readSaved(long& pulsesLeft, long& pulsesRight) {
    uint8_t record[TRIP_RECORD_SIZE];
    for (uint8_t i = 0; i < TRIP_RECORD_SIZE; ++i) {
        record[i] = Hal::eepromRead(TRIP_EEPROM_START_ADDR + i);
    }
    if (record[0] != TRIP_MAGIC || record[9] != recordChecksum(record, TRIP_RECORD_SIZE - 1)) {
        return false;
    }
    pulsesLeft = getLong(&record[1]);
    pulsesRight = getLong(&record[5]);
    return true;
}
