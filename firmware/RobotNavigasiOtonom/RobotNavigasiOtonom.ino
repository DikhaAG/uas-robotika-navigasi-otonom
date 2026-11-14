#include "Config.h"
#include "Hal.h"
#include "Motors.h"
#include "Gyroscope.h"
#include "UltrasonicSonar.h"
#include "WallDetector.h"
#include "Odometry.h"
#include "WaypointQueue.h"
#include "MotionController.h"
#include "ObstacleAvoidance.h"
#include "CalibrationProfile.h"
#include "AsciiProtocolDecoder.h"
#include "CalibrationService.h"
#include "NavigationFSM.h"

static uint32_t lastLoopMs = 0;

void setup() {
    Hal::init();
    Hal::serialBegin(115200);

    Motors::init();
    Gyroscope::init();
    UltrasonicSonar::init();
    WallDetector::init();
    Odometry::init();
    WaypointQueue::init();
    AsciiProtocolDecoder::init();
    CalibrationService::init();
    NavigationFSM::init();

    Hal::serialPrintln("=== ROBOT NAVIGASI OTONOM (POLSRI TC253421) ===");
    Hal::serialPrintln("Pilih Ruangan ('1'-'4'), lalu tekan START (D12).");
}

void loop() {
    uint32_t now = Hal::millis();

    // Deterministic 20 ms control and odometry loop
    if (now - lastLoopMs >= PERIODE_ODOMETRI_MS) {
        float dt = (float)(now - lastLoopMs) / 1000.0f;
        lastLoopMs = now;

        Odometry::update(dt);
        NavigationFSM::update();
    }
}
