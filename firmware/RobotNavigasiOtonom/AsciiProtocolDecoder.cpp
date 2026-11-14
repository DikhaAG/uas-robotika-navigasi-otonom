#include "AsciiProtocolDecoder.h"
#include <stdio.h>

char AsciiProtocolDecoder::rxBuffer[96];
size_t AsciiProtocolDecoder::rxIndex = 0;
bool AsciiProtocolDecoder::inFrame = false;
AsciiProtocolDecoder::RoomSelectHandler AsciiProtocolDecoder::activeRoomHandler = NULL;

void AsciiProtocolDecoder::reset() {
    rxIndex = 0;
    inFrame = false;
    rxBuffer[0] = '\0';
}

bool AsciiProtocolDecoder::isInFrame() {
    return inFrame;
}

void AsciiProtocolDecoder::init(RoomSelectHandler roomHandler) {
    activeRoomHandler = roomHandler;
    reset();
}

bool AsciiProtocolDecoder::feedChar(char c, char* frameOut, size_t maxLen) {
    if (!inFrame) {
        if (c == '!') {
            inFrame = true;
            rxIndex = 0;
            rxBuffer[rxIndex++] = c;
            rxBuffer[rxIndex] = '\0';
            return false;
        } else if (c >= '1' && c <= '4') {
            if (activeRoomHandler != NULL) {
                activeRoomHandler((uint8_t)(c - '0'));
            }
            return false;
        }
        return false;
    }

    // Inside frame
    if (c == '\n') {
        rxBuffer[rxIndex] = '\0';
        if (frameOut != NULL && maxLen > 0) {
            snprintf(frameOut, maxLen, "%s", rxBuffer);
        }
        reset();
        return true;
    }

    if (c == '\r') {
        // Strip carriage return
        return false;
    }

    if (rxIndex < sizeof(rxBuffer) - 1) {
        rxBuffer[rxIndex++] = c;
        rxBuffer[rxIndex] = '\0';
    } else {
        // Buffer overflow protection: drop corrupt frame
        reset();
    }

    return false;
}
