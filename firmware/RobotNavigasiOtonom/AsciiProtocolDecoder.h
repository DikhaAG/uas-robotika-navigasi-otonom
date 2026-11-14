#ifndef ASCII_PROTOCOL_DECODER_H
#define ASCII_PROTOCOL_DECODER_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

class AsciiProtocolDecoder {
public:
    typedef void (*RoomSelectHandler)(uint8_t room);

    static void init(RoomSelectHandler roomHandler = NULL);
    static void reset();
    static bool feedChar(char c, char* frameOut, size_t maxLen);
    static bool isInFrame();

private:
    static char rxBuffer[96];
    static size_t rxIndex;
    static bool inFrame;
    static RoomSelectHandler activeRoomHandler;
};

#endif // ASCII_PROTOCOL_DECODER_H
