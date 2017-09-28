/*
    Logger.h - Library for logging
    Created by Ricardo Alcantara
*/
#ifndef Logger_h
#define Logger_h

#include "Arduino.h"
class Logger {
    public:
        template <typename T>
        void logVariable(char key[], T value);
        void logStatus(char value[]);
        void endChunk();
};

#endif
