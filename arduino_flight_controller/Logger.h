/*
    Logger.h - Library for logging
    Created by Ricardo Alcantara
*/
#ifndef Logger_h
#define Logger_h

#include "Arduino.h"

class Logger {
    public:
        void logVariable(char key[], char value[]);
        void logVariable(char key[], int value);
        void logVariable(char key[], long value);
        void logVariable(char key[], float value);
        void logStatus(char value[]);
        void endChunk();
};

#endif