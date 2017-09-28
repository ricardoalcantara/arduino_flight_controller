/*
    Logger.cpp - Library for Logging
    Created by Ricardo Alcantara
*/

#include <Arduino.h>

#include "Logger.h"

template <typename T>
void Logger::logVariable(char key[], T value) {
    Serial.print("|");
    Serial.print(key);
    Serial.print("=");
    Serial.print(value);
}

void Logger::logStatus(char value[]) {
    Serial.print("|status=");
    Serial.print(value);
}

void Logger::endChunk() {
    Serial.println();    
}
