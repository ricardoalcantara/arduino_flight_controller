/*
    Logger.cpp - Library for Logging
    Created by Ricardo Alcantara
*/

#include <Arduino.h>

#include "Logger.h"

void Logger::logVariable(char key[], char value[]) {
    Serial.print("|");
    Serial.print(key);
    Serial.print("=");
    Serial.print(value);
}

void Logger::logVariable(char key[], int value) {
    Serial.print("|");
    Serial.print(key);
    Serial.print("=");
    Serial.print(value);
}

void Logger::logVariable(char key[], long value) {
    Serial.print("|");
    Serial.print(key);
    Serial.print("=");
    Serial.print(value);
}

void Logger::logVariable(char key[], float value) {
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