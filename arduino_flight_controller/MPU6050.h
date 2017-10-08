/*
    MPU6050.h - Library for MPU6050
    Created by Ricardo Alcantara
*/

#define MPU6050_ADDRESS 0x68

#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H
struct Vector
{
    float XAxis;
    float YAxis;
    float ZAxis;
};
#endif

#ifndef MPU6050_h
#define MPU6050_h

#include "Arduino.h"

class MPU6050 {    
    public:
        MPU6050();
        void begin(int mpua = MPU6050_ADDRESS);
        void calibrateGyro();
        Vector readAngle();
    private:
        long timer;
        double AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //These will be the raw data from the MPU6050.
        int mpua;
        Vector anglesRaw;
        Vector angles;
        Vector anglesError;
};

#endif
