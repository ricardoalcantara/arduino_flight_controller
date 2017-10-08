/*
    MPU6050.cpp - Library for MPU6050
    Created by Ricardo Alcantara
*/

#include <Arduino.h>
#include <Wire.h>

#include "MPU6050.h"

MPU6050::MPU6050()
{
    this->angles.XAxis = 0;
    this->angles.YAxis = 0;
    this->angles.ZAxis = 0;

    this->anglesError.XAxis = 0;
    this->anglesError.YAxis = 0;
    this->anglesError.ZAxis = 0;
}

void MPU6050::begin(int mpua)
{
    // Set up MPU 6050:

    this->mpua = mpua;
    Wire.begin();
#if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

    Wire.beginTransmission(this->mpua);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0);    // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    delay(100);

    //setup starting angle
    //1) collect the data
    Wire.beginTransmission(this->mpua);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(this->mpua, 14, true); // request a total of 14 registers
    AcX = Wire.read() << 8 | Wire.read();   // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY = Wire.read() << 8 | Wire.read();   // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read() << 8 | Wire.read();   // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp = Wire.read() << 8 | Wire.read();   // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX = Wire.read() << 8 | Wire.read();   // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY = Wire.read() << 8 | Wire.read();   // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = Wire.read() << 8 | Wire.read();   // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    //2) calculate pitch and roll
    double roll = atan2(AcY, AcZ) * RAD_TO_DEG;
    double pitch = atan2(-AcX, AcZ) * RAD_TO_DEG;

    //3) set the starting angle to this pitch and roll
    double gyroXangle = roll;
    double gyroYangle = pitch;
    this->anglesRaw.XAxis = roll;
    this->anglesRaw.YAxis = pitch;
    this->anglesRaw.ZAxis = 0;

    timer = micros();
}

void MPU6050::calibrateGyro()
{
    this->readAngle();
    this->anglesError.XAxis = this->anglesRaw.XAxis;
    this->anglesError.YAxis = this->anglesRaw.YAxis;
    this->anglesError.ZAxis = this->anglesRaw.ZAxis;
}

Vector MPU6050::readAngle()
{
    //Now begins the main loop.
    //Collect raw data from the sensor.
    Wire.beginTransmission(this->mpua);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(this->mpua, 14, true); // request a total of 14 registers
    AcX = Wire.read() << 8 | Wire.read();   // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY = Wire.read() << 8 | Wire.read();   // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read() << 8 | Wire.read();   // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp = Wire.read() << 8 | Wire.read();   // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX = Wire.read() << 8 | Wire.read();   // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY = Wire.read() << 8 | Wire.read();   // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = Wire.read() << 8 | Wire.read();   // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    double dt = (double)(micros() - timer) / 1000000; //This line does three things: 1) stops the timer, 2)converts the timer's output to seconds from microseconds, 3)casts the value as a double saved to "dt".
    timer = micros();                                 //start the timer again so that we can calculate the next dt.

    //the next two lines calculate the orientation of the accelerometer relative to the earth and convert the output of atan2 from radians to degrees
    //We will use this data to correct any cumulative errors in the orientation that the gyroscope develops.
    double roll = atan2(AcY, AcZ) * RAD_TO_DEG;
    double pitch = atan2(-AcX, AcZ) * RAD_TO_DEG;

    //The gyroscope outputs angular velocities.  To convert these velocities from the raw data to deg/second, divide by 131.
    //Notice, we're dividing by a double "131.0" instead of the int 131.
    double gyroXrate = GyX / 131.0;
    double gyroYrate = GyY / 131.0;

    //THE COMPLEMENTARY FILTER
    //This filter calculates the angle based MOSTLY on integrating the angular velocity to an angular displacement.
    //dt, recall, is the time between gathering data from the MPU6050.  We'll pretend that the
    //angular velocity has remained constant over the time dt, and multiply angular velocity by
    //time to get displacement.
    //The filter then adds a small correcting factor from the accelerometer ("roll" or "pitch"), so the gyroscope knows which way is down.
    this->anglesRaw.XAxis = (0.99 * (this->anglesRaw.XAxis + gyroXrate * dt) + 0.01 * roll); // Calculate the angle using a Complimentary filter
    this->anglesRaw.YAxis = (0.99 * (this->anglesRaw.YAxis + gyroYrate * dt) + 0.01 * pitch);

    this->angles.XAxis = this->anglesRaw.XAxis - this->anglesError.XAxis;
    this->angles.YAxis = this->anglesRaw.YAxis - this->anglesError.YAxis;

    return this->angles;
}