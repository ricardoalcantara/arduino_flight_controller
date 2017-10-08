#define ESC_A 2
#define ESC_B 3
#define ESC_C 4
#define ESC_D 5

#define NO_MOTOR false
#define NO_SENSORS false

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000

#define SOFT 0
#define HARD 1
#define FLIGHT_MODE SOFT

#define MPU_addr 0x68

// Lib needed to connect to the motors by their ESCs
#include <Servo.h>

// Lib needed to connect to the MPU 6050
// #include <Wire.h>
// Lib requestFrom
// https://github.com/jarzebski/Arduino-MPU6050
#include "MPU6050.h"

#include "Logger.h"

Servo motorA;
Servo motorB;
Servo motorC;
Servo motorD;

MPU6050 mpu;

Logger logger;

struct error {
  float yaw;
  float pitch;
  float roll;
};

struct PID {
  float yaw;
  float pitch;
  float roll;
};

//MPU6050 address of I2C
const int MPU=0x68;

// //Variaveis para armazenar valores dos sensores
// int AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

// Timers
unsigned long timer = 0, timerPrev = 0;
float elapsedTime;

// long timer, timerPrev;
// float compAngleX, compAngleY;

// Pitch, Roll and Yaw values
float pitchAngle = 0;
float rollAngle = 0;
float yawAngle = 0;

//We create variables for the time width values of each PWM input signal
unsigned long counter_1, counter_2, counter_3, counter_4, counter_5, counter_6, current_count;

//We create 4 variables to stopre the previous value of the input signal (if LOW or HIGH)
byte last_CH1_state, last_CH2_state, last_CH3_state, last_CH4_state, last_CH5_state, last_CH6_state;

//To store the 1000us to 2000us value we create variables and store each channel
int input_YAW;      //In my case channel 4 of the receiver and pin D12 of arduino
int input_PITCH;    //In my case channel 2 of the receiver and pin D9 of arduino
int input_ROLL;     //In my case channel 1 of the receiver and pin D8 of arduino
int input_THROTTLE; //In my case channel 3 of the receiver and pin D10 of arduino

int input_RIGHT;
int input_LEFT;

struct PID Kp = { .yaw = 3.55, .pitch = 0.005, .roll = 2.05};
struct PID Ki = { .yaw = 3.55, .pitch = 0.005, .roll = 2.05};
struct PID Kd = { .yaw = 3.55, .pitch = 0.005, .roll = 2.05};

struct error previous_errors;

int i = 0;

int normalize(float value, int min, int max)
{
  if (value > max) {
      value = max;
  } else if (value < min) {
      value = min;
  }

  return value;
}

void setup() {
  /*
   * Port registers allow for lower-level and faster manipulation of the i/o pins of the microcontroller on an Arduino board. 
   * The chips used on the Arduino board (the ATmega8 and ATmega168) have three ports:
     -B (digital pin 8 to 13)
     -C (analog input pins)
     -D (digital pins 0 to 7)
   
  //All Arduino (Atmega) digital pins are inputs when you begin...
  */  
   
  PCICR |= (1 << PCIE0);    //enable PCMSK0 scan                                                 
  PCMSK0 |= (1 << PCINT0);  //Set pin D8 trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);  //Set pin D9 trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);  //Set pin D10 trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);  //Set pin D11 trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT4);  //Set pin D12 trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT5);  //Set pin D13 trigger an interrupt on state change.
  
  // why 12?
  // PCMSK0 |= (1 << PCINT4);  //Set pin D12 trigger an interrupt on state change.  
  
  //Start the serial in order to see the result on the monitor
  //Remember to select the same baud rate on the serial monitor
  Serial.begin(57600);  
  // Serial.begin(9600);
  
  // input_THROTTLE = 0;
  initMotor();
  initSensors();
  // initRCReceiver();
  
  // Tempo para os motores armarem
  delay(1000);
  timer = millis();  // actual time read  
 
  logger.logStatus("Done....");
  logger.endChunk();
}

void loop() {
  // medir se precisa realocar isso como no exemplo
  // http://www.pitt.edu/~mpd41/Angle.ino
  // timerPrev = timer;  // the previous time is stored before the actual time read

  // https://github.com/jarzebski/Arduino-MPU6050/blob/master/MPU6050_gyro_pitch_roll_yaw/MPU6050_gyro_pitch_roll_yaw.ino
  timerPrev = timer;  // the previous time is stored before the actual time read
  timer = millis();  // actual time read  
  elapsedTime = (timer - timerPrev) / 1000; 
  

  readSensors();
  setMotor();

  logger.endChunk();
}

void initMotor() {
  if (NO_MOTOR) {
    logger.logStatus("No motors");
    return;
  }

  logger.logStatus("Init motor 1");
  motorA.attach(ESC_A, MIN_SIGNAL, MAX_SIGNAL);
  motorA.writeMicroseconds(MIN_SIGNAL);

  logger.logStatus("Init motor 2");
  motorB.attach(ESC_B, MIN_SIGNAL, MAX_SIGNAL);
  motorB.writeMicroseconds(MIN_SIGNAL);

  logger.logStatus("Init motor 3");
  motorC.attach(ESC_C, MIN_SIGNAL, MAX_SIGNAL);
  motorC.writeMicroseconds(MIN_SIGNAL);

  logger.logStatus("Init motor 4");
  motorD.attach(ESC_D, MIN_SIGNAL, MAX_SIGNAL);
  motorD.writeMicroseconds(MIN_SIGNAL);
}

void setMotor() {
  int rxRoll;
  int rxPitch;
  int rxYaw;

  int rxLeft = map(input_LEFT, MIN_SIGNAL, MAX_SIGNAL, 0, 20);
  int rxRight = map(input_RIGHT, MIN_SIGNAL, MAX_SIGNAL, 0, 20);

  if (FLIGHT_MODE == SOFT)
  {
    rxRoll = map(input_ROLL, MIN_SIGNAL, MAX_SIGNAL, -45, 45);
    rxPitch = map(input_PITCH, MIN_SIGNAL, MAX_SIGNAL, -45, 45);   
    rxYaw = map(input_YAW, MIN_SIGNAL, MAX_SIGNAL, -180, 180);
  }
  else
  {
    rxRoll = map(input_ROLL, MIN_SIGNAL, MAX_SIGNAL, -180, 180);
    rxPitch = map(input_PITCH, MIN_SIGNAL, MAX_SIGNAL, -180, 180);
    rxYaw = map(input_YAW, MIN_SIGNAL, MAX_SIGNAL, -270, 270);
  }

  logger.logVariable("rxYaw", rxYaw);
  logger.logVariable("rxPitch", rxPitch);
  logger.logVariable("rxRoll", rxRoll);

  logger.logVariable("rxLeft", rxLeft);
  logger.logVariable("rxRight", rxRight);

  //Adjust difference - MIN_SIGNAL
  int throttle = map(input_THROTTLE, MIN_SIGNAL, MAX_SIGNAL, 0, 1000);
  float cmd_motA = throttle;
  float cmd_motB = throttle;
  float cmd_motC = throttle;
  float cmd_motD = throttle;

  logger.logVariable("throttle", throttle);
  
  //Somatório dos erros
  struct error sError = { .yaw = 0, .pitch = 0, .roll = 0};

  //Valor desejado - angulo atual
  struct error anguloAtual = { .yaw = yawAngle, .pitch = pitchAngle, .roll = rollAngle};
  // struct error anguloAtual = { .yaw = 0, .pitch = compAngleY, .roll = compAngleX};
  struct error anguloDesejado = { .yaw = rxYaw, .pitch = rxPitch, .roll = rxRoll};
  
  struct error deltaError;

  //Variáveis de ajuste
  
  if (throttle > 0) {

    struct error errors = { 
      .yaw = anguloAtual.yaw - anguloDesejado.yaw,
      .pitch = anguloAtual.pitch - anguloDesejado.pitch,
      .roll = anguloAtual.roll - anguloDesejado.roll
    };  
    // Yaw - Lacet (Z axis)
    // int yaw_PID = (errors.yaw * Kp.yaw + sError.yaw * Ki.yaw + deltaError.yaw * Kd.yaw);
    int yaw_P = errors.yaw * Kp.yaw;

    int yaw_I = 0;
    if(-3 < errors.yaw <3)
    {
      // yaw_I = yaw_I+(ki*error);  
      yaw_I = (sError.yaw * Ki.yaw) + deltaError.yaw;
    }

    // pid_d = kd*((error - previous_error)/elapsedTime);
    int yaw_D = Kd.yaw * ((sError.yaw - previous_errors.yaw) / elapsedTime );

    int yaw_PID = yaw_P + yaw_I + yaw_D;
    yaw_PID = normalize(yaw_PID, -1000, 1000);
    cmd_motA -= yaw_PID;
    cmd_motB += yaw_PID;
    cmd_motC += yaw_PID;
    cmd_motD -= yaw_PID;

    // Pitch - Tangage (Y axis)
    // int pitch_PID = (errors.pitch * Kp.pitch + sError.pitch * Ki.pitch + deltaError.pitch * Kd.pitch);
    int pitch_P = errors.pitch * Kp.pitch;
    
    int pitch_I = 0;
    if(-3 < errors.pitch <3)
    {
      // pitch_I = pitch_I+(ki*error);  
      pitch_I = (sError.pitch * Ki.pitch) + deltaError.pitch;
    }

    // pid_d = kd*((error - previous_error)/elapsedTime);
    int pitch_D = Kd.pitch * ((sError.pitch - previous_errors.pitch) / elapsedTime );

    int pitch_PID = pitch_P + pitch_I + pitch_D;    
    pitch_PID = normalize(pitch_PID, -1000, 1000);
    cmd_motA += pitch_PID;
    cmd_motB += pitch_PID;
    cmd_motC -= pitch_PID;
    cmd_motD -= pitch_PID;

    // Roll - Roulis (X axis)
    // int roll_PID = (errors.roll * Kp.roll + sError.roll * Ki.roll + deltaError.roll * Kd.roll);
    int roll_P = errors.roll * Kp.roll;
    
    int roll_I = 0;
    if(-3 < errors.roll <3)
    {
      // roll_I = roll_I+(ki*error);  
      roll_I = (sError.roll * Ki.roll) + deltaError.roll;
    }

    // pid_d = kd*((error - previous_error)/elapsedTime);
    int roll_D = Kd.roll * ((sError.roll - previous_errors.roll) / elapsedTime );

    int roll_PID = roll_P + roll_I + roll_D;
    roll_PID = normalize(roll_PID, -1000, 1000);
    cmd_motA -= roll_PID;
    cmd_motB += roll_PID;
    cmd_motC -= roll_PID;
    cmd_motD += roll_PID;

    previous_errors = errors;
  }

  logger.logVariable("Motor_A", cmd_motA);
  logger.logVariable("Motor_B", cmd_motB);
  logger.logVariable("Motor_C", cmd_motC);
  logger.logVariable("Motor_D", cmd_motD);

  // Apply speed for each motor
  if (NO_MOTOR) {
    logger.logStatus("No motors to set");
    return;
  }
  motorA.writeMicroseconds(normalize(cmd_motA, MIN_SIGNAL, MAX_SIGNAL));
  motorB.writeMicroseconds(normalize(cmd_motB, MIN_SIGNAL, MAX_SIGNAL));
  motorC.writeMicroseconds(normalize(cmd_motC, MIN_SIGNAL, MAX_SIGNAL));
  motorD.writeMicroseconds(normalize(cmd_motD, MIN_SIGNAL, MAX_SIGNAL));
}

/*
void setMotor() {
  int rxRoll;
  int rxPitch;
  int rxYaw;

  int rxLeft = map(input_LEFT, MIN_SIGNAL, MAX_SIGNAL, 0, 20);
  int rxRight = map(input_RIGHT, MIN_SIGNAL, MAX_SIGNAL, 0, 20);

  if (FLIGHT_MODE == SOFT)
  {
    rxRoll = map(input_ROLL, MIN_SIGNAL, MAX_SIGNAL, -45, 45);
    rxPitch = map(input_PITCH, MIN_SIGNAL, MAX_SIGNAL, -45, 45);   
    rxYaw = map(input_YAW, MIN_SIGNAL, MAX_SIGNAL, -180, 180);
  }
  else
  {
    rxRoll = map(input_ROLL, MIN_SIGNAL, MAX_SIGNAL, -180, 180);
    rxPitch = map(input_PITCH, MIN_SIGNAL, MAX_SIGNAL, -180, 180);
    rxYaw = map(input_YAW, MIN_SIGNAL, MAX_SIGNAL, -270, 270);
  }

  logger.logVariable("rxYaw", rxYaw);
  logger.logVariable("rxPitch", rxPitch);
  logger.logVariable("rxRoll", rxRoll);

  logger.logVariable("rxLeft", rxLeft);
  logger.logVariable("rxRight", rxRight);

  //Adjust difference - MIN_SIGNAL
  int throttle = map(input_THROTTLE, MIN_SIGNAL, MAX_SIGNAL, 0, 1000);
  float cmd_motA = throttle;
  float cmd_motB = throttle;
  float cmd_motC = throttle;
  float cmd_motD = throttle;

  logger.logVariable("throttle", throttle);
  
  //Somatório dos erros
  struct error sError = { .yaw = 0, .pitch = 0, .roll = 0};

  //Valor desejado - angulo atual
  struct error anguloAtual = { .yaw = yawAngle, .pitch = pitchAngle, .roll = rollAngle};
  // struct error anguloAtual = { .yaw = 0, .pitch = compAngleY, .roll = compAngleX};
  struct error anguloDesejado = { .yaw = rxYaw, .pitch = rxPitch, .roll = rxRoll};

  struct error errors = { 
      .yaw = anguloAtual.yaw - anguloDesejado.yaw,
      .pitch = anguloAtual.pitch - anguloDesejado.pitch,
      .roll = anguloAtual.roll - anguloDesejado.roll
  };
  
  struct error deltaError;

  //Variáveis de ajuste
  struct PID Kp = { .yaw = 1.5, .pitch = 5, .roll = 5};
  struct PID Ki = { .yaw = 0, .pitch = 0, .roll = 0};
  struct PID Kd = { .yaw = rxLeft, .pitch = rxRight, .roll = rxRight};
  
  if (throttle > 0) {
      // Calculate sum of errors : Integral coefficients
      sError.yaw += errors.yaw;
      sError.pitch += errors.pitch;
      sError.roll += errors.roll;

      // Calculate error delta : Derivative coefficients
      deltaError.yaw = errors.yaw - lastErrors.yaw;
      deltaError.pitch = errors.pitch - lastErrors.pitch;
      deltaError.roll = errors.roll - lastErrors.roll;

      // Save current error as lastErr for next time
      lastErrors.yaw = errors.yaw;
      lastErrors.pitch = errors.pitch;
      lastErrors.roll = errors.roll;

      // Yaw - Lacet (Z axis)
      int yaw_PID = (errors.yaw * Kp.yaw + sError.yaw * Ki.yaw + deltaError.yaw * Kd.yaw);
      cmd_motA -= yaw_PID;
      cmd_motB += yaw_PID;
      cmd_motC += yaw_PID;
      cmd_motD -= yaw_PID;

      // Pitch - Tangage (Y axis)
      int pitch_PID = (errors.pitch * Kp.pitch + sError.pitch * Ki.pitch + deltaError.pitch * Kd.pitch);
      cmd_motA += pitch_PID;
      cmd_motB += pitch_PID;
      cmd_motC -= pitch_PID;
      cmd_motD -= pitch_PID;

      // Roll - Roulis (X axis)
      int roll_PID = (errors.roll * Kp.roll + sError.roll * Ki.roll + deltaError.roll * Kd.roll);
      cmd_motA -= roll_PID;
      cmd_motB += roll_PID;
      cmd_motC -= roll_PID;
      cmd_motD += roll_PID;
  }

  logger.logVariable("Motor_A", cmd_motA);
  logger.logVariable("Motor_B", cmd_motB);
  logger.logVariable("Motor_C", cmd_motC);
  logger.logVariable("Motor_D", cmd_motD);

  // Apply speed for each motor
  if (NO_MOTOR) {
    logger.logStatus("No motors to set");
    return;
  }
  motorA.writeMicroseconds(normalize(cmd_motA));
  motorB.writeMicroseconds(normalize(cmd_motB));
  motorC.writeMicroseconds(normalize(cmd_motC));
  motorD.writeMicroseconds(normalize(cmd_motD));
}
*/

void initSensors() {
  if (NO_SENSORS) {
    logger.logStatus("No sensors");
    return;
  }

  // // Initialize MPU6050
  // while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  // {
  //   logger.logStatus("Could not find a valid MPU6050 sensor, check wiring!");
  //   delay(500);
  // }

  // // Calibrate gyroscope. The calibration must be at rest.
  // // If you don't want calibrate, comment this line.
  // mpu.calibrateGyro();
  
  // // Set threshold sensivty. Default 3.
  // // If you don't want use threshold, comment this line or set 0.
  // mpu.setThreshold(3);

  mpu.begin();
}

int calibrateI = 0;

void readSensors() {
  if (NO_SENSORS) {
    return;
  }

  Vector angles = mpu.readAngle();
  
  if (calibrateI < 200) {
    mpu.calibrateGyro();
    calibrateI++;
  }

  pitchAngle = angles.YAxis;
  rollAngle = angles.XAxis;
  yawAngle = angles.ZAxis;
  
  // Vector norm = mpu.readNormalizeGyro();

  // // Calculate Pitch, Roll and Yaw
  // pitchAngle = pitchAngle + norm.YAxis * 0.01;
  // rollAngle = rollAngle + norm.XAxis * 0.01;
  // yawAngle = yawAngle + norm.ZAxis * 0.01;

  logger.logVariable("yawAngle", yawAngle);
  logger.logVariable("pitchAngle", pitchAngle);
  logger.logVariable("rollAngle", rollAngle);
}

ISR(PCINT0_vect){
  //First we take the current count value in micro seconds using the micros() function
    
  current_count = micros();
  ///////////////////////////////////////Channel 1
  if(PINB & B00000001){                              //We make an AND with the pin state register, We verify if pin 8 is HIGH???
    if(last_CH1_state == 0){                         //If the last state was 0, then we have a state change...
      last_CH1_state = 1;                            //Store the current state into the last state for the next loop
      counter_1 = current_count;                     //Set counter_1 to current value.
    }
  }
  else if(last_CH1_state == 1){                      //If pin 8 is LOW and the last state was HIGH then we have a state change      
    last_CH1_state = 0;                              //Store the current state into the last state for the next loop
    input_ROLL = current_count - counter_1;   //We make the time difference. Channel 1 is current_time - timer_1.
  }
  ///////////////////////////////////////Channel 2
  if(PINB & B00000010 ){                             //pin D9 -- B00000010                                              
    if(last_CH2_state == 0){                                               
      last_CH2_state = 1;                                                   
      counter_2 = current_count;                                             
    }
  }
  else if(last_CH2_state == 1){                                           
    last_CH2_state = 0;                                                     
    input_PITCH = current_count - counter_2;                             
  }
  ///////////////////////////////////////Channel 3
  if(PINB & B00000100 ){                             //pin D10 - B00000100                                         
    if(last_CH3_state == 0){                                             
      last_CH3_state = 1;                                                  
      counter_3 = current_count;                                               
    }
  }
  else if(last_CH3_state == 1){                                             
    last_CH3_state = 0;                                                    
    input_THROTTLE = current_count - counter_3;                            

  }
  ///////////////////////////////////////Channel 4
  if(PINB & B00001000 ){                             //pin D11  -- B00001000
    if(last_CH4_state == 0){                                               
      last_CH4_state = 1;                                                   
      counter_4 = current_count;                                              
    }
  }
  else if(last_CH4_state == 1){                                             
    last_CH4_state = 0;                                                  
    input_YAW = current_count - counter_4;                            
  }
  ///////////////////////////////////////Channel 5
  if(PINB & B00010000 ){                             //pin D12  -- B00010000
    if(last_CH5_state == 0){                                               
      last_CH5_state = 1;                                                   
      counter_5 = current_count;                                              
    }
  }
  else if(last_CH5_state == 1){                                             
    last_CH5_state = 0;                                                  
    input_LEFT = current_count - counter_5;                            
  }
  ///////////////////////////////////////Channel 6
  if(PINB & B00100000 ){                             //pin D13  -- B00100000
    if(last_CH6_state == 0){                                               
      last_CH6_state = 1;                                                   
      counter_6 = current_count;                                              
    }
  }
  else if(last_CH6_state == 1){                                             
    last_CH6_state = 0;                                                  
    input_RIGHT = current_count - counter_6;                            
  }   
}
