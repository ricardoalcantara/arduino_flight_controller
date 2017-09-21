#define ESC_A 2
#define ESC_B 3
#define ESC_C 4
#define ESC_D 5

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000

// Lib needed to connect to the motors by their ESCs
#include <Servo.h>

Servo motorA;
Servo motorB;
Servo motorC;
Servo motorD;

void setup() {
  Serial.begin(19200);  
  
  // put your setup code here, to run once:
  initMotor();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void initMotor() {
  Serial.println("Init motor 1");
  motorA.attach(ESC_A, MIN_SIGNAL, MAX_SIGNAL);
  

  Serial.println("Init motor 2");
  motorB.attach(ESC_B, MIN_SIGNAL, MAX_SIGNAL);
  

  Serial.println("Init motor 3");
  motorC.attach(ESC_C, MIN_SIGNAL, MAX_SIGNAL);
  

  Serial.println("Init motor 4");
  motorD.attach(ESC_D, MIN_SIGNAL, MAX_SIGNAL);
  

  motorA.writeMicroseconds(MAX_SIGNAL);
  motorB.writeMicroseconds(MAX_SIGNAL);
  motorC.writeMicroseconds(MAX_SIGNAL);
  motorD.writeMicroseconds(MAX_SIGNAL);
  
  Serial.println("Turn on power source, then Wait 2 seconds and press any key");
  while(!Serial.available());

  motorA.writeMicroseconds(MIN_SIGNAL);
  motorB.writeMicroseconds(MIN_SIGNAL);
  motorC.writeMicroseconds(MIN_SIGNAL);
  motorD.writeMicroseconds(MIN_SIGNAL);
  
  // setMotor();
}
