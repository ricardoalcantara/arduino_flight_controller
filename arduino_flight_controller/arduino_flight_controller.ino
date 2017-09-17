#define ESC1 2
#define ESC2 4
#define ESC3 7
#define ESC4 8

#define NO_MOTOR true

#define CALIBRATE_MODE false
// Debug
#define DEBUG true

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000

// Lib needed to connect to the motors by their ESCs
#include <Servo.h>

// Lib needed to connect to the MPU 6050
#include<Wire.h>

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

//MPU6050 address of I2C
const int MPU=0x68;

//Variaveis para armazenar valores dos sensores
int AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

void setup() {
  Serial.begin(9600);
  initMotor();

  if (DEBUG) {
    Serial.println("Debuging...");
  }

  if (!NO_MOTOR) {
    if (CALIBRATE_MODE) {
      setMotor(MAX_SIGNAL);
      
      Serial.println("Turn on power source, then Wait 2 seconds and press any key");
      while(!Serial.available());
      Serial.read();  
    }
    
    setMotor(MIN_SIGNAL);
  }

  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  
  //Inicializa o MPU-6050
  Wire.write(0); 
  Wire.endTransmission(true);

  delay(5000);
  Serial.println("Done....");
}

void loop() {

  readSensors();
  
  // send data only when you receive data:
  if (Serial.available() > 0) {
    //Le o valor do potenciometro
    int valor = Serial.parseInt();
    
    Serial.print("Input: ");
    Serial.println(valor);
    
    //Envia o valor para o motor
    setMotor(valor);
  }
}

void initMotor() {
  if (NO_MOTOR) {
    Serial.println("No motors");
    return;
  }

  int initValue = 0;

  Serial.println("Init motor 1");
  motor1.attach(ESC1);
  // motor1.write(MIN_SIGNAL);

  Serial.println("Init motor 2");
  motor2.attach(ESC2);
  // motor2.write(MIN_SIGNAL);

  Serial.println("Init motor 3");
  motor3.attach(ESC3);
  // motor3.write(MIN_SIGNAL);

  Serial.println("Init motor 4");
  motor4.attach(ESC4);
  motor4.write(MIN_SIGNAL);
}

void setMotor(int value) {

  if (NO_MOTOR || value == -1) {
    Serial.println("No motors to set");
    return;
  }

  if (value >= 0 && value <= 179) {
    value = map(value, 0, 179, MIN_SIGNAL, MAX_SIGNAL);
  } else if (value < MIN_SIGNAL || value > MAX_SIGNAL) {
    Serial.println("Invalid signal");
    return;
  }

  // Serial.print("Set motor 1: ");
  // Serial.println(value);
  motor1.writeMicroseconds(value);
  // Serial.print("Set motor 2: ");
  // Serial.println(value);
  motor2.writeMicroseconds(value);
  // Serial.print("Set motor 3: ");
  // Serial.println(value);
  motor3.writeMicroseconds(value);
  // Serial.print("Set motor 4: ");
  // Serial.println(value);
  motor4.writeMicroseconds(value);
}

void readSensors() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  //Solicita os dados do sensor
  Wire.requestFrom(MPU,14,true);  
  //Armazena o valor dos sensores nas variaveis correspondentes
  AcX=Wire.read()<<8|Wire.read();  //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //Envia valor da temperatura para a serial e o LCD
  //Calcula a temperatura em graus Celsius
  Serial.print("Tmp = "); Serial.print(Tmp/340.00+36.53);
  //Envia valor X do acelerometro para a serial e o LCD
  Serial.print(" | AcX = "); Serial.print(AcX);
  //Envia valor Y do acelerometro para a serial e o LCD
  Serial.print(" | AcY = "); Serial.print(AcY);
  //Envia valor Z do acelerometro para a serial e o LCD
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  //Envia valor X do giroscopio para a serial e o LCD
  Serial.print(" | GyX = "); Serial.print(GyX);
  //Envia valor Y do giroscopio para a serial e o LCD  
  Serial.print(" | GyY = "); Serial.print(GyY);
  //Envia valor Z do giroscopio para a serial e o LCD
  Serial.print(" | GyZ = "); Serial.println(GyZ);

  if (DEBUG && NO_MOTOR) {
    //Aguarda 300 ms e reinicia o processo
    delay(300);
  }
}

