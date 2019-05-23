#include <I2Cdev.h>
#include <MPU6050.h>
#include <MsTimer2.h>
#include <BalanbotMotor.h>
#include <BalanbotController.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Kalman.h>

//------------------------------------
//---------- For controller ----------
//------------------------------------
void PID_Control();
double getPhi();
float error=0, feedback=0;

float kp = 18;
float kd = 0.06;
float ki = 108;
float reference = -2;

float sum_error = 0;
float diff_error = 0;
float pre_error = 0;

const float K = 2;

const int error_list_size = 10;
float error_list[error_list_size];

int motor_speed = 0;
//------------------------------------
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

Kalman kalmanX; // Create the Kalman instances

const double offset = 0;
double accX, accY, accZ;
double gyroX;

double kalAngleX; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data


SoftwareSerial BT(12,13);

String cmd = "", data = "";   
bool startRecieve = false;  

BalanbotMotor motor1;
BalanbotMotor motor2;
BalanbotEncoder encoder1;
BalanbotEncoder encoder2;
PIDController C;

float dT = 0.01;

//-------------------------------------------
//--------------- Interrupt -----------------
//-------------------------------------------


void timerInterrupt(){
    sei();
    PID_Control();
    motor1.Update();
    //Serial.println(getPhi());
    sendState();
}

void encoder1Interrupt(){
    encoder1.Update();
}

void encoder2Interrupt(){
    encoder2.Update();
}

//------------------------------------------------------
//-------------- Motor and MPU6050setup ----------------
//------------------------------------------------------
void setupMPU6050()
{
    Wire.begin();
    TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

    i2cData[0] = 7;    // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
    while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
    while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

    while (i2cRead(0x75, i2cData, 1));
    if (i2cData[0] != 0x68)
    { // Read "WHO_AM_I" register
        Serial.print(F("Error reading sensor"));
        while (1);
    }

    delay(100); // Wait for sensor to stabilize

    /* Set kalman and gyro starting angle */
    while (i2cRead(0x3B, i2cData, 6));
    accX = (i2cData[0] << 8) | i2cData[1];
    accY = (i2cData[2] << 8) | i2cData[3];
    accZ = (i2cData[4] << 8) | i2cData[5];

    double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    kalmanX.setAngle(roll); // Set starting angle
    timer = micros();
}

void setupMotor(){
    int PWMA = 5, PWMB = 9;               // Speed control 
    int AIN1 = 6, BIN1 = 11;              // Direction +
    int AIN2 = 4, BIN2 = 10;              // Direction -
    int STBY = 7;                         // standby(停止)
    motor1.SetMotorPins(PWMA,AIN1,AIN2,STBY);
    motor2.SetMotorPins(PWMB,BIN1,BIN2,STBY);
}

void setupEncoder(){
    int C1_A = A3 , C2_A = 2;
    int C1_B = 8 , C2_B = 3;
    encoder1.SetInterruptPin(C2_A);
    encoder2.SetInterruptPin(C2_B);
    encoder1.SetDirectionPin(C1_A);
    encoder2.SetDirectionPin(C1_B);
    attachInterrupt(digitalPinToInterrupt(encoder1.GetInterruptPin())
                    ,encoder1Interrupt,RISING);
    attachInterrupt(digitalPinToInterrupt(encoder2.GetInterruptPin())
                    ,encoder2Interrupt,RISING);
}

void setupController(){
  C.SetPID(kp, ki, kd);
  C.SetReference(reference);
}
void setup(){

    //---Initialize list for controller---
    for(int i = 0; i < error_list_size; i++)
      error_list[i] = 0.0;
    //------------------------------------
    
    Serial.begin(57600);   
    BT.begin(57600); 
    setupMotor();
    setupEncoder();
    setupController();
    setupMPU6050();
    MsTimer2::set(dT*1000, timerInterrupt);
    MsTimer2::start();
}

double getPhi()
{
    /* Update all the values */
    while (i2cRead(0x3B, i2cData, 14));
    accX = ((i2cData[0] << 8) | i2cData[1]);
    accY = ((i2cData[2] << 8) | i2cData[3]);
    accZ = ((i2cData[4] << 8) | i2cData[5]);
    gyroX = (i2cData[8] << 8) | i2cData[9];

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

    double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    return (kalAngleX + offset);
}
/*void PID_Control()
{
  error = float(getPhi());

  //stop if phi is too big
  if(abs(error) > 70){

    while(true){
      motor1.InverseRotationDirectionDefinition(true);
      motor2.InverseRotationDirectionDefinition(true);

      motor1.Rotate(0);
      motor2.Rotate(0);
    }
  }
    
  for(int i = 0; i < error_list_size-1; i++)
    error_list[i] = error_list[i+1];
  error_list[error_list_size-1] = error;

  //calculate ki term
  sum_error = 0;
  for(int i = 0; i < error_list_size; i++)
    sum_error += error_list[i];

  //calculate kd term
  diff_error = error - pre_error;
  pre_error = error;

  if(error != 0)
    feedback =K*( kp*error + ki*sum_error + kd*diff_error );
  else
    feedback = 0;
  if(feedback > 255.0)
    feedback = 255.0;
  if(feedback < -255.0) 
    feedback = -255.0;

  motor_speed = int(feedback);
  if(motor_speed < 0){
    motor1.InverseRotationDirectionDefinition(false);
    motor2.InverseRotationDirectionDefinition(false);
            
    motor1.Rotate(abs(motor_speed)); 
    motor2.Rotate(abs(motor_speed));    
  }
  else{
    motor1.InverseRotationDirectionDefinition(true);
    motor2.InverseRotationDirectionDefinition(true);
            
    motor1.Rotate(abs(motor_speed)); 
    motor2.Rotate(abs(motor_speed));     
  }
  
  
}*/

void PID_Control()
{
  error = getPhi();
  //stop if phi is too big
  if(abs(error) > 70){

    while(true){
      motor1.InverseRotationDirectionDefinition(true);
      motor2.InverseRotationDirectionDefinition(true);

      motor1.Rotate(0);
      motor2.Rotate(0);
    }
  }

  feedback = C.Update(error);
  
  if(feedback > 255.0)
    feedback = 255.0;
  if(feedback < -255.0) 
    feedback = -255.0;

  motor_speed = int(feedback);
  if(motor_speed < 0){
    motor1.InverseRotationDirectionDefinition(true);
    motor2.InverseRotationDirectionDefinition(true);
            
    motor1.Rotate(abs(motor_speed)); 
    motor2.Rotate(abs(motor_speed));    
  }
  else{
    motor1.InverseRotationDirectionDefinition(false);
    motor2.InverseRotationDirectionDefinition(false);
            
    motor1.Rotate(abs(motor_speed)); 
    motor2.Rotate(abs(motor_speed));     
  }
  
  
}
void sendState()
{
  data += String(getPhi());
  data += '~';
  data += String(motor1.GetAngle());
  data += '|';
  BT.print(data);
  Serial.println(data);
  data = "";
    
}
void parseCmd()
{
  int first_commaAt = -1, second_commaAt = -1;
  int cmdLen = cmd.length();
  for(int i = 0; i<cmdLen; ++i){
    if(cmd[i] ==','){
      first_commaAt = i;
      break;
    }
  }
  for(int i = first_commaAt+1; i<cmdLen; ++i){
    if(cmd[i] ==','){
      second_commaAt = i;
      break;
    }
  }

  if(first_commaAt != -1 && second_commaAt != -1){
    kp = cmd.substring(0, first_commaAt).toFloat();
    ki = cmd.substring(first_commaAt+1, second_commaAt).toFloat();
    kd = cmd.substring(second_commaAt+1).toFloat();
    C.SetPID(kp, ki, kd);
    
  }
  cmd = "";
}

void updateBT()
{
  if(BT.available()){
    char c = BT.read();
    bool isEnd = (c == '#')?true:false;
    if(isEnd){
      parseCmd();
    }else{
      cmd += String(c);
    }
  }
}

void loop(){
    updateBT();
}
