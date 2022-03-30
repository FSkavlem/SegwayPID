#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "PID.h"
#include "JGA25Motor.h"

JGA25Motor rightMotor(8,7,5,50);
JGA25Motor leftMotor(10,9,6,50);

double motorPower;
bool motorClamp = false;
int const abortAngle = 60; //40, hvor mange grader som skal til for at alt stopper

//Steering
double const steeringMulForwardBackward = 4; //recommended 2-4
double const steeringMulLeftRight = 20;
double xSteering,ySteering,powSteering;
int steeringIntTemp;
//bt 
SoftwareSerial BT(3,4); // RX, TX
String BT_input;
//PID VALUES
double currentAngle, desiredAngle;
double const offset = 2; //angle offset
PID pid(5.73,28,0.7,0.01,motorClamp,15);
//BNO IMU
sensors_event_t event;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
//Misc
unsigned long looptime;
bool abortBool = false;
double newton;

//void declaration
void engineSet(double motorPow, double steering);
void safetyCheck(double angle);
double getAngle();
String BluetoothInput();
void BluetoothOutput();
double power2newton(double x);
void MakeSerialPlotterString();
void CalculateXYsteering(String input);

//Main loops
void setup() {
  Serial.begin(115200);                              
  Serial.setTimeout(5);                              
  BT.begin(115200);                                  //38400 for AT, hc05 set to 115200
  BT.setTimeout(5);                                  
  pinMode(LED_BUILTIN, OUTPUT);
  //BNO
  Serial.println("Orientation Sensor Test\n");
  if(!bno.begin()){                                  //Initialise IMU
  Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  while(1);
  } else{
    Serial.print("......Success\n");
  }
  bno.setExtCrystalUse(false);

  pid.printFilterCoeffs();
  delay(500);
  
  looptime = micros(); 
}
void loop(){
  BT_input = BluetoothInput();
  if (BT_input != ""){
    CalculateXYsteering(BT_input);                             //gets value xSteering, ySteering and powSteering
  }
  currentAngle = getAngle();                       
  safetyCheck(abs(currentAngle));                              //if angle is greater than abortAngle, engineSet function can only give 0 
  desiredAngle = (steeringMulForwardBackward*ySteering*powSteering)+offset;
 
  if (abortBool) pid.resetPID();
  motorClamp = (motorPower>255 || motorPower<-255) ? true : false;
  pid.updateMotorClamp(motorClamp);
  motorPower = pid.calculate(desiredAngle,currentAngle); 

  if (!abortBool) engineSet(motorPower, xSteering);
  
  //logging
  //BluetoothOutput();
  //MakeSerialPlotterString();
  newton = power2newton(motorPower);        

  if(micros() - looptime > 10000){digitalWrite(LED_BUILTIN,HIGH);} //if cycle time becomes greater than 10ms, illuminate red light
  while(micros() - looptime < 10000);                              //halts program to maintain 10ms cycle time
  looptime = micros(); 
}
//motor Voids
void engineSet(double motorPow, double steering){
  double motorPowL, motorPowR;
    if (steering > 0){                     
      motorPowR = steeringMulLeftRight*steering*powSteering;;
      motorPowL = 0;
    }
    else{                                 
      motorPowR = 0;
      motorPowL = steeringMulLeftRight*steering*powSteering;
    }
  rightMotor.SetSpeed(motorPow,motorPowR);
  leftMotor.SetSpeed(motorPow,motorPowL);
}
void safetyCheck(double angle){
  if (angle > abortAngle){
    abortBool = true;
    leftMotor.Stop();
    rightMotor.Stop();
  }
  else if (angle < 1){
    abortBool = false;
  }
}
//sensor voids
double getAngle(){
  bno.getEvent(&event);
  return event.orientation.z;
}
//Bluetooth Voids
String BluetoothInput(){
  if (BT.available()>7)
  {
    return BT.readStringUntil('#');
  }
  return "";
} 
void BluetoothOutput(){
BT.print(currentAngle);
BT.print(",");
BT.print(newton);
BT.print(",");
BT.print(motorPower);
BT.print(",");
BT.print(pid.getIntegral());
BT.print(",");
BT.println(pid.getFilteredDerivative());
}
//Support voids
double power2newton(double x){
  x = (x/double(255))*4.6358709; //4.6358709 max nm of engine
  return x;
}
void MakeSerialPlotterString(){
Serial.print(pid.getDerivative());
Serial.print(",");
Serial.println(pid.getFilteredDerivative());
}
//Steering
void CalculateXYsteering(String input){
  //gets angle of joystick and converts to xy coordinates
  if (input=="0000000"){ //checks if joystick is in neutral position
    xSteering = 0;
    ySteering = 0;
    powSteering = 0;
  }
  else{
    String steeringStringTemp = input;
    steeringStringTemp.remove(3,7);
    steeringIntTemp = steeringStringTemp.toInt();
    ySteering = sin((double)steeringIntTemp*PI/180);
    xSteering = cos((double)steeringIntTemp*PI/180);
    //henter hvor mye i retningen xy kontrolleren er
    steeringStringTemp = input;
    steeringStringTemp.remove(0,3);
    steeringIntTemp = steeringStringTemp.toInt();
    powSteering = (double)steeringIntTemp/1000;
  }
}
