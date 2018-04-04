#include <Encoder.h>
#include <EEPROM.h>
#include <Wire.h>

//PIN definitions
#define EndstopPin 8 //Endstop is connected to this pin
#define DIRpin 7     //1 turns counterclockwise, 0 turns clockwise
#define PWMpin 5     //Defines the speed of the motor


//Variable variables
long setPoint; //Requested setPoint
long errorPoint;
int moveSpeed;         //Should be upgraded to PID control
int endstopCheck = 0;  //Should be 0 first time sketch runs
int axisID;
unsigned long encoderMax; 
int maxSpeed;
int minSpeed;
int axisTolerance;
int axisSpeedGain;
int x;
int endstopDir;
int axisStepGain;

//Encoder.h vars
long oldPosition = 0;
long newPosition;

//Start encoder on interrupt pin 0 and 1
Encoder myEnc(2,3);

void setup() {
  Serial.begin(9600);
  
  //Outputs
  pinMode(DIRpin, OUTPUT);
  pinMode(PWMpin, OUTPUT);
  pinMode(EndstopPin, INPUT);
  pinMode(A0, INPUT);    //Analog input for testing
  
  //Axis uC controllers  
  //EEPROM.write(10, 13); 
  axisID = EEPROM.read(10);
  if (axisID == 11) {  //uC for axis 1 (A-1 base Red)
    encoderMax = 48400;
    minSpeed = 50;
    maxSpeed = 255;
    axisTolerance = 10;
    axisSpeedGain = 5;
    endstopDir = 0;
    axisStepGain = 161;  //Total encoder steps divided by degrees
    Serial.println("axidID 1 detected!");
    Serial.print("encoderMax = ");
    Serial.println(encoderMax);
    Serial.print("minSpeed = ");
    Serial.println(minSpeed);
    Serial.print("maxSpeed = ");
    Serial.println(maxSpeed);
    Serial.print("axisTolerance = ");
    Serial.println(axisTolerance);
    Serial.print("axisSpeedGain = 5");
    Serial.println(axisSpeedGain);
    delay(1000);
  }
  else if (axisID == 12) {  //uC for axis 2 (A-2 shoulder Yellow)
    encoderMax = 21450;
    minSpeed = 40;
    maxSpeed = 255;
    endstopDir = 0;
    axisTolerance = 15;
    axisSpeedGain = 5;
    endstopDir = 0;
    axisStepGain = 165;
    Serial.println("axidID 2 detected!");
    Serial.print("encoderMax = ");
    Serial.println(encoderMax);
    Serial.print("minSpeed = ");
    Serial.println(minSpeed);
    Serial.print("maxSpeed = ");
    Serial.println(maxSpeed);
    Serial.print("axisTolerance = ");
    Serial.println(axisTolerance);
    Serial.print("axisSpeedGain = 5");
    Serial.println(axisSpeedGain);
    delay(1000);
  }
  else if (axisID == 13) {  //uC for axis 3 (A-3 elbow Green)
    encoderMax = 15900;
    minSpeed = 80;
    maxSpeed = 255;
    axisTolerance = 20;
    axisSpeedGain = 5;
    endstopDir = 0;
    axisStepGain = 173;
    Serial.println("axidID 3 detected!");
    Serial.print("encoderMax = ");
    Serial.println(encoderMax);
    Serial.print("minSpeed = ");
    Serial.println(minSpeed);
    Serial.print("maxSpeed = ");
    Serial.println(maxSpeed);
    Serial.print("axisTolerance = ");
    Serial.println(axisTolerance);
    Serial.print("axisSpeedGain = 5");
    Serial.println(axisSpeedGain);
    delay(1000);
  }
  Wire.begin(axisID);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent); // register event
}

void loop()  {
  encoderCheck();
}  //LOOP ENDS HERE

void encoderCheck(){
  if (endstopCheck == 0) { //Go to the endstop and reset encoder.
    endstopRun();
    myEnc.write(0);
  }
  else if (endstopCheck == 1) {
    setDirection(axisTolerance);
    newPosition = myEnc.read();
    if (newPosition != oldPosition) {
        oldPosition = newPosition;    
    }
  }
}

void setDirection(int tolerance) {
  errorPoint = setPoint - newPosition;
  
  if (errorPoint > tolerance) {
    setMoveSpeed();
    digitalWrite(DIRpin, 1);
    analogWrite(PWMpin, moveSpeed);
  }
  else if (errorPoint < tolerance*-1) {
    setMoveSpeed();
    digitalWrite(DIRpin, 0);
    analogWrite(PWMpin, moveSpeed);
  } 
  else {
    analogWrite(PWMpin, 0);
  }
}

void setMoveSpeed() {
  if (errorPoint > 0) {
    moveSpeed = constrain(((setPoint-newPosition)/axisSpeedGain), minSpeed, maxSpeed);
  }
  else if (errorPoint < 0) {
    moveSpeed = constrain((((setPoint-newPosition)/axisSpeedGain)*-1), minSpeed, maxSpeed);
  }
}

void endstopRun() {
  if (digitalRead(EndstopPin) == 0) {
    digitalWrite(DIRpin, endstopDir);
    analogWrite(PWMpin, 250);
    Serial.println("Moving to endstop");
  }
  else if (digitalRead(EndstopPin) == 1) {
    analogWrite(PWMpin, 0);
    endstopCheck = 1; 
    Serial.println("Endstop reached, encoders reset");
  }
}

void receiveEvent(int howMany) {
  while (Wire.available() > 0){
    setPoint = (Wire.read()*axisStepGain);    // receive byte as an integer
    setPoint = constrain(setPoint, 0, encoderMax);
  }
}

void requestEvent() {
  //Wire.write(newPosition/axisStepGain);
}

/* UNUSED FUNCTIONS

void serialControl() {
  while (Serial.available() > 0) {
    setPoint = Serial.parseFloat(); 
    if (Serial.read() == '\n') {
      setPoint = constrain(setPoint*axisStepGain, 0, encoderMax);
      Serial.print("Moving to: ");  Serial.print(setPoint); Serial.print(" - "); Serial.println(Serial.parseFloat());
    }
  }
}

void manualControl() {
  setPoint = constrain(map(analogRead(A0), 33, 1012, 0, encoderMax), 0, encoderMax);  //A-1 (base)
}

*/


