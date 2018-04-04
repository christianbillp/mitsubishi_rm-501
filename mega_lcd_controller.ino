//i2c master (mega)
#include <Wire.h>
#include <LiquidCrystal.h>
#include <math.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// define some values used by the panel and buttons
int lcd_key     = 0;
int adc_key_in  = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

//Constants (robot dimensions in mm)
const float link1Length = 220;
const float link2Length = 165;
float values[] = {200,300};
int currentValue = 0;

//Calculated values
float th2;
float th1;

//Array for axis positions: {axis1position, axis2position, etc.
byte positionArray[9][3] = { //[# of instruction sets][NUMBER OF POSITIONS PER INSTRUCTION]
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
};
byte arrayCounter = 0;
int instructionLength;
int instructionCount;

int read_LCD_buttons()  {  // read the buttons
  adc_key_in = analogRead(0);      // read the value from the sensor 
  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
  if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
  // For V1.1 us this threshold
  if (adc_key_in < 50)   return btnRIGHT;  
  if (adc_key_in < 250)  return btnUP; 
  if (adc_key_in < 450)  return btnDOWN; 
  if (adc_key_in < 650)  return btnLEFT; 
  if (adc_key_in < 850)  return btnSELECT;  
  return btnNONE;  // when all others fail, return this...
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  instructionLength = sizeof(positionArray[0]);
  instructionCount = sizeof(positionArray)/sizeof(positionArray[0]);
  
  
  Serial.println("========================================");
  Serial.println("Hello and welcome to Labibot v1.0");
  Serial.println("https://labitat.dk/wiki/MOVE_MASTER_II");
  Serial.println("Labitrack #231");
  Serial.println("========================================");
  delay(1000);
  Serial.println("System ready!");
  lcd.begin(16, 2);              // start the library
  lcd.setCursor(0,0);
  lcd.print("Labibot v1.0"); // print a simple message
}

void loop() { 
  lcdControl();
//  //Read X and Y from Processing
//  processingSerialControl3();
//  
//  //Calculate the angles  
//  findTheta(values[0],values[1]);
//  
//  
////  Wire.beginTransmission(2);
////  Wire.write(th1);
////  Wire.endTransmission();
////      
////  Wire.beginTransmission(3);
////  Wire.write(th2);
////  Wire.endTransmission();
//  
//Serial.print(th1);
//Serial.print(" - ");
//Serial.println(th2);
//  
}

void displayInstructionSets() { //Sends all current instruction sets through serial
  for (int i=0;i<instructionCount;i++){
    Serial.print("Instruction #");
    Serial.println(i);
    for (int n=0;n<instructionLength;n++){
      Serial.print(positionArray[i][n]);
      Serial.print(", ");
    }
    Serial.println();
  }
}

void setAxis(int interval) { //Sets all Axis based on instruction sets
  for (int i=0;i<instructionCount;i++) {
    for (int n=0;n<instructionLength;n++){
      Wire.beginTransmission(n+1);
      Wire.write(positionArray[i][n]);
      Wire.endTransmission();
    }
    delay(interval);
  }
}

void requestAxisPositions() {
  for (int i=0;i<instructionCount;i++) {
      Wire.requestFrom(i+1, 1);
      while(Wire.available()) {  //slave may send less than requested
        byte c = Wire.read();    //receive a byte as character
        Serial.print(c);        //print the character
        Serial.print(" ");
        positionArray[arrayCounter][i] = c;                  //PUT READ VALUE INTO AN ARRAY SOMEWHERE  
      }
      delay(100);
  }
  arrayCounter++;
  Serial.println("Positions saved");
}

void lcdControl() {
  lcd.setCursor(9,1);            // move cursor to second line "1" and 9 spaces over
  lcd.print(millis()/1000);      // display seconds elapsed since power-up
  lcd.setCursor(0,1);            // move to the begining of the second line
  lcd_key = read_LCD_buttons();  // read the buttons

  switch (lcd_key)  {            //16 characters max!
    case btnRIGHT:  {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("RIGHT ");
      lcd.setCursor(0,1);
      break;
    }
    case btnLEFT:  { //Requests and save position
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Saving position");
      lcd.setCursor(0,1);
      lcd.print("");
      requestAxisPositions();
      break;
    }
    case btnUP:    {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("(UP)");    
      break;
    }
    case btnDOWN:  {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Running pattern");
      setAxis(2000);
      break;
    }
    case btnSELECT:{
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Sending set");
      displayInstructionSets();
      break;
    }
    case btnNONE:{
      lcd.print("NONE");
      break;
    }
  }
}

float findTheta(int requestX, int requestY) {
  th2 = acos((pow(requestX,2)+pow(requestY,2)-pow(link1Length,2)-pow(link2Length,2))/(2*link1Length*link2Length));
  th2 = th2*180/PI;

  th1 = atan2(link1Length+link2Length*((pow(requestX,2)+pow(requestY,2)-pow(link1Length,2)-pow(link2Length,2))/(2*link1Length*link2Length)),link2Length*(sqrt(1-pow(((pow(requestX,2)+pow(requestY,2)-pow(link1Length,2)-pow(link2Length,2))/(2*link1Length*link2Length)),2))))-atan2(requestX,requestY);
  th1 = th1*180/PI;
  
/*  
  float steptwo = ((pow(requestX,2)+pow(requestY,2)-pow(link1Length,2)-pow(link2Length,2))/(2*link1Length*link2Length)); //CALLED C2
  float stepthree = (sqrt(1-pow(((pow(requestX,2)+pow(requestY,2)-pow(link1Length,2)-pow(link2Length,2))/(2*link1Length*link2Length)),2))); //CALLED S2
  float part1 = link1Length+link2Length*((pow(requestX,2)+pow(requestY,2)-pow(link1Length,2)-pow(link2Length,2))/(2*link1Length*link2Length)); //CALLED K1
  float part2 = link2Length*(sqrt(1-pow(((pow(requestX,2)+pow(requestY,2)-pow(link1Length,2)-pow(link2Length,2))/(2*link1Length*link2Length)),2))); //CALLED K2
  float theta = atan2(part1,part2)-atan2(requestX,requestY);
*/    
}

void processingSerialControl3() {  //Based on inverse kinematics request
  if(Serial.available()){
    float incomingValue = Serial.read();
    values[currentValue] = incomingValue;
    
    currentValue++;
    if(currentValue > 1){
      currentValue = 0;
    }
  }
}


