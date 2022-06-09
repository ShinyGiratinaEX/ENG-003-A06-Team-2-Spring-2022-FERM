// C++ code
#include <LiquidCrystal_I2C.h>

//Accelerometer
#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_LSM303DLH_Mag_Unified mag = Adafruit_LSM303DLH_Mag_Unified(12345);

//Accelerometer variables
const float Pi = 3.14159;
float heading = 0.0;
float baseHeading = 0.0;

//Motor controller variables
int enablePinL = 11;
int in1PinL = 10;
int in2PinL = 9;
int speedL = 440;
bool reverseL = LOW;

int enablePinR = 6;
int in1PinR = 5;
int in2PinR = 3;
int speedR = 485;
bool reverseR = HIGH;

//Temp sensor variables
const int tempPin = A0;
const float baseLineTemp = 75;

//LCD variables
LiquidCrystal_I2C lcd_1(0x20,16,2);

//Button variables 
int bSubState = 0;
int bAddState = 0;
int bStartState = 0;

int bSubPin = 2;
int bAddPin = 4;
int bStartPin = 8;

//Dist sensor variables
float cm = 0;
int triggerPin = 12;
int echoPin = 13;

//LED and Buzzer
int buzzPin = 7;

//Movement Variables
bool moving = LOW;
int dist = 0;
const float vel = 7;
const float rot_vel = 0.0153;

//Methods

//Moves vehicle forward the given distance, then rotates, then returns
void moveVehicle() {

  forwardVehicle(dist, HIGH);

  rotateVehicle(180.0, HIGH);

  forwardVehicle(dist, HIGH);

  moving = LOW;
  dist = 0;
}

//Goes forward given a distance
void forwardVehicle(float distance, bool avoid) {
  float starttime = millis();
  float endtime = starttime;
  float avoidTime = 0.0;
  
  int runtime = int(distance * vel*1000);
  int distTravelled = 0;

  printLCD(0, 0, "Moving...");
  
  setMotor(speedL, reverseL, 'L');
  setMotor(speedR, reverseR, 'R');
  
  while ((endtime - starttime) <= runtime) {
    distTravelled = int((endtime-starttime)/1000 * vel);
    cm = 0.01723*readUltrasonicDistance(triggerPin, echoPin);
    // Object avoidance method goes here

    if (cm < 25 && avoid) {
      avoidTime += avoidObject(); 
      runtime -= int(0.75 * vel * 1000);
      setMotor(speedL, reverseL, 'L');
      setMotor(speedR, reverseR, 'R');
    }

    alarm();
    endtime = millis()-avoidTime;
  }
  
  setMotor(0, reverseL, 'L');
  setMotor(0, reverseR, 'R');
}

//Rotates for given angle
void rotateVehicle(float angle, bool dir) {

  float starttime = millis();
  float endtime = starttime;

  int runtime = int(angle * rot_vel * 1000);

  setMotor(speedL, dir, 'L');
  setMotor(speedR, dir, 'R');

  while ((endtime - starttime) <= runtime) {
    alarm();
    endtime = millis();
  }

  setMotor(0, reverseL, 'L');
  setMotor(0, reverseR, 'R');
}

//Object avoidance
float avoidObject() {

  float starttime = millis();

  rotateVehicle(90, HIGH);
  forwardVehicle(0.25, LOW);
  rotateVehicle(90, LOW);
  forwardVehicle(0.75, LOW);
  rotateVehicle(90, LOW);
  forwardVehicle(0.25, LOW);
  rotateVehicle(90, HIGH);

  float endtime = millis();
  return endtime-starttime;

}

//Returns the distance from the distance sensor
long readUltrasonicDistance(int triggerPin, int echoPin)
{
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pinMode(echoPin, INPUT);
  return pulseIn(echoPin, HIGH);
}

//Returns the compass heading
float readAccel() {

  //Accelerometer
  sensors_event_t event;
  mag.getEvent(&event);
  float dir = (atan2(event.magnetic.y, event.magnetic.x) * 180.0) / Pi;

  if (dir < 0) {
    dir = 360 + dir;
  }

  return dir;
}

//Returns the temperature reading
float readTemp(int pin) {
  int tempVal = analogRead(pin);
  float volt = tempVal*5.0;
  return ((volt/1024.0 - 0.5) * 100);
}

//Causes the indicated motor to move
void setMotor(int speed, bool reverse, char dir) {

  if(dir == 'L'){
    analogWrite(enablePinL, speed);
    digitalWrite(in1PinL, !reverse);
    digitalWrite(in2PinL, reverse);
  }
  
  else if (dir == 'R'){
    analogWrite(enablePinR, speed);
    digitalWrite(in1PinR, !reverse);
    digitalWrite(in2PinR, reverse);
  }
}

//Prints on LCD screen
void printLCD(int col, int row, String text) {
  lcd_1.setCursor(col, row);
  lcd_1.print(text);

  for (int i = 0; i < 16-col-text.length(); i++){
    lcd_1.print(" ");
  }
}

//Alarm for when temperature is critical
void alarm() {
  float temp = readTemp(tempPin);
  printLCD(9,1, String(temp));
  while (temp > 40 ) {
    tone(buzzPin, 262, 2000);
    temp = readTemp(tempPin);
    printLCD(9,1, String(temp));
    delay(2000);
  }
}

void setup(void)
{
  Serial.begin(115200);

  if (!mag.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1)
      ;
  }
  
  //Motor setup
  pinMode(in1PinL, OUTPUT);
  pinMode(in2PinL, OUTPUT);
  pinMode(enablePinL, OUTPUT);
  
  pinMode(in1PinR, OUTPUT);
  pinMode(in2PinR, OUTPUT);
  pinMode(enablePinR, OUTPUT);
  
  //LCD setup
  lcd_1.init();
  lcd_1.backlight();
  lcd_1.home();
  lcd_1.setCursor(0,1);
  lcd_1.print("Temp(C): ");
  
  //Button setup
  pinMode(bAddPin, OUTPUT);
  pinMode(bSubPin, OUTPUT);
  pinMode(bStartPin, OUTPUT);
}

void loop(void)
{
  //Temp reading
  float temp = readTemp(tempPin);
  alarm();

  cm = 0.01723*readUltrasonicDistance(triggerPin, echoPin);

  //Accelerometer
  heading = readAccel();

  Serial.print("Compass Heading: ");
  Serial.println(heading);
  
  //Button readings
  bAddState = digitalRead(bAddPin);
  bSubState = digitalRead(bSubPin);
  bStartState = digitalRead(bStartPin);
  
  //If start button pressed, begins to move
  if (bStartState == HIGH) {
    moving = HIGH;
    baseHeading = heading;
  }
  
  //If add button is pressed, adds 1 to the distance
  else if (bAddState == HIGH) {
    dist += 1;
  }
  
  //If sub button is pressed, subtracts 1 from the distance
  else if (bSubState ==HIGH && dist >0) {
    dist -= 1;
  }
  
  //Turns on motors if the moving variable is- HIGH
  if (moving == HIGH) {
    moveVehicle();
  }
  
  //LCD prints out the distance
  printLCD(0, 0, "Dist(m): " + String(dist));
  
  delay(100);
}