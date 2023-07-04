#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

//Pin Driver Motor
#define pwmLeft 5
#define motorLeftA 8
#define motorLeftB 7

#define pwmRight 6
#define motorRightA 4
#define motorRightB 9

// Encoder pins for left wheel
const int encoderPinALeft = 3;
const int encoderPinBLeft = 2;

// Encoder pins for right wheel
const int encoderPinARight = 18;
const int encoderPinBRight = 19;

// Parameters
const float wheelCircumference = 3.1416 * 9.7; // cm
const int pulsesPerRevolution = 400;
const float robotBaseWidth = 20.0; // cm

// Variables
volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;
float distanceTraveledLeft = 0.0;
float distanceTraveledRight = 0.0;
float robotX = 0.0;
float robotY = 0.0;
float robotTheta = 0.0;
float dL = 0.0;
float dR = 0.0;
float distanceLeft, distanceRight;

float head;

byte thetaChar[8] = {
  0b00110,
  0b01001,
  0b01001,
  0b01111,
  0b01001,
  0b01001 ,
  0b00110,
  0b00000
};

void setup() {
  pinMode(encoderPinALeft, INPUT);
  pinMode(encoderPinBLeft, INPUT);
  pinMode(encoderPinARight, INPUT);
  pinMode(encoderPinBRight, INPUT);

  pinMode(motorLeftA, OUTPUT);
  pinMode(motorRightA, OUTPUT);
  pinMode(motorLeftB, OUTPUT);
  pinMode(motorRightB, OUTPUT);

  // Attach interrupts for encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderPinALeft), updateEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinARight), updateEncoderRight, RISING);

  Serial.begin(115200);
  lcd.begin();
  lcd.backlight();
  if (!mag.begin())
  {
    Serial.println("Ooops, no HMC5883 detected!");
    while (1);
  }
  sensor_t sensor;
  mag.getSensor(&sensor);

  lcd.setCursor(8, 0);
  lcd.print("ROBOT");
  lcd.setCursor(7, 1);
  lcd.print("MEDICAL");
  lcd.setCursor(6, 2);
  lcd.print("ASSISTANT");
  delay(2000);
}

void diam() {
  digitalWrite(motorLeftA, LOW);
  digitalWrite(motorLeftB, LOW);
  digitalWrite(motorRightA, LOW);
  digitalWrite(motorRightB, LOW);
  dL = 0;
  dR = 0;
  distanceLeft = 0;
  distanceRight = 0;
  head = getHead();
  odometry();
  Serial.println(String(dL) + "==DIAM==" + String(dR));
  delay(3000);
}

void maju(float target, float headTar) {
  float headMax = headTar+2;
  float headMin = headTar-2;
  dL = 0;
  dR = 0;
  target = target - 10;
  odometry();
  delay(5);
  analogWrite(pwmLeft, 51);
  analogWrite(pwmRight, 50);
  while (dL < target || dR < target) {
    head=getHead();
    if(head>headMax){
      digitalWrite(motorLeftA, LOW);
      digitalWrite(motorLeftB, HIGH);
      digitalWrite(motorRightA, LOW);
      digitalWrite(motorRightB, HIGH);
      delay(100);
    }
    else if(head<headMin){
    digitalWrite(motorLeftA, HIGH);
    digitalWrite(motorLeftB, LOW);
    digitalWrite(motorRightA, HIGH);
    digitalWrite(motorRightB, LOW);
      delay(100);
    }
    digitalWrite(motorLeftA, HIGH);
    digitalWrite(motorLeftB, LOW);
    digitalWrite(motorRightA, LOW);
    digitalWrite(motorRightB, HIGH);
    odometry();
    Serial.println(String(dL) + "==MAJU==" + String(dR));
  }
}

void mundur(float target) {
  dL = 0;
  dR = 0;
  target = target * -1;
  odometry();
  delay(5);
  analogWrite(pwmLeft, 51);
  analogWrite(pwmRight, 50);
  while (dL > target || dR > target) {
    digitalWrite(motorLeftA, LOW);
    digitalWrite(motorLeftB, HIGH);
    digitalWrite(motorRightA, HIGH);
    digitalWrite(motorRightB, LOW);
    odometry();
    Serial.println(String(dL) + "==MUNDUR==" + String(dR));
  }
}

void kiri(float headTar) {
  Serial.println("==KIRI==");
  dL = 0;
  dR = 0;
  odometry();
  delay(5);
  analogWrite(pwmLeft, 65);
  analogWrite(pwmRight, 65);
  float headMax = headTar+2;
  float headMin = headTar-2;
  head=getHead();
  while(head<headMin || head>headMax){
    head=getHead();
    digitalWrite(motorLeftA, LOW);
    digitalWrite(motorLeftB, HIGH);
    digitalWrite(motorRightA, LOW);
    digitalWrite(motorRightB, HIGH);
    head=getHead();
  }
  diam();
}

void kanan(float headTar) {
  Serial.println("==KANAN=="); 
  dL = 0;
  dR = 0;
  odometry();
  delay(5);
  analogWrite(pwmLeft, 65);
  analogWrite(pwmRight, 65);
  float headMax = headTar+2;
  float headMin = headTar-2;
  head=getHead();
  while(head<headMin || head>headMax){
    head=getHead();
    digitalWrite(motorLeftA, HIGH);
    digitalWrite(motorLeftB, LOW);
    digitalWrite(motorRightA, HIGH);
    digitalWrite(motorRightB, LOW);
    head=getHead();
  }
  diam();
}

int i = 0;

void loop() {
  getHead();
  modeA();
  delay(1000);
}

void modeA(){
  float headL=283;
  float headR=338;
  float headP=75;
  odometry();
  while (i < 1) {
    maju(100,headL);
    diam();
    kanan(headR);
    diam();
    maju(80,headR);
    diam();
    mundur(80);
    diam();
    kiri(headL);
    diam();
    maju(100,headL);
    diam();
    kanan(headR);
    diam();
    maju(80,headR);
    diam();
    mundur(80);
    diam();
    kanan(headP);
    diam();
    maju(180,headP);
    i++;
  }
  odometry();
  diam();
}

float getHead() {
  sensors_event_t event;
  mag.getEvent(&event);

  float heading = atan2(event.magnetic.y, event.magnetic.x);
  float declinationAngle = 0.22;
  heading += declinationAngle;

  if (heading < 0)
    heading += 2 * PI;

  if (heading > 2 * PI)
    heading -= 2 * PI;

  float headingDegrees = heading * 180 / M_PI;

  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  return headingDegrees;
}

void odometry() {
  // Calculate distance traveled based on encoder counts
  float pulsesLeft = encoderCountLeft / 2.0; // Assuming 2 pulses per cycle
  float distanceLeft = (pulsesLeft / pulsesPerRevolution) * wheelCircumference;
  distanceTraveledLeft += distanceLeft;
  dL += distanceLeft;

  float pulsesRight = encoderCountRight / 2.0; // Assuming 2 pulses per cycle
  float distanceRight = (pulsesRight / pulsesPerRevolution) * wheelCircumference;
  distanceTraveledRight += distanceRight;
  dR += distanceRight;

  // Reset encoder counts
  encoderCountLeft = 0;
  encoderCountRight = 0;

  // Calculate delta distance and delta theta
  float deltaDistance = (distanceLeft + distanceRight) / 2.0;
  float deltaTheta = (distanceLeft - distanceRight) / robotBaseWidth;

  // Update robot position and orientation
  robotX += deltaDistance * cos(robotTheta + deltaTheta / 2.0);
  robotY += deltaDistance * sin(robotTheta + deltaTheta / 2.0);
  robotTheta += deltaTheta;

  // Output robot position and orientation
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Pos. Robot:");
  lcd.setCursor(0, 1);
  lcd.print("X:"); lcd.print(robotX); lcd.print("cm, Y:"); lcd.print(robotY); lcd.print("cm");
  lcd.createChar(1, thetaChar);
  lcd.setCursor(0, 2);
  lcd.write(1); lcd.print(":"); lcd.print(robotTheta); lcd.print(" Radians");
  lcd.setCursor(0, 3);
  lcd.print("L:"); lcd.print(distanceTraveledLeft); lcd.print("cm");
  lcd.print("R:"); lcd.print(distanceTraveledRight); lcd.print("cm");

  Serial.print("Robot Position - X: ");
  Serial.print(robotX);
  Serial.print(" cm, Y: ");
  Serial.print(robotY);
  Serial.print(" cm, Theta: ");
  Serial.print(robotTheta);
  Serial.println(" radians");
  Serial.print("Distance Left   : "); Serial.println(distanceLeft);
  Serial.print("Distance Right  : "); Serial.println(distanceRight);
  Serial.print("Distance Traveled - Left Wheel: ");
  Serial.print(distanceTraveledLeft);
  Serial.println(" cm");
  Serial.print("Distance Traveled - Right Wheel: ");
  Serial.print(distanceTraveledRight);
  Serial.println(" cm");
  Serial.println("=====ODOMETRY=====");
}

// Interrupt service routines for encoders
void updateEncoderLeft() {
  if (digitalRead(encoderPinBLeft) == HIGH) {
    encoderCountLeft++;
  } else {
    encoderCountLeft--;
  }
}

void updateEncoderRight() {
  if (digitalRead(encoderPinBRight) == HIGH) {
    encoderCountRight++;
  } else {
    encoderCountRight--;
  }
}