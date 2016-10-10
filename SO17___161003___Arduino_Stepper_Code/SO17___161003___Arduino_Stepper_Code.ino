/**
 *  Robot Arm Arduino Code - Arduino #1
 *
 *  This code does the following:
 *  1)    Reads values from the potentiometers on the joints of a 2-joint arm 
 *        and scales the input to a degree reading
 *  2)    Sends those values using the modbus protocol to a connected raspberry pi
 *  3)    Recieves commands from a connected Raspberry Pi using Modbus Protocol 
 *        regarding motor position 
 *  4)    Controls 3 stepper motors to position the arm based on recieved angles
 *  
 *  This code is maintained by David Cutting on Github as part of the WM-Roboarm Repository:
 *  https://github.com/davecutting/WM-Roboarm
 *
 *  (C) 2016-2017 by David Cutting
**/


// Software libraries for Modbus communication
#include <SoftwareSerial.h>
#include <ModbusRtu.h>

// Data array for Modbus network sharing
uint16_t au16data[7] = {
  0, 0, 0, 0, 0, 0 };

// Modbus object declaration
Modbus slave(1,0,0); // this is slave @1 and RS-232 or USB-FTDI

// Stepper motor driver pin numbers
const int en = 8;
const int xdir = 5;
const int xstp = 2;
const int ydir = 6;
const int ystp = 3;

// Potentiometer pin numbers
const int elbowPot = A1;
const int shoulderPot = A0;

// Potentiometer variables
int elbowPotVal = 0;
int shoulderPotVal = 0;

int xang = 0;
int yang = 0;

void setup() {
  // Set stepper pin types
  pinMode(en, OUTPUT);
  pinMode(xdir, OUTPUT);
  pinMode(xstp, OUTPUT);
  pinMode(ydir, OUTPUT);
  pinMode(ystp, OUTPUT);
  
  // Enable the stepper drivers
  digitalWrite(en, LOW);
  
  // Begin the Modbus communication as a slave
  slave.begin( 19200 );
}

void loop() {
  // Read arm potentiometers
  elbowPotVal = analogRead(elbowPot);
  shoulderPotVal = analogRead(shoulderPot);
  
  // Map arm potentiometer readings to corresponding degree measurements
  int elbowAngle = map(elbowPotVal, 0, 76, 30, 155);
  int shoulderAngle = map(shoulderPotVal, 6, 187, 150, 0);
  
  // Update Modbus registers with raw potentiometer values and arm degree measurements
  au16data[2] = shoulderAngle*10;
  au16data[3] = elbowAngle*10;
  au16data[4] = shoulderPotVal;
  au16data[5] = elbowPotVal;
  
  // Update Modbus network 
  slave.poll( au16data, 7 );
  
  // Store motor angles
  xang = au16data[0];
  yang = au16data[1];
  int enable = au16data[6];
  
  if (enable == 1) {
    digitalWrite(en, HIGH);
  }
  
  if (xang < shoulderAngle) {
    digitalWrite(xdir, HIGH);
    digitalWrite(xstp, LOW);
    delay(10);
    digitalWrite(xstp, HIGH);
    delay(10);
  }
  if (xang > shoulderAngle) {
    digitalWrite(xdir, LOW);
    digitalWrite(xstp, LOW);
    delay(10);
    digitalWrite(xstp, HIGH);
    delay(10);
  }
}
