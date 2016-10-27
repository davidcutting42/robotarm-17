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
 *
 *
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
uint16_t au16data[10] = {
  1800, 450, 0, 0, 0, 0, 0, 0, 0, 0 };

// Modbus object declaration
Modbus slave(1,0,0); // this is slave @1 and RS-232 or USB-FTDI

// Stepper motor driver pin numbers
const int en = 8;
const int xdir = 5;
const int xstp = 2;
const int ydir = 6;
const int ystp = 3;
const int gdir = 7;
const int gstep = 4;

// Potentiometer pin numbers
const int elbowPot = A1;
const int shoulderPot = A0;

// Potentiometer variables
int elbowPotVal = 0;
int shoulderPotVal = 0;

float xang = 0;
float yang = 0;
float gang = 0;

int xstate = 0;
int ystate = 0;
int gstate = 0;

int xclk = 0;
int yclk = 0;
int gclk = 0;

float xdelay = 0;
float ydelay = 0;

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
  float elbowAngle = map(elbowPotVal, 30, 62, 900, 450)/10.0;
  float shoulderAngle = map(shoulderPotVal, 27, 100, 1800, 900)/10.0;
  
  au16data[2] = shoulderAngle*10;
  au16data[3] = elbowAngle*10;
  au16data[4] = shoulderPotVal;
  au16data[5] = elbowPotVal;
  
  // Update Modbus network 
  slave.poll( au16data, 10 );
  
  // Store motor angles
  xang = au16data[0]/10;
  yang = au16data[1]/10;
  
  
  /////////////////////////////////////////////////////////////////////
  //////////////////SHOULDER MOTOR CONTROL LOOP////////////////////////
  /////////////////////////////////////////////////////////////////////

  
  float xerr = abs(xang-shoulderAngle);
  
  if(xerr < 5) {
    xerr = 0;
    xstate = 0;
  }
  
  if (xerr > 0) {  
    if (xang < shoulderAngle) {
      switch (xstate) {
        case 0:
          digitalWrite(xdir, HIGH);
          digitalWrite(xstp, LOW);
          xdelay = 20/xerr;
          if (xdelay < 1){
            xdelay = 1;
          }
          xclk = millis();
          xstate = 1;
          break;
        case 1:
          if (millis() >= xclk+xdelay) {
            digitalWrite(xstp, HIGH);
            xclk = millis();
            xstate = 2;
          }
          break;
        case 2:
          if (millis() >= xclk+xdelay) {
            digitalWrite(xstp, LOW);
            xstate = 0;
          }
          break;
      }
    }
    else if (xang > shoulderAngle) {
      switch (xstate) {
        case 0:
          digitalWrite(xdir, LOW);
          digitalWrite(xstp, LOW);
          xdelay = 20/xerr;
          if (xdelay < 1){
            xdelay = 1;
          }
          xclk = millis();
          xstate = 1;
          break;
        case 1:
          if (millis() >= xclk+xdelay) {
            digitalWrite(xstp, HIGH);
            xclk = millis();
            xstate = 2;
          }
          break;
        case 2:
          if (millis() >= xclk+xdelay) {
            digitalWrite(xstp, LOW);
            xstate = 0;
          }
          break;
      }
    }
  }
  
  /////////////////////////////////////////////////////////////////////
  //////////////////ELBOW MOTOR CONTROL LOOP///////////////////////////
  /////////////////////////////////////////////////////////////////////
  
  float yerr = abs(yang-elbowAngle);
  
  if(yerr < 5) {
    yerr = 0;
    ystate = 0;
  }
  
  if (yerr > 0) {  
    if (yang < elbowAngle) {
      switch (ystate) {
        case 0:
          digitalWrite(ydir, HIGH);
          digitalWrite(ystp, LOW);
          ydelay = 20/yerr;
          if (ydelay < 1){
            ydelay = 1;
           }
          yclk = millis();
          ystate = 1;
          break;
        case 1:
          if (millis() >= yclk+ydelay) {
            digitalWrite(ystp, HIGH);
            yclk = millis();
            ystate = 2;
          }
          break;
        case 2:
          if (millis() >= yclk+ydelay) {
            digitalWrite(ystp, LOW);
            ystate = 0;
          }
          break;
      }
    }  
    else if (yang > elbowAngle) {
      switch (ystate) {
        case 0:
          digitalWrite(ydir, LOW);
          digitalWrite(ystp, LOW);
          ydelay = 20/yerr;
            if (ydelay < 1){
            ydelay = 1;
          }
          yclk = millis();
          ystate = 1;
          break;
        case 1:
          if (millis() >= yclk+ydelay) {
            digitalWrite(ystp, HIGH);
            yclk = millis();
            ystate = 2;
          }
          break;
        case 2:
          if (millis() >= yclk+ydelay) {
            digitalWrite(ystp, LOW);
            ystate = 0;
          }
          break;
      }
    }
  }
}
