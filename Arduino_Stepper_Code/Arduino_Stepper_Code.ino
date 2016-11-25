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
  1800, 500, 0, 0, 0, 0, 350, 0, 0, 0 };

// Modbus object declaration
Modbus slave(1,0,0); // this is slave @1 and RS-232 or USB-FTDI

// Stepper motor driver pin numbers
const int xdir = 27;
const int xstp = 5;
const int ydir = 26;
const int ystp = 4;
const int gdir = 28;
const int gstp = 6;

const int rled = 26;
const int gled = 4;
const int bled = 27;

const int stpselect0 = 32;
const int stpselect1 = 33;
const int stpselect2 = 34;

// Potentiometer pin numbers
const int shoulderPot = A2;
const int elbowPot = A3;
const int slidePot = A4; 

// Potentiometer variables
int elbowPotVal = 0;
int shoulderPotVal = 0;

float xang = 90;
float yang = 45;
float gamma = 0;

int xstate = 0;
int ystate = 0;
int gstate = 0;

int xclk = 0;
int yclk = 0;
unsigned long gclk = 0;

float xdelay = 0;
float ydelay = 0;
unsigned long gdelay = 0;
int a = 1;

void setup() {
  // Set stepper pin types
  pinMode(xdir, OUTPUT);
  pinMode(xstp, OUTPUT);
  pinMode(ydir, OUTPUT);
  pinMode(ystp, OUTPUT);
  pinMode(gdir, OUTPUT);
  pinMode(gstp, OUTPUT);

  pinMode(rled, OUTPUT);
  pinMode(gled, OUTPUT);
  pinMode(bled, OUTPUT);

  pinMode(stpselect0, OUTPUT);
  pinMode(stpselect1, OUTPUT);
  pinMode(stpselect2, OUTPUT);

  // Potentiometer pin numbers
  pinMode(shoulderPot, INPUT);
  pinMode(elbowPot, INPUT);
  pinMode(slidePot, INPUT); 
  
  // Set to 8 microsteps/step  
  digitalWrite(stpselect0, HIGH);
  digitalWrite(stpselect1, LOW);
  digitalWrite(stpselect2, LOW);
  
  // Begin the Modbus communication as a slave
  slave.begin( 19200 );
}

void loop() {
  if (a == 1) {
    a = 0;
    delay(2000);
  }
  
  
  // Read arm potentiometers
  elbowPotVal = analogRead(elbowPot);
  shoulderPotVal = analogRead(shoulderPot);
  
  // Map arm potentiometer readings to corresponding degree measurements
  float elbowAngle = map(elbowPotVal,1005,970,900,450);
  float shoulderAngle = map(shoulderPotVal,50,147,1800,900);
  shoulderAngle /= 10;
  elbowAngle /= 10;
  
  au16data[2] = shoulderAngle*10;
  au16data[3] = elbowAngle*10;
  au16data[4] = shoulderPotVal;
  au16data[5] = elbowPotVal;
  
  // Update Modbus network 
  slave.poll( au16data, 10 );
  
  // Store motor angles
  xang = au16data[0]/10;
  yang = au16data[1]/10;
  gamma = au16data[6];
  gamma -= 350;
  
  
  
  
  /////////////////////////////////////////////////////////////////////
  //////////////////SHOULDER MOTOR CONTROL LOOP////////////////////////
  /////////////////////////////////////////////////////////////////////

  
  float xerr = abs(xang-shoulderAngle);
  
  if(xerr < 5) {
    xerr = 0;
    xstate = 0;
  }
  
  if (xerr > 0) {  
    if (xang > shoulderAngle) {
      switch (xstate) {
        case 0:
          digitalWrite(xdir, HIGH);
          digitalWrite(xstp, LOW);
          xdelay =  50/xerr;
          if (xdelay < 2){
            xdelay = 2;
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
    else if (xang < shoulderAngle) {
      switch (xstate) {
        case 0:
          digitalWrite(xdir, LOW);
          digitalWrite(xstp, LOW);
          xdelay = 50/xerr;
          if (xdelay < 2){
            xdelay = 2;
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
    if (yang > elbowAngle) {
      switch (ystate) {
        case 0:
          digitalWrite(ydir, LOW);
          digitalWrite(ystp, LOW);
          ydelay = 50/yerr;
          if (ydelay < 2){
            ydelay = 2;
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
    else if (yang < elbowAngle) {
      switch (ystate) {
        case 0:
          digitalWrite(ydir, HIGH);
          digitalWrite(ystp, LOW);
          ydelay = 50/yerr;
            if (ydelay < 2){
            ydelay = 2;
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
  
  
  /////////////////////////////////////////////////////////////////////
  //////////////////BASE MOTOR CONTROL/////////////////////////////////
  /////////////////////////////////////////////////////////////////////
  gdelay = 350/(abs(gamma))*1000;
  if (gdelay < 2000){
    gdelay = 2000;
  }
  
  if (gamma > 0) { 
    digitalWrite(gdir, LOW);
  }
  if (gamma < 0) {
    digitalWrite(gdir, HIGH);
  }
        
  if (abs(gamma) > 0) {
    switch (gstate) {
      case 0: //idle
        digitalWrite(gstp, HIGH);
        gclk = micros();
        gstate = 1;
        break;
      case 1: //running high
        if (micros() >= gclk+gdelay) {
          digitalWrite(gstp, LOW);
          gclk = micros();
          gstate = 2;
        }
        break;
      case 2: //running low
        if (micros() >= gclk+gdelay) {
          digitalWrite(gstp, HIGH);
          gclk = micros();
          gstate = 1;
        }
        break;
    }
  }
  else {
    gstate = 0;
  }
}
