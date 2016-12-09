/**
 *  Robot Arm Arduino Code - Arduino Mega with Custom Shield
 *
 *  This code is maintained by David Cutting on Github as part of the WM-Roboarm Repository:
 *  https://github.com/davecutting/WM-Roboarm
 *
 *  (C) 2016-2017 by David Cutting
 
**/

// Software libraries for Modbus communication
#include <SoftwareSerial.h>
#include <ModbusRtu.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//PWM Driver Object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define ASERVOMIN 110 // this is the 'minimum' pulse length count (out of 4096)
#define ASERVOMAX 550 // this is the 'maximum' pulse length count (out of 4096)

#define BSERVOMIN 110 // this is the 'minimum' pulse length count (out of 4096)
#define BSERVOMAX 550 // this is the 'maximum' pulse length count (out of 4096)

#define DSERVOMIN 110 // this is the 'minimum' pulse length count (out of 4096)
#define DSERVOMAX 550 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
uint8_t servonum = 0;

// Data array for Modbus network sharing
uint16_t au16data[10] = {
  1500, 450, 0, 0, 0, 0, 350, 0, 0, 0 };

// Modbus object declaration
Modbus slave(1,0,0); // this is slave @1 and RS-232 or USB-FTDI

// Stepper motor driver pin numbers
const int xdir = 27;
const int xstp = 5;
const int ydir = 26;
const int ystp = 4;
const int gdir = 28;
const int gstp = 6;

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

float xang = 180;
float yang = 45;
float gamma = 0;

int xstate = 0;
int ystate = 0;
int gstate = 0;

unsigned long xclk = 0;
unsigned long yclk = 0;
unsigned long gclk = 0;

float xdelay = 0;
float ydelay = 0;
unsigned long gdelay = 0;
int a = 1;

int shouldergain = 5;
int elbowgain = 5;

int minstep = 50;

int servoapulse = 0;
int servobpulse = 0;
int servocpulse = 0;
int servodpulse = DSERVOMIN;

int servoaangle = 90;
int servobangle = 90;
int servocangle = 90;

int side = 0;
int sidearchive = 0;

int dstate = 0; // 0 = empty, 1 = heads, 2 = tails
unsigned long dtimer = 0;
#define DDELAY 500000

//////////////////////////////////////////////////////////////////////////////////////////////

//SETUP CODE

//////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // Set stepper pin types
  pinMode(xdir, OUTPUT);
  pinMode(xstp, OUTPUT);
  pinMode(ydir, OUTPUT);
  pinMode(ystp, OUTPUT);
  pinMode(gdir, OUTPUT);
  pinMode(gstp, OUTPUT);

  pinMode(stpselect0, OUTPUT);
  pinMode(stpselect1, OUTPUT);
  pinMode(stpselect2, OUTPUT);

  // Potentiometer pin numbers
  pinMode(shoulderPot, INPUT);
  pinMode(elbowPot, INPUT);
  pinMode(slidePot, INPUT); 
  
  // Set to 8 microsteps/step  
  digitalWrite(stpselect0, HIGH);
  digitalWrite(stpselect1, HIGH);
  digitalWrite(stpselect2, LOW);
  
  analogReference(INTERNAL2V56);
  
  // Begin the Modbus communication as a slave
  slave.begin( 19200 );
  
  pwm.begin();
  
  pwm.setPWMFreq(60);
  
  pwm.setPWM(3, 0, (DSERVOMIN));
  delay(500);
  pwm.setPWM(3, 0, (DSERVOMAX));
 
}

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  pulselength /= 4096;  // 12 bits of resolution
  pulse *= 1000;
  pulse /= pulselength;
  pwm.setPWM(n, 0, pulse);
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
  float elbowAngle = (-0.8859 * elbowPotVal + 143.08)*10;
  float shoulderAngle = (1157.8 * pow(shoulderPotVal, -0.37))*10;
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
  
  // Store servo angles and penny side
  servoaangle = au16data[7];
  servobangle = au16data[8];
  side = au16data[9];
  
  switch (dstate) {
    case 0:  // waiting for command
      if ((side != sidearchive) && (side == 1)) {
        dstate = 1;
        dtimer = micros() + DDELAY;
        pwm.setPWM(3, 0, ((DSERVOMIN+DSERVOMAX)/2));
      }
      if ((side != sidearchive) && (side == 2)) {
        dstate = 2;
        dtimer = micros() + DDELAY;
        pwm.setPWM(3, 0, ((DSERVOMIN+DSERVOMAX)/2));
      }
      break;
    
    case 1:  // center hold, go min when done
      if (micros() > dtimer) {
        pwm.setPWM(3, 0, (DSERVOMIN));
        dstate = 0;
        au16data[9] = 0;
      }
      
      break;
    
    case 2:  // center hold, go max when don
      if (micros() > dtimer) {
        pwm.setPWM(3, 0, (DSERVOMAX));
        dstate = 0;
        au16data[9] = 0;
      }
      break;
  }
  sidearchive = side;
  
  servoapulse = map(servoaangle, 0, 180, ASERVOMIN, ASERVOMAX);
  servobpulse = map(servobangle, 0, 180, BSERVOMIN, BSERVOMAX);
  
  pwm.setPWM(0, 0, servoapulse);
  pwm.setPWM(1, 0, servobpulse);
  
  
  /////////////////////////////////////////////////////////////////////
  //////////////////SHOULDER MOTOR CONTROL LOOP////////////////////////
  /////////////////////////////////////////////////////////////////////

  
  float xerr = abs(xang-shoulderAngle);
  
  if(xerr < 1) {
    xerr = 0;
    xstate = 0;
  }
  
  if (xerr > 0) {  
    if (xang > shoulderAngle) {
      switch (xstate) {
        case 0:
          digitalWrite(xdir, HIGH);
          digitalWrite(xstp, LOW);
          xdelay =  shouldergain/xerr*1000;
          if (xdelay < minstep){
            xdelay = minstep;
          }
          xclk = micros();
          xstate = 1;
          break;
        case 1:
          if (micros() >= xclk+xdelay) {
            digitalWrite(xstp, HIGH);
            xclk = micros();
            xstate = 2;
          }
          break;
        case 2:
          if (micros() >= xclk+xdelay) {
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
          xdelay = shouldergain/xerr*1000;
          if (xdelay < minstep){
            xdelay = minstep;
          }
          xclk = micros();
          xstate = 1;
          break;
        case 1:
          if (micros() >= xclk+xdelay) {
            digitalWrite(xstp, HIGH);
            xclk = micros();
            xstate = 2;
          }
          break;
        case 2:
          if (micros() >= xclk+xdelay) {
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
  
  if(yerr < 3) {
    yerr = 0;
    ystate = 0;
  }
  
  if (yerr > 0) {  
    if (yang > elbowAngle) {
      switch (ystate) {
        case 0:
          digitalWrite(ydir, LOW);
          digitalWrite(ystp, LOW);
          ydelay = elbowgain/yerr*1000;
          if (ydelay < minstep){
            ydelay = minstep;
           }
          yclk = micros();
          ystate = 1;
          break;
        case 1:
          if (micros() >= yclk+ydelay) {
            digitalWrite(ystp, HIGH);
            yclk = micros();
            ystate = 2;
          }
          break;
        case 2:
          if (micros() >= yclk+ydelay) {
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
          ydelay = elbowgain/yerr*1000;
            if (ydelay < minstep){
            ydelay = minstep;
          }
          yclk = micros();
          ystate = 1;
          break;
        case 1:
          if (micros() >= yclk+ydelay) {
            digitalWrite(ystp, HIGH);
            yclk = micros();
            ystate = 2;
          }
          break;
        case 2:
          if (micros() >= yclk+ydelay) {
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
