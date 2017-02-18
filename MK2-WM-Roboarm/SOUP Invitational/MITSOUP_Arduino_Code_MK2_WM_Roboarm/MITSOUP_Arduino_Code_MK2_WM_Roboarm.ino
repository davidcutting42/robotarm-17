/*  Modbus Register Table (Arduino is slave)
 *  Register:       Register Name:      Source:         Description:
 *  0               xtarget             Master          Target x coordinate (mm)
 *  1               ytarget             Master          Target y coordinate (mm)
 *  2               bendpreference      Master          Bend preference (0 or 1)
 *  3               basetarget          Master          Target angle for base motor (degrees)
 *  4               steppersinposition  Slave           0 = last command not executed, 1 = last command completed
 *  5               benddirection       Slave           Current bend direction
 *  6               mode                Master          0 = idle, 1 = run, 2 = stream (calibrate), 3 = set current position as zero
 *  7               servapos            Master          Target position (degrees) of servo A
 *  8               servbpos            Master          Target position (degrees) of servo B
 *  9               motdangle           Master          Motor d target position (degrees)
 *  12              encoderadeg         Slave           Current angle outputted by encoder A
 *  13              encoderbdeg         Slave           Current angle outputted by encoder B
 *  14              encoderddeg         Slave           Current angle outputted by encoder D
 *  15              servcpos            Master          Target position (degrees) of servo B
 */

#include "MyTypes.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ModbusRtu.h>
#include <SoftwareSerial.h>

#define stpmode 8 // Sets the stepping mode of all 3 motors. Set to 1, 2, 4, 8, 16, or 32
#define dmotstpmode 1

#define ASERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define ASERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

#define BSERVOMIN  50 // this is the 'minimum' pulse length count (out of 4096)
#define BSERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

#define CSERVOMIN  300 // this is the 'minimum' pulse length count (out of 4096)
#define CSERVOMAX  620 // this is the 'maximum' pulse length count (out of 4096)



Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int mode = 0; // 0 = no state, 1  

// Modbus object declaration
Modbus slave(1,0,0); // this is slave @1 and RS-232 or USB-FTDI

// Set pin numbers for steppers A, B, and C
const int astep = 4;
const int adir = 26;
const int bstep = 5;
const int bdir = 27;
const int cstep = 6;
const int cdir = 28;
const int dstep = A4;
const int ddir = A3;

// Set pin numbers for stepping mode selection
const int stpmode0 = 32;
const int stpmode1 = 33;
const int stpmode2 = 34;

// Set lengths of humerus and ulna (arm links)
float h = 249.2;
float u = 249.2;

// Data array for Modbus network sharing
uint16_t au16data[] = { 0, 0, 0, 0, 0, 0, 0, 110, 180, 0, 0, 0 };

// Array for storing calculated angles
float angles[2];

long astepcount = adegreesstep(90);
long bstepcount = bdegreesstep(0);
long cstepcount = cdegreesstep(0);
long dstepcount = ddegreesstep(0);

long astepdifference = 0;
long bstepdifference = 0;
long cstepdifference = 0;
long dstepdifference = 0;

int astepdirection = 0;
int bstepdirection = 0;
int cstepdirection = 0;
int dstepdirection = 0;

int astepswitch = 0;
int bstepswitch = 0;
int cstepswitch = 0;
int dstepswitch = 0;

unsigned long astepswitchtimer = 0;
unsigned long bstepswitchtimer = 0;
unsigned long cstepswitchtimer = 0;
unsigned long dstepswitchtimer = 0;

// Set ratio for number of steps to number of degrees
const float aratio = 200.0 * 32 / 10 * stpmode / 360.0;
const float bratio = 200.0 * 24 / 10 * stpmode / 360.0;
const float cratio = 200.0 * 50.0 * stpmode / 360.0;
const float dratio = 200.0 * dmotstpmode / 360.0;

const unsigned long minmotadelay = 5000 / aratio * 2.5;
const unsigned long minmotbdelay = 5000 / bratio * 2.5;
const unsigned long minmotcdelay = 10000 / cratio * 2;
const unsigned long minmotddelay = 10000 / dratio;

const unsigned long maxmotadelay = minmotadelay * 5;
const unsigned long maxmotbdelay = minmotbdelay * 5;

const long motadelaydecrement = 25;
const long motbdelaydecrement = motadelaydecrement * bratio / aratio;

unsigned long motadelay = minmotadelay;
unsigned long motbdelay = minmotbdelay;
unsigned long motcdelay = minmotcdelay;
unsigned long motddelay = minmotddelay;

unsigned long motadelayramp = motadelay;
unsigned long motbdelayramp = motbdelay;

unsigned long astepdifferencehalf = 0;
unsigned long bstepdifferencehalf = 0;

long adecrementcount = 0;
long bdecrementcount = 0;

int waypointselect = 0;

int servoatargetcount = map(110, 0, 180, ASERVOMIN, ASERVOMAX);
int servobtargetcount = map(180, 0, 180, BSERVOMIN, BSERVOMAX);

int servoacurrcount = 0;
int servobcurrcount = 0;

const unsigned long minservoadelay = 2000;
const unsigned long minservobdelay = 1000;

unsigned long servoadelay = minservoadelay;
unsigned long servobdelay = minservobdelay;

long servoatimer = 0;
long servobtimer = 0;

int servoaswitch = 0;
int servobswitch = 0;

int getstream = 0;

//long stepdifferenceall = 0;

void setup() {
  // Set each control pin to an output
  pinMode(astep, OUTPUT);
  pinMode(adir, OUTPUT);
  pinMode(bstep, OUTPUT);
  pinMode(bdir, OUTPUT);
  pinMode(cstep, OUTPUT);
  pinMode(cdir, OUTPUT);
  pinMode(dstep, OUTPUT);
  pinMode(ddir, OUTPUT);
  
  pinMode(stpmode0, OUTPUT);
  pinMode(stpmode1, OUTPUT);
  pinMode(stpmode2, OUTPUT);

  digitalWrite(astep, LOW);
  digitalWrite(bstep, LOW);
  digitalWrite(cstep, LOW);
  digitalWrite(dstep, LOW);
  
  digitalWrite(adir, LOW);
  digitalWrite(bdir, LOW);
  digitalWrite(cdir, LOW);
  digitalWrite(ddir, LOW);

  // Set stepping mode based on the value of stpmode 
  
  // 1/2 microstep mode
  if(stpmode == 2) {
    digitalWrite(stpmode0, HIGH);
    digitalWrite(stpmode1, LOW);
    digitalWrite(stpmode2, LOW);
  }
  // 1/4 microstep mode
  else if(stpmode == 4) {
    digitalWrite(stpmode0, LOW);
    digitalWrite(stpmode1, HIGH);
    digitalWrite(stpmode2, LOW);
  }
  // 1/8 microstep mode
  else if(stpmode == 8) {
    digitalWrite(stpmode0, HIGH);
    digitalWrite(stpmode1, HIGH);
    digitalWrite(stpmode2, LOW);
  }
  // 1/16 microstep mode
  else if(stpmode == 16) {
    digitalWrite(stpmode0, LOW);
    digitalWrite(stpmode1, LOW);
    digitalWrite(stpmode2, HIGH);
  }
  // 1/32 microstep mode
  else if(stpmode == 32) {
    digitalWrite(stpmode0, HIGH);
    digitalWrite(stpmode1, LOW);
    digitalWrite(stpmode2, HIGH);
  }
  // Full step mode
  else {
    digitalWrite(stpmode0, LOW);
    digitalWrite(stpmode1, LOW);
    digitalWrite(stpmode2, LOW);
  }

  slave.begin( 57600 );
  
  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  
  pwm.setPWM(1, 0, map(120, 0, 180, ASERVOMIN, ASERVOMAX));
  pwm.setPWM(0, 0, map(180, 0, 180, BSERVOMIN, BSERVOMAX));
  delay(100);
}

/////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// MAIN LOOP //////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void loop() {
  //stepdifferenceall = astepdifference + bstepdifference + cstepdifference + dstepdifference;
 
  slave.poll( au16data, 12 );
  
  /*
  if(au16data[6] == 1) {
    if (steppersdone() && servosdone()) {
      if(waypointselect >= 13) {
        au16data[6] = 0;
      }
      else {
        inversekinematics(wp[waypointselect]);
        waypointselect++; 
        delay(500);
      }
    }
    movemotors();
  }
  */
  
  if(au16data[6] == 2) {
    if(getstream == 0) { 
      waypoint target;
      target.x = au16data[0];
      target.y = au16data[1];
      target.lhrh = au16data[2];
      target.base = au16data[3];
      target.dangle = au16data[9];
      target.saangle = au16data[7];
      target.sbangle = au16data[8];
      target.actiontypexy = au16data[10];
      target.actiontypelift = au16data[11];
      target.actiontypeservos = 1;
      target.x -= 1000;
      target.y -= 1000;
      target.dangle -= 1000;
      inversekinematics(target);
      getstream = 1;
    }
    movemotors();
    if(steppersdone() && servosdone()) {
      au16data[6] = 0;
      getstream = 0;
    }
  }
  else if(au16data[6] == 3) {
    astepcount = adegreesstep(90);
    bstepcount = bdegreesstep(0);
    cstepcount = cdegreesstep(0);
    dstepcount = ddegreesstep(0); 
    au16data[6] = 0;
  }
}

bool steppersdone()
{
  long stepdifferenceall = astepdifference + bstepdifference + cstepdifference + dstepdifference;
  return (stepdifferenceall == 0);
}

bool servosdone()
{
  return (servoacurrcount == servoatargetcount && servobcurrcount == servobtargetcount); 
}

/////////////////////////////////////////////////////////////////////////////////
////////////////////////////// MOVE MOTORS //////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
void movemotors() {
  switch (astepswitch) {
    case 0:
      if ((astepdifference != 0) && (cstepdifference == 0)) {
        astepswitch = 1;
        astepswitchtimer = micros();
        motadelayramp = maxmotadelay;
        astepdifferencehalf = astepdifference / 2;
        adecrementcount = 0;
      }
      break;
    case 1:
      if(micros() >= (astepswitchtimer + motadelayramp)) {
        digitalWrite(astep, LOW);
        astepswitchtimer = micros();
        astepswitch = 2;
      }
      break;
    case 2:
      if(micros() >= (astepswitchtimer + motadelayramp)) {
        digitalWrite(astep, HIGH);
        astepswitchtimer = micros();
        if(astepdirection == 0) {
          astepcount++;
          
        }
        else if(astepdirection == 1) {
          astepcount--;
        }
        astepdifference--;
        if(astepdifference == 0) {
          astepswitch = 0;
        }
        else {
          astepswitch = 1;
          if(motadelayramp > motadelay && astepdifference > astepdifferencehalf) {
            motadelayramp -= motadelaydecrement;
            adecrementcount++;
          }
          else if(motadelayramp < maxmotadelay && astepdifference < adecrementcount) {
            motadelayramp += motadelaydecrement;
          }
        }
      }
      break;
  }


  switch (bstepswitch) {
    case 0:
      if ((bstepdifference != 0) && (cstepdifference == 0)) {
        bstepswitch = 1;
        bstepswitchtimer = micros();  
        motbdelayramp = maxmotbdelay;
        bstepdifferencehalf = bstepdifference / 2;
        bdecrementcount = 0;
      }
      break;
    case 1:
      if(micros() >= (bstepswitchtimer + motbdelayramp)) {
        digitalWrite(bstep, LOW);
        bstepswitchtimer = micros();
        bstepswitch = 2;
      }
      break;
    case 2:
      if(micros() >= (bstepswitchtimer + motbdelayramp)) {
        digitalWrite(bstep, HIGH);
        bstepswitchtimer = micros();
        if(bstepdirection == 0) {
          bstepcount++;
        }
        else if(bstepdirection == 1) {
          bstepcount--;
        }
        bstepdifference--;
        if(bstepdifference == 0) {
          bstepswitch = 0;
        }
        else {
          bstepswitch = 1;
          if(motbdelayramp > motbdelay && bstepdifference > bstepdifferencehalf) {
            motbdelayramp -= motbdelaydecrement;
            bdecrementcount++;
          }
          else if(motbdelayramp < maxmotbdelay && bstepdifference < bdecrementcount) {
            motbdelayramp += motbdelaydecrement;
          }
        }
      }
      break;
  }

  switch (cstepswitch) {
    case 0:
      if (cstepdifference != 0) {
        cstepswitch = 1;
        cstepswitchtimer = micros();
      }
      break;
    case 1:
      if(micros() >= (cstepswitchtimer + motcdelay)) {
        digitalWrite(cstep, LOW);
        cstepswitchtimer = micros();
        cstepswitch = 2;
      }
      break;
    case 2:
      if(micros() >= (cstepswitchtimer + motcdelay)) {
        digitalWrite(cstep, HIGH);
        cstepswitchtimer = micros();
        if(cstepdirection == 0) {
          cstepcount--;
        }
        else if(cstepdirection == 1) {
          cstepcount++;
        }
        cstepdifference--;
        if(cstepdifference == 0) {
          cstepswitch = 0;
        }
        else {
          cstepswitch = 1;
        }
      }
      break;
  }
  switch (dstepswitch) {
    case 0:
      if (dstepdifference != 0) {
        dstepswitch = 1;
        dstepswitchtimer = micros();
      }
      break;
    case 1:
      if(micros() >= (dstepswitchtimer + motddelay)) {
        digitalWrite(dstep, LOW);
        dstepswitchtimer = micros();
        dstepswitch = 2;
      }
      break;
    case 2:
      if(micros() >= (dstepswitchtimer + motddelay)) {
        digitalWrite(dstep, HIGH);
        dstepswitchtimer = micros();
        if(dstepdirection == 0) {
          dstepcount--;
        }
        else if(dstepdirection == 1) {
          dstepcount++;
        }
        dstepdifference--;
        if(dstepdifference == 0) {
          dstepswitch = 0;
        }
        else {
          dstepswitch = 1;
        }
      }
      break;
  }
  
  switch (servoaswitch) {
    case 0:
      if(steppersdone() && (servoatargetcount != servoacurrcount)) {
        servoaswitch = 1;
        servoatimer = micros();
      }
      break;
    case 1:
      if(micros() >= (servoatimer + servoadelay)) {
        if(servoacurrcount > servoatargetcount) {
          servoacurrcount--;
        }
        else if(servoacurrcount < servoatargetcount) {
          servoacurrcount++;
        }
        else {
          servoaswitch = 0;
        } 
        pwm.setPWM(1, 0, servoacurrcount);
        servoatimer = micros();
      }
      break;
  }
  switch (servobswitch) {
    case 0:
      if(steppersdone() && (servobtargetcount != servobcurrcount)) {
        servobswitch = 1;
        servobtimer = micros();
      }
      break;
    case 1:
      if(micros() >= (servobtimer + servobdelay)) {
        if(servobcurrcount > servobtargetcount) {
          servobcurrcount--;
        }
        else if(servobcurrcount < servobtargetcount) {
          servobcurrcount++;
        }
        else {
          servobswitch = 0;
        } 
        pwm.setPWM(0, 0, servobcurrcount);
        servobtimer = micros();
      }
      break;
  }
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// INVERSE KINEMATICS //////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void inversekinematics(waypoint target) {
  //Calculating the two different possibilities for the beta angle
  double beta1 = atan2(  (sqrt(1-pow(((pow(target.x, 2)+pow(target.y, 2)-pow(h, 2)-pow(u, 2))/(2.0 * h * u)), 2))),((pow(target.x, 2)+pow(target.y, 2)-pow(h, 2)-pow(u, 2))/(2.0 * h * u)));
  double beta2 = atan2((-(sqrt(1-pow(((pow(target.x, 2)+pow(target.y, 2)-pow(h, 2)-pow(u, 2))/(2.0 * h * u)), 2)))),((pow(target.x, 2)+pow(target.y, 2)-pow(h, 2)-pow(u, 2))/(2.0 * h * u)));

  // Calculating the two different possibilities for the alpha angle
  float k2a = u * sin(beta1);
  float k2b = u * sin(beta2);
  float k1a = h + (u * cos(beta1));
  float k1b = h + (u * cos(beta2));
  float alpha1 = atan2(target.y, target.x)-atan2(k2a, k1a);
  float alpha2 = atan2(target.y, target.x)-atan2(k2b, k1b);

  // Create an array of the 4 calculated alpha and beta angle, converted to degrees.
  float trigresults[] = {alpha1*180.0/M_PI, alpha2*180.0/M_PI, beta1*180.0/M_PI, beta2*180.0/M_PI};

  // Make all negative angles positive
  while(trigresults[0] < 0) {
    trigresults[0] += 360;
  }
  while(trigresults[1] < 0) {
    trigresults[1] += 360;
  }
  while(trigresults[2] < 0) {
    trigresults[2] += 360;
  }
  while(trigresults[3] < 0) {
    trigresults[3] += 360;
  }

  // Variables for check that ensures angles are within permissible range
  boolean checkone = true;
  boolean checktwo = true;

  // Test alpha1 and beta1 to see if they are inside the allowable limits
  if((trigresults[0] > 200 && trigresults[0] < 360) || (trigresults[2] > 160 && trigresults[2] < 185) || (trigresults[0] != trigresults[0]) || (trigresults[2] != trigresults[2])) {
    checkone = false;
  }
  
  // Test alpha2 and beta2 to see if they are inside the allowable limits
  if((trigresults[1] > 200 && trigresults[1] < 360) || (trigresults[3] > 160 && trigresults[3] < 185) || (trigresults[1] != trigresults[1]) || (trigresults[3] != trigresults[3])) {
    checktwo = false;
  }

  // If both alpha and beta are in an acceptable range in both cases, the predefined preference will be used to determine which angles will be used.
  if(checkone && checktwo) {
    if(target.lhrh == 1) {
      angles[0] = trigresults[0];
      angles[1] = trigresults[2];
    }
    else {
      angles[0] = trigresults[1];
      angles[1] = trigresults[3];
    }
  }

  // If one of the angle sets lies in an acceptable range and the other does not, the one in range will be transferred to a new array
  else if(checkone && (!checktwo)) {
    angles[0] = trigresults[0];
    angles[1] = trigresults[2];
  }
  else if((!checkone) && checktwo) {
    angles[0] = trigresults[1];
    angles[1] = trigresults[3];
  }
  
  long stepperAtarget = adegreesstep(angles[0]);
  long stepperBtarget = bdegreesstep(angles[1]);
  long stepperCtarget = cdegreesstep(target.base);
  long stepperDtarget = ddegreesstep(target.dangle);
  
  if(stepperAtarget < astepcount) {
    digitalWrite(adir, LOW);
    astepdirection = 1;
    astepdifference = astepcount - stepperAtarget; 
  }
  else if(stepperAtarget > astepcount) {
    digitalWrite(adir, HIGH);
    astepdirection = 0;
    astepdifference = stepperAtarget - astepcount;
  }
  
  if((bstepdegrees(stepperBtarget) < 180) && (bstepdegrees(bstepcount) > 180)) {
    digitalWrite(bdir, LOW);
    bstepdirection = 0;
    bstepdifference = bdegreesstep(bstepdegrees(bdegreesstep(360) - bstepcount + stepperBtarget));
  }
  else if((bstepdegrees(stepperBtarget) > 180) && (bstepdegrees(bstepcount) < 180)) {
    digitalWrite(bdir, HIGH);
    bstepdirection = 1;
    bstepdifference = bdegreesstep(bstepdegrees(bdegreesstep(360) - stepperBtarget + bstepcount));
  }
  else if((bstepdegrees(stepperBtarget) > bstepdegrees(bstepcount))) {
    digitalWrite(bdir, LOW);
    bstepdirection = 0;
    bstepdifference = bdegreesstep(bstepdegrees(stepperBtarget - bstepcount));
  }
  else if((bstepdegrees(stepperBtarget) < bstepdegrees(bstepcount))) {
    digitalWrite(bdir, HIGH);
    bstepdirection = 1;
    bstepdifference = bdegreesstep(bstepdegrees(bstepcount - stepperBtarget));
  }
  else {
    bstepdifference = 0;  
  }

  if(stepperCtarget < cstepcount) {
    digitalWrite(cdir, HIGH);
    cstepdirection = 0;
    cstepdifference = cstepcount - stepperCtarget; 
  }
  else if(stepperCtarget > cstepcount) {
    digitalWrite(cdir, LOW);
    cstepdirection = 1;
    cstepdifference = stepperCtarget - cstepcount;
  }
  else {
    cstepdifference = 0;  
  }
  
  if(stepperDtarget < dstepcount) {
    digitalWrite(ddir, HIGH);
    dstepdirection = 0;
    dstepdifference = dstepcount - stepperDtarget; 
  }
  else if(stepperDtarget > dstepcount) {
    digitalWrite(ddir, LOW);
    dstepdirection = 1;
    dstepdifference = stepperDtarget - dstepcount;
  }
  else {
    dstepdifference = 0;  
  }
  
  servoatargetcount = map(target.saangle, 0, 180, ASERVOMIN, ASERVOMAX);
  servobtargetcount = map(target.sbangle, 0, 180, BSERVOMIN, BSERVOMAX);

  motadelay = target.actiontypexy ? minmotadelay : minmotadelay * 5;
  motbdelay = target.actiontypexy ? minmotbdelay : minmotbdelay * 5;
  motcdelay = target.actiontypelift ? minmotcdelay : minmotcdelay * 3;
  motddelay = minmotddelay;
  servoadelay = target.actiontypeservos ? minservoadelay : minservoadelay * 5;
  servobdelay = target.actiontypeservos ? minservobdelay : minservobdelay * 5;
}

float astepdegrees(long steps) {
  int degreescalc = (float)steps / aratio;
  while(degreescalc > 360) {
    degreescalc -= 360.0;
  }
  while(degreescalc < 0 && degreescalc) {
    degreescalc += 360.0;
  }
  return degreescalc;
}

float bstepdegrees(long steps) {
  int degreescalc = (float)steps / bratio;
  while(degreescalc > 360) {
    degreescalc -= 360.0;
  }
  while(degreescalc < 0 && degreescalc) {
    degreescalc += 360.0;
  }
  return degreescalc;
}

float cstepdegrees(long steps) {
  int degreescalc = (float)steps / cratio;
  while(degreescalc > 360 && degreescalc) {
    degreescalc -= 360.0;
  }
  while(degreescalc < 0 && degreescalc) {
    degreescalc += 360.0;
  }
  return degreescalc;
}

float dstepdegrees(long steps) {
  int degreescalc = (float)steps / dratio;
  while(degreescalc > 360 && degreescalc) {
    degreescalc -= 360.0;
  }
  while(degreescalc < 0 && degreescalc) {
    degreescalc += 360.0;
  }
  return degreescalc;
}

long adegreesstep(float deg) {
  return (long)(deg * aratio);
}

long bdegreesstep(float deg) {
  return (long)(deg * bratio);
}

long cdegreesstep(float deg) {
  return (long)(deg * cratio);
}

long ddegreesstep(float deg) {
  return (long)(deg * dratio);
}
