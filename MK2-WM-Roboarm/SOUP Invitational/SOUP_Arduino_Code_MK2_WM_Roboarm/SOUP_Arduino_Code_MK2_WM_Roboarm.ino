/*  Modbus Register Table (Arduino is slave, USB Device running python is master)
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
 *  10              xymode              Master          Mode of X any X axes... 1=Fast, 0=Slow
 *  11              liftmode            Master          Mode of Lift motor... 1=Fast, 0=Slow
 *  12              encoderadeg         Slave           Current angle outputted by encoder A
 *  13              encoderbdeg         Slave           Current angle outputted by encoder B
 *  14              encoderddeg         Slave           Current angle outputted by encoder D
 *  15              servcpos            Master          Target position (degrees) of servo B
 */

#include "MyTypes.h" // File that defines waypoint constructor
#include <SimpleModbusSlave.h>
#include <Wire.h> // Library that contains framework for communication with PCA9685 (servo controller) and AS5048Bs (encoders)
#include <Adafruit_PWMServoDriver.h> // Library that manages PCA9685 outputs
#include <ams_as5048b.h> // Library that communicates with encoders (uses Wire)

#define stpmode 8 // Sets the stepping mode of all 3 motors which have chips on the main arduino shield. Set to 1, 2, 4, 8, 16, or 32
#define dmotstpmode 1 // Sets the stepping mode of the D motor. Change based on hardware selection to 1, 2, 4, 8, 16, or 32.

// Set minimum and maximum pulse lengths for A servo and B servo. Need to be tuned to get proper sweep for servo.
#define ASERVOMIN  215 // this is the 'minimum' pulse length count (out of 4096)
#define ASERVOMAX  550 // this is the 'maximum' pulse length count (out of 4096)
#define BSERVOMIN  100 // this is the 'minimum' pulse length count (out of 4096)
#define BSERVOMAX  550 // this is the 'maximum' pulse length count (out of 4096)
#define CSERVOMIN  300 // this is the 'minimum' pulse length count (out of 4096)
#define CSERVOMAX  620 // this is the 'maximum' pulse length count (out of 4096)

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // Construct servo controller object

// Define constants for the encoders
#define U_RAW 1
#define U_DEG 3

// Encoder Zero Positions, attained from zeroing code.
const float azero = 4918-4096;
const float bzero = 10642;
const float dzero = 1410;

// Construct encoder objects
AMS_AS5048B encodera(0x44);
AMS_AS5048B encoderb(0x48);
AMS_AS5048B encoderd(0x4C);

int mode = 0; // Mode variable - controls whether motors are moving, transmitted to raspberry pi as a check to make sure motors have reached target before continuting to next waypoint

// Set pin numbers for steppers A, B, C, and D
const int astep = 4;
const int adir = 26;
const int bstep = 5;
const int bdir = 27;
const int cstep = 6;
const int cdir = 28;
const int dstep = A4;
const int ddir = A3;

// Set pin numbers for A, B, and C (not D) step mode selection
const int stpmode0 = 32;
const int stpmode1 = 33;
const int stpmode2 = 34;

// Set lengths of humerus and ulna (arm links)
const float h = 249.2;
const float u = 249.2;

// Registers for the slave, make sure to include TOTAL_REGS_SIZE variable as last one to finish count.
enum 
{    
  mb_xtarget,  
  mb_ytarget,             
  mb_bendpreference,      
  mb_basetarget,          
  mb_steppersinposition,  
  mb_benddirection,       
  mb_mode,                
  mb_servapos,            
  mb_servbpos,            
  mb_motdangle,           
  mb_xymode,              
  mb_liftmode,
  mb_encoderadeg,
  mb_encoderbdeg,
  mb_encoderddeg,
  mb_servcpos,
  TOTAL_REGS_SIZE 
};

unsigned int holdingRegs[TOTAL_REGS_SIZE];

float angles[2]; // Array for storing calculated angles in inverse kinematics section

// Number of steps each motor needs to take before it reaches its target, decremented in each step
long astepdifference = 0;
long bstepdifference = 0;
long cstepdifference = 0;
long dstepdifference = 0;

// Direction of stepper motors, used to calculate the current absolute position of the motor.
int cstepdirection = 0;

// Switch case variables that control whether steppers are idle or stepping.
int astepswitch = 0;
int bstepswitch = 0;
int cstepswitch = 0;
int dstepswitch = 0;

// Timer (microseconds) that controls the pulselength of the stepper motors in the switch cases.
unsigned long cstepswitchtimer = 0;

// Ratio for number of steps of motor to number of degrees of joint based on gearing and pulley ratios
const float aratio = 200.0 * 32 / 10 * stpmode / 360.0;
const float bratio = 200.0 * 24 / 10 * stpmode / 360.0;
const float cratio = 200.0 * 50.0 * stpmode / 360.0;
const float dratio = 200.0 * dmotstpmode / 360.0;

// Function prototypes for degrees to step calculations
long adegreesstep(float deg);
long bdegreesstep(float deg);
long cdegreesstep(float deg);
long ddegreesstep(float deg); 

// Set all motors to their calibration position (zero)
long astepcount = adegreesstep(90);
long bstepcount = bdegreesstep(0);
long cstepcount = cdegreesstep(0);
long dstepcount = ddegreesstep(0);

// Minimum motor delay (time between steps)
const unsigned long minmotadelay = 5000 / aratio * 2;
const unsigned long minmotbdelay = 5000 / bratio * 2;
const unsigned long minmotcdelay = 10000 / cratio * 2;
const unsigned long minmotddelay = 10000 / dratio;

// Maximum motor delay (time between steps). Set only for a and b because they have acceleration and deceleration
const unsigned long maxmotadelay = minmotadelay * 5;
const unsigned long maxmotbdelay = minmotbdelay * 5;

// Acceleration and deceleration increments for adjusting the delay after every step. 
const long motadelaydecrement = 25;
const long motbdelaydecrement = motadelaydecrement * bratio / aratio;

// Sets taget delay (essentially fastest speed) for a given move. Currently has only a fast and a slow mode based on the actiontypexy variable
unsigned long motadelay = minmotadelay;
unsigned long motbdelay = minmotbdelay;
unsigned long motcdelay = minmotcdelay;
unsigned long motddelay = minmotddelay;

// Current delay, the one actually used to time steps.
unsigned long motadelayramp = motadelay;
unsigned long motbdelayramp = motbdelay;

// Variable used in acceleration and deceleration to calculate when to begin decelerating if the interval is shorter than the an acceleration plus deceleration curve.
unsigned long astepdifferencehalf = 0;
unsigned long bstepdifferencehalf = 0;

// Counts number of decrements of the delay during acceleration so that an equal number will be used in deceleration.
long adecrementcount = 0;
long bdecrementcount = 0;

// Initializes target position of servos to zero positions
int servoatargetcount = map(110, 0, 180, ASERVOMIN, ASERVOMAX);
int servobtargetcount = map(180, 0, 180, BSERVOMIN, BSERVOMAX);
int servoctargetcount = map(180, 0, 180, CSERVOMIN, CSERVOMAX);

// Current position of motor, incremented towards the target to give speed control of servos.
int servoacurrcount = 0;
int servobcurrcount = 0;
int servoccurrcount = 0;

// Sets maximum speed of servo position change (speed control)
const unsigned long minservoadelay = 2000;
const unsigned long minservobdelay = 1000;
const unsigned long minservocdelay = 2000;

// Sets speed of servo position change (speed control)
unsigned long servoadelay = minservoadelay;
unsigned long servobdelay = minservobdelay;
unsigned long servocdelay = minservocdelay;

// Timer used in servo switch cases
unsigned long servoatimer = 0;
unsigned long servobtimer = 0;
unsigned long servoctimer = 0;

// Selection variable used in servo switch cases
int servoaswitch = 0;
int servobswitch = 0;
int servocswitch = 0;

// Assists mode register in determining the current state of the machine and whether it is ready to move to next position.
int getstream = 0;

// Encoder readings (degrees) for A, B, and D joints
float jointacurrent = 0;
float jointbcurrent = 0;
float jointdcurrent = 0;

// Target position for joints A, B, and D
long stepperAtarget;
long stepperBtarget;
long stepperDtarget;

// Deadband constants for each encodered motor
const int dbsteppera = adegreesstep(1);
const int dbstepperb = bdegreesstep(1);
const int dbstepperd = cdegreesstep(2.5);

void setup() {
  // Set up each stepper motor control pin to an output
  pinMode(astep, OUTPUT);
  pinMode(adir, OUTPUT);
  pinMode(bstep, OUTPUT);
  pinMode(bdir, OUTPUT);
  pinMode(cstep, OUTPUT);
  pinMode(cdir, OUTPUT);
  pinMode(dstep, OUTPUT);
  pinMode(ddir, OUTPUT);

  // Set up each step mode control pin as an output 
  pinMode(stpmode0, OUTPUT);
  pinMode(stpmode1, OUTPUT);
  pinMode(stpmode2, OUTPUT);

  // Initialize stepper direction and step pins 
  digitalWrite(astep, LOW);
  digitalWrite(bstep, LOW);
  digitalWrite(cstep, LOW);
  digitalWrite(dstep, LOW);
  digitalWrite(adir, LOW);
  digitalWrite(bdir, LOW);
  digitalWrite(cdir, LOW);
  digitalWrite(ddir, LOW);

  // Set stepping mode based on the value of stpmode 
  
  if(stpmode == 2) // 1/2 microstep mode
  {
    digitalWrite(stpmode0, HIGH);
    digitalWrite(stpmode1, LOW);
    digitalWrite(stpmode2, LOW);
  }
  
  else if(stpmode == 4) // 1/4 microstep mode
  { 
    digitalWrite(stpmode0, LOW);
    digitalWrite(stpmode1, HIGH);
    digitalWrite(stpmode2, LOW);
  }
  
  else if(stpmode == 8) // 1/8 microstep mode
  { 
    digitalWrite(stpmode0, HIGH);
    digitalWrite(stpmode1, HIGH);
    digitalWrite(stpmode2, LOW);
  }
  
  else if(stpmode == 16) // 1/16 microstep mode
  {
    digitalWrite(stpmode0, LOW);
    digitalWrite(stpmode1, LOW);
    digitalWrite(stpmode2, HIGH);
  }
  
  else if(stpmode == 32) // 1/32 microstep mode
  {
    digitalWrite(stpmode0, HIGH);
    digitalWrite(stpmode1, LOW);
    digitalWrite(stpmode2, HIGH);
  }
  
  else // Full step mode
  {
    digitalWrite(stpmode0, LOW);
    digitalWrite(stpmode1, LOW);
    digitalWrite(stpmode2, LOW);
  }

  modbus_configure(57600, 1, 2, TOTAL_REGS_SIZE, 0);
  
  pwm.begin(); // Start servo driver chip
  
  pwm.setPWMFreq(60); // Set frequency to ~60 Hz

  // Initialize servos to starting position
  pwm.setPWM(1, 0, servoatargetcount);
  pwm.setPWM(0, 0, servobtargetcount);
  pwm.setPWM(2, 0, servoctargetcount);

  // Start encoder communication
  encodera.begin();
  encoderb.begin();
  encoderd.begin(); 

  // Set encoders to ccw or clockwise (false = ccw, true = cw)
  encodera.setClockWise(false);
  encoderb.setClockWise(true);
  encoderd.setClockWise(false);

  noInterrupts(); // disable all interrupts
  TCNT3 = 0;
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR3B |= (1 << CS31)|(1 << WGM32);    // 8 prescaler (gives about 33 ms max range) 
  TIMSK3 &= (~(1 << OCIE3A));   // disable timer overflow interrupt
  
  TCNT4 = 0;
  TCCR4A = 0;
  TCCR4B = 0;
  TCCR4B |= (1 << CS41)|(1 << WGM42);    // 8 prescaler (gives about 33 ms max range) 
  TIMSK4 &= (~(1 << OCIE4A));   // disable timer overflow interrupt
  
  TCNT5 = 0;
  TCCR5A = 0;
  TCCR5B = 0;
  TCCR5B |= (1 << CS51)|(1 << WGM52);    // 8 prescaler (gives about 33 ms max range) 
  TIMSK5 &= (~(1 << OCIE5A));   // disable timer overflow interrupt
  interrupts();             // enable all interrupts
}

/////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// MAIN LOOP //////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void loop() {
  // Update all modbus registers
  modbus_update(holdingRegs);

  if(holdingRegs[mb_mode] == 2) {
    if(getstream == 0) { 
      waypoint target;
      target.x = holdingRegs[mb_xtarget];
      target.y = holdingRegs[mb_ytarget];
      target.lhrh = holdingRegs[mb_benddirection];
      target.base = holdingRegs[mb_basetarget];
      target.dangle = holdingRegs[mb_motdangle];
      target.saangle = holdingRegs[mb_servapos];
      target.sbangle = holdingRegs[mb_servbpos];
      target.actiontypexy = holdingRegs[mb_xymode];
      target.actiontypelift = holdingRegs[mb_liftmode];
      target.actiontypeservos = 1;
      target.scangle = holdingRegs[mb_servcpos];
      target.x -= 1000;
      target.y -= 1000;
      target.dangle -= 1000;
      inversekinematics(target);
      getstream = 1;
    }
    
    if(cstepdifference == 0) {
      readencodera();
      readencoderb();
      
      movemotora();
      movemotorb();
      if(bstepdifference < dbstepperb && astepdifference < dbsteppera) {
        readencoderd();
        movemotord();
      }
      moveservoa();
      moveservob();
      moveservoc();
    } 

    
    
    else {
      movemotorc();
    }
    
    if(steppersdone() && servosdone()) {
      holdingRegs[mb_mode] = 0;
      getstream = 0;
    }
  }
  else if(holdingRegs[mb_mode] == 3) {
    cstepcount = cdegreesstep(0);
    holdingRegs[mb_mode] = 0;
  }
}

/////////////////////////////////////////////////////////////////////////////////
////////////////////// STEPPER COMPLETION CHECK /////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

bool steppersdone() {
  return ((astepdifference < dbsteppera) && (bstepdifference < dbstepperb) && (cstepdifference == 0) && (dstepdifference < dbstepperd));
}

/////////////////////////////////////////////////////////////////////////////////
//////////////////////// SERVO COMPLETION CHECK /////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

bool servosdone() {
  return ((servoacurrcount == servoatargetcount) && (servobcurrcount == servobtargetcount) && (servoccurrcount == servoctargetcount)); 
}

/////////////////////////////////////////////////////////////////////////////////
////////////////////////////// READ ENCODERS ////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

inline void readencodera() {
  jointacurrent = encodera.angleR(U_RAW) - azero;
  while(jointacurrent > 16384) {
    jointacurrent -= 16384;
  }
  while(jointacurrent < 0) {
    jointacurrent += 16384;
  }
  jointacurrent = (jointacurrent / 16384.0) * 360.0;
  astepcount = adegreesstep(jointacurrent);  
  holdingRegs[mb_encoderadeg] = jointacurrent*100;
  astepdifference = abs(astepcount - stepperAtarget);
}

inline void readencoderb() {
  jointbcurrent = encoderb.angleR(U_RAW) - bzero;
  while(jointbcurrent > 16384) {
    jointbcurrent -= 16384;
  }
  while(jointbcurrent < 0) {
    jointbcurrent += 16384;
  }
  jointbcurrent = (jointbcurrent / 16384.0) * 360.0;
  bstepcount = bdegreesstep(jointbcurrent);  
  holdingRegs[mb_encoderbdeg] = jointbcurrent*100;
  bstepdifference = abs(bstepcount - stepperBtarget);
}

inline void readencoderd() {  
  //noInterrupts();
  jointdcurrent = encoderd.angleR(U_RAW) - dzero;
  while(jointdcurrent > 16384) {
    jointdcurrent -= 16384;
  }
  while(jointdcurrent < 0) {
    jointdcurrent += 16384;
  }
  jointdcurrent = (jointdcurrent / 16384.0) * 360.0;
  dstepcount = ddegreesstep(jointdcurrent);
  holdingRegs[mb_encoderddeg] = jointdcurrent*100;
  dstepdifference = abs(dstepcount - stepperDtarget);
}

/////////////////////////////////////////////////////////////////////////////////
////////////////////////////// MOVE MOTORS //////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

////////////////////////////// MOVE MOTOR A /////////////////////////////////////
inline void movemotora() {
  if ((astepswitch == 0) && (astepdifference > dbsteppera)) {
    astepswitch = 1;
    motadelayramp = maxmotadelay;
    astepdifferencehalf = astepdifference / 2;
    adecrementcount = 0;
    setadirection();
    TCNT3 = 0;
    OCR3A = 0; // Jump immediately to ISR (preloading timer)
    TIMSK3 |= (1 << OCIE3A);   // enable timer overflow interrupt
  }
}  

////////////////////////////// MOVE MOTOR B /////////////////////////////////////
inline void movemotorb() {
  if ((bstepswitch == 0) && (bstepdifference > dbstepperb)) {
    bstepswitch = 1;  
    motbdelayramp = maxmotbdelay;
    bstepdifferencehalf = bstepdifference / 2;
    bdecrementcount = 0;
    setbdirection();
    TCNT4 = 0; 
    OCR4A = 0; // Jump immediately to ISR (preloading timer)
    TIMSK4 |= (1 << OCIE4A);   // enable timer overflow interrupt
  }
}

////////////////////////////// MOVE MOTOR C /////////////////////////////////////
inline void movemotorc() {
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
}

////////////////////////////// MOVE MOTOR D /////////////////////////////////////

inline void movemotord() {
  if ((dstepswitch == 0) && (dstepdifference > dbstepperd)) {
    dstepswitch = 1;
    setddirection();
    TCNT5 = 0;
    OCR5A = 0; // Jump immediately to ISR (preloading timer)
    TIMSK5 |= (1 << OCIE5A);   // enable timer overflow interrupt
  }
}

////////////////////////////// MOVE SERVO A /////////////////////////////////////

void moveservoa() {
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
}

////////////////////////////// MOVE SERVO B /////////////////////////////////////

void moveservob() {
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

////////////////////////////// MOVE SERVO C /////////////////////////////////////

void moveservoc() {
  switch (servocswitch) {
    case 0:
      if(steppersdone() && (servoctargetcount != servoccurrcount)) {
        servocswitch = 1;
        servoctimer = micros();
      }
      break;
    case 1:
      if(micros() >= (servoctimer + servocdelay)) {
        if(servoccurrcount > servoctargetcount) {
          servoccurrcount--;
        }
        else if(servoccurrcount < servoctargetcount) {
          servoccurrcount++;
        }
        else {
          servocswitch = 0;
        } 
        pwm.setPWM(2, 0, servoccurrcount);
        servoctimer = micros();
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

  stepperAtarget = adegreesstep(angles[0]);
  stepperBtarget = bdegreesstep(angles[1]);
  stepperDtarget = ddegreesstep(target.dangle);
  long stepperCtarget = cdegreesstep(target.base);
  
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
  
  servoatargetcount = map(target.saangle, 0, 180, ASERVOMIN, ASERVOMAX);
  servobtargetcount = map(target.sbangle, 0, 180, BSERVOMIN, BSERVOMAX);
  servoctargetcount = map(target.scangle, 0, 180, CSERVOMIN, CSERVOMAX);

  motadelay = target.actiontypexy ? minmotadelay : minmotadelay * 2.5;
  motbdelay = target.actiontypexy ? minmotbdelay : minmotbdelay * 2.5;
  motcdelay = target.actiontypelift ? minmotcdelay : minmotcdelay * 3;
  motddelay = minmotddelay;
  servoadelay = target.actiontypeservos ? minservoadelay : minservoadelay * 5;
  servobdelay = target.actiontypeservos ? minservobdelay : minservobdelay * 5;
  servocdelay = target.actiontypeservos ? minservocdelay : minservocdelay * 5;
}

inline float astepdegrees(long steps) {
  int degreescalc = (float)steps / aratio;
  while(degreescalc > 360) {
    degreescalc -= 360.0;
  }
  while(degreescalc < 0 && degreescalc) {
    degreescalc += 360.0;
  }
  return degreescalc;
}

inline float bstepdegrees(long steps) {
  int degreescalc = (float)steps / bratio;
  while(degreescalc > 360) {
    degreescalc -= 360.0;
  }
  while(degreescalc < 0 && degreescalc) {
    degreescalc += 360.0;
  }
  return degreescalc;
}

inline float cstepdegrees(long steps) {
  int degreescalc = (float)steps / cratio;
  while(degreescalc > 360 && degreescalc) {
    degreescalc -= 360.0;
  }
  while(degreescalc < 0 && degreescalc) {
    degreescalc += 360.0;
  }
  return degreescalc;
}

inline float dstepdegrees(long steps) {
  int degreescalc = (float)steps / dratio;
  while(degreescalc > 360 && degreescalc) {
    degreescalc -= 360.0;
  }
  while(degreescalc < 0 && degreescalc) {
    degreescalc += 360.0;
  }
  return degreescalc;
}

inline long adegreesstep(float deg) {
  return (long)(deg * aratio);
}

inline long bdegreesstep(float deg) {
  return (long)(deg * bratio);
}

inline long cdegreesstep(float deg) {
  return (long)(deg * cratio);
}

inline long ddegreesstep(float deg) {
  return (long)(deg * dratio);
}

inline void setadirection() {
  if(astepdegrees(stepperAtarget) < astepdegrees(astepcount)) {
    digitalWrite(adir, LOW); 
  }
  else {
    digitalWrite(adir, HIGH);
  }
}

inline void setbdirection() {
  if((bstepdegrees(stepperBtarget) < 180) && (bstepdegrees(bstepcount) > 180)) {
    digitalWrite(bdir, LOW);
  }
  else if((bstepdegrees(stepperBtarget) > 180) && (bstepdegrees(bstepcount) < 180)) {
    digitalWrite(bdir, HIGH);
  }
  else if((bstepdegrees(stepperBtarget) > bstepdegrees(bstepcount))) {
    digitalWrite(bdir, LOW);
  }
  else if((bstepdegrees(stepperBtarget) < bstepdegrees(bstepcount))) {
    digitalWrite(bdir, HIGH);
  }
}

inline void setddirection() {
  if(dstepdegrees(stepperDtarget) < dstepdegrees(dstepcount)) {
    digitalWrite(ddir, HIGH);
  }
  else {
    digitalWrite(ddir, LOW);
  }
}

ISR(TIMER3_COMPA_vect)        // interrupt service routine 
{
  OCR3A = (2 * motadelayramp) - 1;   //compare match register = [ 16,000,000Hz/ (prescaler * desired interrupt frequency) ] - 1

  switch (astepswitch) {
    case 1:
      digitalWrite(astep, LOW);
      astepswitch = 2;
      break;
    case 2:
      setadirection();
      digitalWrite(astep, HIGH);
      if(astepdifference < dbsteppera) {
        TIMSK3 &= (~(1 << OCIE3A));   // disable timer overflow interrupt
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
      break;
  }
}

ISR(TIMER4_COMPA_vect)        // interrupt service routine 
{
  OCR4A = (2 * motbdelayramp) - 1;   //compare match register = [ 16,000,000Hz/ (prescaler * desired interrupt frequency) ] - 1
  
  switch (bstepswitch) {
    case 1:
      digitalWrite(bstep, LOW);
      bstepswitch = 2;
      break;
    case 2:
      setbdirection();
      digitalWrite(bstep, HIGH);
      if(bstepdifference < dbstepperb) {
        TIMSK4 &= (~(1 << OCIE4A));   // disable timer overflow interrupt
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
      break;
  }
}

ISR(TIMER5_COMPA_vect)        // interrupt service routine 
{
  OCR5A = (2 * motddelay) - 1;   //compare match register = [ 16,000,000Hz/ (prescaler * desired interrupt frequency) ] - 1
  
  switch (dstepswitch) {
    case 1:
      digitalWrite(dstep, LOW);
      dstepswitch = 2;
      break;
    case 2:
      setddirection();
      digitalWrite(dstep, HIGH);
      if(dstepdifference < dbstepperd) {
        TIMSK5 &= (~(1 << OCIE5A));   // disable timer overflow interrupt
        dstepswitch = 0;
      }
      else {
        dstepswitch = 1;
      }
      break;
  }
}
