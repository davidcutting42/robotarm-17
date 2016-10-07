#include <SoftwareSerial.h>
#include <ModbusRtu.h>

// data array for modbus network sharing
uint16_t au16data[6] = {
  0, 0, 0, 0, 0, 0 };

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  u8serno : serial port (use 0 for Serial)
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Modbus slave(1,0,0); // this is slave @1 and RS-232 or USB-FTDI

const int en = 8;
const int xdir = 5;
const int xstp = 2;
const int ydir = 6;
const int ystp = 3;

const int elbowPot = A1;
const int shoulderPot = A0;

int elbowPotVal = 0;
int shoulderPotVal = 0;

void setup() {
  pinMode(en, OUTPUT);
  pinMode(xdir, OUTPUT);
  pinMode(xstp, OUTPUT);
  pinMode(ydir, OUTPUT);
  pinMode(ystp, OUTPUT);
  digitalWrite(en, LOW);
  //Serial.begin(115200);
  //while (! Serial); // Wait untilSerial is ready - Leonardo
  //Serial.println("Enter LED Number 0 to 7 or 'x' to clear");
  slave.begin( 19200 );
}

void loop() {
  elbowPotVal = analogRead(elbowPot);
  int elbowAngle = map(elbowPotVal, 0, 76, 30, 155);
  shoulderPotVal = analogRead(shoulderPot);
  int shoulderAngle = map(shoulderPotVal, 6, 187, 150, 0);
    
  au16data[2] = shoulderAngle*10;
  au16data[3] = elbowAngle*10;
  au16data[4] = shoulderPotVal;
  au16data[5] = elbowPotVal;
  
  slave.poll( au16data, 6 );

  /*if ((ch >= '0') && (ch <= '6')) {
    int axis = ch -'0';
    if (axis == 0) {
      digitalWrite (xdir, LOW);
      for (int x = 40; x > 0; x--) {
        digitalWrite(xstp, LOW);
        delay(10);
        digitalWrite(xstp, HIGH);
        delay(10);
      }
   }
   if (axis == 1) {
     digitalWrite (xdir, HIGH);
     for (int x = 40; x > 0; x--) {
       digitalWrite(xstp, LOW);
       delay(10);
       digitalWrite(xstp, HIGH);
       delay(10);
     }
   }
   if (axis == 2) {
     digitalWrite (ydir, LOW);
     for (int x = 40; x > 0; x--) {
       digitalWrite(ystp, LOW);
       delay(10);
       digitalWrite(ystp, HIGH);
       delay(10);
     }
   }
   if (axis == 3) {
     digitalWrite (ydir, HIGH);
     for (int x = 40; x > 0; x--) {
       digitalWrite(ystp, LOW);
       delay(10);
       digitalWrite(ystp, HIGH);
       delay(10);
      }
    }
  }*/
}
