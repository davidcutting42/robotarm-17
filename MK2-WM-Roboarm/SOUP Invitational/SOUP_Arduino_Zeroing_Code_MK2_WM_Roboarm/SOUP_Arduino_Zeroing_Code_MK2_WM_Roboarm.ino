/*  
 *  A simple utility program that will read the values of three AS5048B's connected over I2C and print their values to the Serial monitor for the purpouse of debugging/setup. 
 *  Written by David Cutting Feb. 3, 2017
 */
 
#include <Wire.h> // Library that contains framework for communication with AS5048Bs (encoders)
#include <ams_as5048b.h> // Library that communicates with encoders (uses Wire)

// Define constants for the encoders
#define U_RAW 1
//#define U_DEG 3
#define U_DEG 1

// Construct encoder objects
AMS_AS5048B encodera(0x44);
AMS_AS5048B encoderb(0x48);
AMS_AS5048B encoderd(0x4C);

void setup() {
  // Start encoder communication
  encodera.begin();
  encoderb.begin();
  encoderb.setClockWise(true);
  encoderd.begin();

  Serial.begin(115200);
}

void loop() {
  Serial.print("Encoder D: ");
  float heeeey = (encoderd.angleR(U_DEG) - (1410));
  while(heeeey > 16384) {
    heeeey -= 16384;
  }
  while(heeeey < 0) {
    heeeey += 16384;
  }
  heeeey = heeeey  / 16384 * 360;
  Serial.println(heeeey);
  //Serial.print("Encoder B: ");
  //Serial.println(encoderb.angleR(U_DEG));
  //Serial.print("Encoder D: ");
  //Serial.println(encoderd.angleR(U_DEG));
  delay(500);
}
