// Potentiometer pin numbers
const int shoulderPot = A3;
const int elbowPot = A2;
const int slidePot = A4; 

// Potentiometer variables
int elbowPotVal = 0;
int shoulderPotVal = 0;

void setup() {
  // Potentiometer pin numbers
  pinMode(shoulderPot, INPUT);
  pinMode(elbowPot, INPUT);
  pinMode(slidePot, INPUT); 
  Serial.begin(9600);
}

void loop() {
  elbowPotVal = analogRead(elbowPot);
  shoulderPotVal = analogRead(shoulderPot);
  float elbowAngle = map(elbowPotVal,991,962,900,450);
  float shoulderAngle = map(shoulderPotVal,5,75,900,0);
  elbowAngle/=10;
  shoulderAngle/=10;
  Serial.print(elbowPotVal);
  Serial.print(", ");
  Serial.print(shoulderPotVal);
  Serial.print(", ");
  Serial.print(elbowAngle);
  Serial.print(", ");
  Serial.print(shoulderAngle);
  Serial.println();
}
