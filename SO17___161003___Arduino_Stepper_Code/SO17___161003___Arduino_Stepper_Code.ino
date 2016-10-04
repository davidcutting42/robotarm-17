int en = 8;
int xdir = 5;
int xstp = 2;
int ydir = 6;
int ystp = 3;

void setup() {
  pinMode(en, OUTPUT);
  pinMode(xdir, OUTPUT);
  pinMode(xstp, OUTPUT);
  pinMode(ydir, OUTPUT);
  pinMode(ystp, OUTPUT);
  digitalWrite(en, LOW);
  Serial.begin(9600);
  while (! Serial); // Wait untilSerial is ready - Leonardo
  Serial.println("Enter LED Number 0 to 7 or 'x' to clear");
}

void loop() {
  if (Serial.available()) {
    char ch = Serial.read();
    if ((ch >= '0') && (ch <= '6')) {
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
    }
  }
}
