const int gdir = 28;
const int gstp = 6;
const int stpselect0 = 32;
const int stpselect1 = 33;
const int stpselect2 = 34;
int i = 0;
int y = 0;

void setup() {
  pinMode(gdir, OUTPUT);
  pinMode(gstp, OUTPUT);
  pinMode(stpselect0, OUTPUT);
  pinMode(stpselect1, OUTPUT);
  pinMode(stpselect2, OUTPUT);

  digitalWrite(stpselect0, HIGH);
  digitalWrite(stpselect2, LOW);
  digitalWrite(stpselect1, LOW);
}
void loop() {
  digitalWrite(gdir, HIGH);
  if (i>100) {
    digitalWrite(gdir, LOW);
    y++;
  }
  if (y>100) {
    i = 0;
    y = 0;
  }
  digitalWrite(gstp, LOW);
  delay(10);
  digitalWrite(gstp, HIGH);
  delay(10);
  i++;
}
