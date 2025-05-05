#include <Servo.h>
Servo servo;

#define trig 6
#define echo 5

#define kp 6
#define ki 0.02
#define kd 0.7

double priError = 0;
double toError = 0;

void setup() {
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  servo.attach(9);
  Serial.begin(9600);
  servo.write(50);
}

void loop() {
  PID();
  delay(100);  // Add delay for stability and readable output
}

long distance() {
  digitalWrite(trig, LOW);
  delayMicroseconds(4);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long t = pulseIn(echo, HIGH);
  long cm = t / 29 / 2;
  return cm;
}

void PID() {
  int dis = distance();

  int setP = 15;
  double error = setP - dis;

  double Pvalue = error * kp;
  double Ivalue = toError * ki;
  double Dvalue = (error - priError) * kd;

  double PIDvalue = Pvalue + Ivalue + Dvalue;
  priError = error;
  toError += error;

  int Fvalue = (int)PIDvalue;
  Fvalue = map(Fvalue, -135, 135, 135, 0);

  if (Fvalue < 0) {
    Fvalue = 0;
  }
  if (Fvalue > 135) {
    Fvalue = 135;
  }

  servo.write(Fvalue);

  // Print distance and PID value
  Serial.print("Distance: ");
  Serial.print(dis);
  Serial.print(" cm | PID Output (mapped): ");
  Serial.println(Fvalue);
}
