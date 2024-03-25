#include <Arduino.h>

double thrust;
double pitch;
double yaw;
double roll;

int flport = 0;
int frport = 0;
int blport = 0;
int brport = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(frport, OUTPUT);
  pinMode(flport, OUTPUT);
  pinMode(brport, OUTPUT);
  pinMode(blport, OUTPUT);
  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  double frontRight = thrust + yaw + pitch + roll;
  double frontLeft = thrust - yaw + pitch - roll;
  double backRight = thrust - yaw - pitch + roll;
  double backLeft = thrust + yaw - pitch - roll;

  analogWrite(frport, frontRight);
  analogWrite(flport, frontLeft);
  analogWrite(brport, backRight);
  analogWrite(blport, backLeft);
  
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}