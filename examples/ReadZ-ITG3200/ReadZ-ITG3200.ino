
#include <ITG3200.h>
#include <Wire.h>

ITG3200* gyro;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  gyro = new ITG3200();
}

void loop() {
  gyro->update();
  Serial.println(gyro->getAngle(2));
}


