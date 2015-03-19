#include "ITG3200.h"
#include <Wire.h>

ITG3200::ITG3200() {
  gyro = new double[3];
  angle = new double[3];
  offset = new double[3];
  multiplier = new double[3];
  for (int i=0; i<3; ++i) {
    multiplier[i]=1;
  }
  //gyro_dev = new I2C(I2C.Port.kOnboard, GYRO_ADDR);

  Wire.beginTransmission(ITG3200::GYRO_ADDR);
  Wire.write((byte)0x16); //0x16 is DLPF_FS: DLP,Full Scale
  Wire.write((byte)0b00011000);
  //000 unused
  //11 is +-2000 deg/sec
  //000 is 8kHz with 256Hz low pass
  Wire.endTransmission();

  Wire.beginTransmission(ITG3200::GYRO_ADDR);
  Wire.write((byte)0x3E); //0x3E is PWR_MGM: Power Management
  Wire.write((byte)0b00000011);
  //0 is don't reset
  //0 is don't sleep
  //000 is XYZ active (don't standby)
  //011 is Z-Axis Clock
  Wire.endTransmission();

  prevtime = micros();
}

double ITG3200::getRotation(int axis) {
  return gyro[axis];
}

double ITG3200::getAngle(int axis) {
  return angle[axis];
}

void ITG3200::calibrate0() {
  zero();
  unsigned long stoptime = millis() + 10000;
  while (millis()<stoptime) {
    this->update();
  }
  for (int i=0; i<3; ++i) {
    //divide by multiplier to get raw value, then divide by for offset
    offset[i] += (angle[i]/multiplier[i])/(double)10;
  }
  zero();
}

void ITG3200::calibrate360() {
  //z-only :P
  //multiplier*angle=360, hence multiplier=360/angle
  //assume multiplier positive, regardless of which way we spun it around
  multiplier[2] = abs((double)360/angle[2]);
  zero();
}

void ITG3200::zero() {
  for (int i=0; i<3; ++i) {
    angle[i] = 0;
  }
}

void ITG3200::update() {
  byte temp_byte[6];

  //low byte first for gyro
  Wire.beginTransmission(ITG3200::GYRO_ADDR);
  Wire.write(0x1D);
  Wire.endTransmission();
  Wire.requestFrom(ITG3200::GYRO_ADDR, 6);
  
  for (int i=0; i<6; ++i) {
    while (!Wire.available());
    temp_byte[i] = Wire.read();
  }
  //clear buffer
  while (Wire.available()) {
    Wire.read();
  }
  
  const double divisor = (double)32768/(double)2000;
  for (int i=0; i<3; ++i) {
    int temp = (temp_byte[2*i]<<8) | (temp_byte[2*i+1]);
    gyro[i] = (double)temp/divisor;
    gyro[i] -= offset[i];
    gyro[i] *= multiplier[i];
  }

  unsigned long time = micros();
  double weight = (double)(time-prevtime)/(double)pow(10, 6);
  prevtime = time;

  //accumulative gyroscope angle
  for (int i=0; i<3; ++i) {
    angle[i] += (weight*gyro[i]);
  }
}

