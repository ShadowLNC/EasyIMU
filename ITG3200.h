#ifndef ITG3200_h
#define ITG3200_h

#include <Arduino.h>

class ITG3200 {
public:
  const static int XAxis = 0;
  const static int YAxis = 1;
  const static int ZAxis = 2;
  const static int GYRO_ADDR = 0b1101000; //0x68
  
  ITG3200();
  double getRotation(int axis);
  double getAngle(int axis);
  void calibrate0();
  void calibrate360(); 
  void zero();
  void update();

protected:
  double* gyro;
  double* angle;
  double* offset;
  double* multiplier;
  unsigned long prevtime;
  //I2C gyro_dev;
};

#endif

