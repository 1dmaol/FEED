#ifndef SENSORES_H
#define SENSORES_H

#include "Arduino.h"

class Sensor {
  private:
    int pin;
  public:
    Sensor();
    Sensor(int);
    void lectorSalinidad(int, int, int, int, int);
    void lectorHumedad(int, int, int, int);
    void lectorTemperatura(int, int, int);
    void setPin(int);
    int getPin();
};

#endif
