#ifndef SENSORES_H
#define SENSORES_H

#include "Arduino.h"
#include <Adafruit_ADS1015.h>
#include <SFE_BMP180.h>

class Sensor {
  private:
    int pin;
    double min_valor;
    double power_pin;
    int value;
    double max_valor;
    Adafruit_ADS1115 ads1115;
    SFE_BMP180 bmp180;
  public:
    Sensor(int, double, double, double, Adafruit_ADS1115);
    Sensor(int, double, double, Adafruit_ADS1115);
    Sensor(SFE_BMP180);
    Sensor();
    void mostrar();
    void mostrarIluminacion();
    void mostrar(char, double);
    void leerInterruptor();
    void leerPresion();
    void leer();
    void leerTemperatura();
    void convertirPorcentaje();
};

#endif
