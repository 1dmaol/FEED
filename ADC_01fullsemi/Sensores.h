#ifndef Sensores_h
#define Sensores_h

class Sensores
{
  public:
    Sensores(int pin);
    void lectorSalinidad(int MAX_SALINIDAD, int MIN_SALINIDAD, int PIN_ENERGIA_SALINIDAD, int PIN_ANALOG_SALINIDAD);
    void lectorHumedad(int MAX_HUMEDAD, int MIN_HUMEDAD, int PIN_ANALOG_HUMEDAD);
};

#endif
