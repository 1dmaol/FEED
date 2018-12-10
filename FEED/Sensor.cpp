#include "Sensor.h"

// ======CONSTRUCTOR======
Sensor::Sensor(int _pin, double _max_valor, double _min_valor, Adafruit_ADS1115 _ads){
  pin = _pin;
  max_valor = _max_valor;
  min_valor = _min_valor;
  ads1115 = _ads;
  power_pin = 0;
  value = 0;
}

Sensor::Sensor(int _pin, double _max_valor, double _min_valor, double _power_pin, Adafruit_ADS1115 _ads){
  pin = _pin;
  max_valor = _max_valor;
  min_valor = _min_valor;
  power_pin = _power_pin;
  ads1115 = _ads;
  value = 0;
}

Sensor::Sensor(SFE_BMP180 _bmp180){
  bmp180 = _bmp180;
}

Sensor::Sensor(){
}

// ========METODOS========

void Sensor::leerInterruptor() {
 digitalWrite( power_pin, HIGH ); // Activamos el sensor // @suppress("Invalid arguments")
 delay(100); // Esperamos para la lectura
 int lectura = ads1115.readADC_SingleEnded(pin); // @suppress("Invalid arguments")
 digitalWrite( power_pin, LOW ); // Desactivamos el sensor // @suppress("Invalid arguments")
 value = 100 * min_valor / (min_valor - max_valor)
      - lectura * 100 / (min_valor - max_valor);
}

void Sensor::leerPresion() {
  char status;
  double T, P, p0;

    status = bmp180.startTemperature();//Inicio de lectura de temperatura
    if (status != 0) {
      delay(status); //Pausa para que finalice la lectura
      status = bmp180.getTemperature(T); //Obtener la temperatura
      if (status != 0) {
        status = bmp180.startPressure(3); //Inicio lectura de presion
          if (status != 0) {
            delay(status);//Pausa para que finalice la lectura
            status = bmp180.getPressure(P,T); //Obtenemos la presion
            p0 = bmp180.sealevel(P, 22);
            mostrar(status, p0);
          }
      }
    }
}
void Sensor::leerTemperatura(){
  float a=0.76;
  float b=0.036;
  float c=0;
  float d=8000;
  leer();
  value=(value-c-(d*a))/(d*b);
  Serial.print("Temperatura de la prueba: "); Serial.print(value); Serial.println(" C");
}

void Sensor::leer() {
  value = ads1115.readADC_SingleEnded(pin); // @suppress("Invalid arguments")
  Serial.print("Lectura de la prueba: "); Serial.println(value);
}
void Sensor::convertirPorcentaje(){
  value = 100 * min_valor / (min_valor - max_valor) - value*100 / (min_valor - max_valor);
}

void Sensor::mostrar(char status, double p0) {
  if (value > 100)
    value = 100;
  if (value < 0)
    value = 0;
  Serial.print("La presion: ");
  Serial.print(p0);
}

void Sensor::mostrar() {
  if (value > 100)
    value = 100;
  if (value < 0)
    value = 0;
  Serial.print(" - Porcentaje: ");
  Serial.print(value);
  Serial.println(" %");
}

void Sensor::mostrarIluminacion() {
 if (value>=0 && value<5  ){
    Serial.println("Esta oscuro");
  }
  else if (value>=5 && value<50){
    Serial.println("Dia nublado");
  }
  else if (value>=50 && value<85){
    Serial.println("Dia soleado");
  }
  else if (value>=85 && value<=100){
    Serial.println("Dia muy soleado");
  }
}
