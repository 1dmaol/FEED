#include <Wire.h>
#include "Sensor.h"

int pin;

// ---------------------------------
Sensor::Sensor(int _pin){
  pin = _pin;  
}

// ---------------------------------
Sensor::Sensor(){
  pin = -1;  
}

void Sensor::setPin(int _pin){
  pin = _pin;
}

int Sensor::getPin(){
  return pin;
}

// -------Lector de salinidad-------
// Muestra por pantalla los datos recogidos del sensor de salinidad.
void Sensor::lectorSalinidad(int MAX_SALINIDAD, int MIN_SALINIDAD, int PIN_ENERGIA_SALINIDAD, int PIN_ANALOG_SALINIDAD, int lectura) {
  // Control y lectura de datos
  int salinidad = 100*MIN_SALINIDAD/(MIN_SALINIDAD-MAX_SALINIDAD)-lectura*100/(MIN_SALINIDAD-MAX_SALINIDAD);

  // Mostrar por pantalla
  Serial.println("=== SALINIDAD ===");
  Serial.print("La salinidad de la prueba es: "); Serial.println(lectura);
  Serial.print("El porcentaje de sal es: "); Serial.print(salinidad>100?salinidad=100:salinidad); Serial.println("%");
}


// -------Lector de humedad-------
// Muestra por pantalla los datos recogidos del sensor de humedad.
void Sensor::lectorHumedad(int MAX_HUMEDAD, int MIN_HUMEDAD, int PIN_ANALOG_HUMEDAD, int lectura){
  // Control y lectura de datos
  int humedad = 100*MIN_HUMEDAD/(MIN_HUMEDAD-MAX_HUMEDAD)-lectura*100/(MIN_HUMEDAD-MAX_HUMEDAD);

  // Mostrar por pantalla
  Serial.println("=== HUMEDAD ===");
  Serial.print("Humedad de la prueba: "); Serial.println(lectura);
  Serial.print("Rango de humedad (%): "); Serial.print(humedad>100?humedad=100:humedad); Serial.println("%");
}


// -------Lector de temperatura-------
// Muestra por pantalla los datos recogidos del sensor de humedad.
void Sensor::lectorTemperatura(int MAX_TEMPERATURA, int MIN_TEMPERATURA, int lectura){
  // Control y lectura de datos
  float T;
  float a=0.76;
  float b=0.036;
  float c=0;
  float d=8000;
  T=(lectura-c-(d*a))/(d*b);
  int temperatura = 100*MIN_TEMPERATURA/(MIN_TEMPERATURA-MAX_TEMPERATURA)-lectura*100/(MIN_TEMPERATURA-MAX_TEMPERATURA);

  // Mostrar por pantalla
  Serial.print("=== TEMPERATURA (");
  Serial.print(lectura);
  Serial.println(") ===");
  Serial.print("Temperatura de la prueba: "); Serial.print(T); Serial.println(" C");
  Serial.print("Rango de temperatura (%): "); Serial.print(temperatura>100?temperatura=100:temperatura); Serial.println("%");
  delay(1000);
}
