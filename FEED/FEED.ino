#include "Arduino.h"
#include <Wire.h>
#include "Sensor.h"

int contador = 0;
// CONSTANTES DE LA HUMEDAD
const int MIN_HUMEDAD = 20475; // Medimos el mÃ­nimo valor de humedad (valor en seco)
const int MAX_HUMEDAD = 14720; // Medimos el mÃ¡ximo valor de humedad (valor en agua)
const int PIN_ANALOG_HUMEDAD = 0; // Pin de entrada analogica del dispositivo para la humedad
// CONSTANTES DE LA SALINIDAD
const int MAX_SALINIDAD = 2780; //Medimos valor en la maxima salinidad
const int MIN_SALINIDAD = 5; //Medimos el valor de sal en aire, por defecto el minimo
const int PIN_ENERGIA_SALINIDAD = 5;
const int PIN_ANALOG_SALINIDAD = 1; // Pin de entrada analogica del dispositivo para la salinidad

//CONSTANTES DE LA TEMPERATURA
const int PIN_ANALOG_TEMPERATURA = 2; //Pin de entrada analogica del dispositivo para la temperatura
const int MIN_TEMPERATURA = 6517;
const int MAX_TEMPERATURA = 17600;

// CONSTANTES DE LA PRESION
const int ALTITUDE = 2200;

// VARIABLES SENSORES
Adafruit_ADS1115 ads1115(0x48);
SFE_BMP180 bmp180;
Sensor salinidad(PIN_ANALOG_SALINIDAD, MIN_SALINIDAD, MAX_SALINIDAD,
    PIN_ENERGIA_SALINIDAD, ads1115);
Sensor humedad(PIN_ANALOG_HUMEDAD, MIN_HUMEDAD, MAX_HUMEDAD, ads1115);
Sensor temperatura(PIN_ANALOG_TEMPERATURA, MIN_TEMPERATURA, MAX_TEMPERATURA,
    ads1115);
Sensor presion(bmp180);

void setup() {
  Serial.begin(9600);
  Serial.println("Comienzo...");
  ads1115.begin(); //Iniciar ads1115
  ads1115.setGain(GAIN_ONE);
  Serial.println("Inicializamos el dispositivo wifi...");

  if (bmp180.begin()) {
    Serial.println("BMP180 iniciado correctamenten");
  } else {
    Serial.println("Error al iniciar el BMP180");
  }
}

void leerDatos() {
  // Control de la humedad
  // Muestra por pantalla los datos recogidos del sensor de humedad.
  Serial.print("\n\tSensor de humedad. ");
  humedad.leer();
  humedad.mostrar();

  // Control de la salinidad
  // Muestra por pantalla los datos recogidos del sensor de salinidad.
  Serial.print("\n\tSensor de salinidad. ");
  salinidad.leerInterruptor();
  salinidad.mostrar();

  // Control de temperatura
  // Muestra por pantalla los datos recogidos del sensor de temperatura.
  Serial.print("\n\tSensor de temperatura. ");
  temperatura.leer();
  temperatura.mostrar();

  // Control de presion
  // Muestra por pantalla los datos recogidos del sensor de presion.
  Serial.print("\n\tSensor de presion. ");
  presion.leerPresion();
}

void loop() {
  Serial.print("\n\nPrueba ");
  Serial.println(contador++);
  leerDatos();
  //Refresco de los datos
  delay(1000);
}