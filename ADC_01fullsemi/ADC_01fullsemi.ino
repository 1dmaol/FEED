#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <Sensores.h>

Adafruit_ADS1115 ads1115(0x48); // construct an ads1115 at address 0x48

// CONSTANTES DE LA HUMEDAD
const int MIN_HUMEDAD = 20475;  // Medimos el mínimo valor de humedad (valor en seco)
const int MAX_HUMEDAD = 14720;  // Medimos el máximo valor de humedad (valor en agua)
const int PIN_ANALOG_HUMEDAD = 0; // Pin de entrada analogica del dispositivo para la humedad

// CONSTANTES DE LA SALINIDAD
const int MAX_SALINIDAD = 2780; //Medimos valor en la maxima salinidad
//const int MinimaSalinidad = 771; //Medimos el valor en la minima salinidad (agua normal)
const int MIN_SALINIDAD = 5; //Medimos el valor de sal en aire, por defecto el minimo
const int PIN_ENERGIA_SALINIDAD = 5;
const int PIN_ANALOG_SALINIDAD = 1; // Pin de entrada analogica del dispositivo para la salinidad

void setup() {
  Serial.begin(9600);
  Serial.println("Inicializando...");
  ads1115.begin(); //Iniciar ads1115
  Serial.println("Ajustando la ganancia...");
  ads1115.setGain(GAIN_ONE);
  Serial.println("Tomando medidas del canal AIN0");
  Serial.println("Rango del ADC: +/- 4.096V (1 bit=2mV)");
}

void loop() {
  // Control de la salinidad
  // Muestra por pantalla los datos recogidos del sensor de salinidad.
  lectorSalinidad(MAX_SALINIDAD, MIN_SALINIDAD, PIN_ENERGIA_SALINIDAD, PIN_ANALOG_SALINIDAD);

  // Control de la humedad
  // Muestra por pantalla los datos recogidos del sensor de humedad.
  lectorHumedad(MAX_HUMEDAD, MIN_HUMEDAD, PIN_ENERGIA_HUMEDAD);

  //Refresco de los datos
  delay(500);
}
