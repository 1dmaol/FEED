#include <Wire.h>
#include <Adafruit_ADS1015.h>

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
  //Control de la salinidad
  lectorSalinidad( );

  //Control de la humedad
  lectorHumedad( );

  //Refresco de los datos
  delay(500);
}

// -------Lector de humedad-------
// Muestra por pantalla los datos recogidos del sensor de humedad.

void lectorHumedad( ){
  // Control y lectura de datos
  int lectura;
  lectura = ads1115.readADC_SingleEnded(PIN_ANALOG_HUMEDAD);
  int humedad = 100*MIN_HUMEDAD/(MIN_HUMEDAD-MAX_HUMEDAD)-lectura*100/(MIN_HUMEDAD-MAX_HUMEDAD);

  // Mostrar por pantalla
  Serial.println("=== HUMEDAD ===");
  Serial.print("Humedad de la prueba: "); Serial.println(lectura);
  Serial.print("Rango de humedad (%): "); Serial.print(humedad>100?humedad=100:humedad); Serial.println("%");
}

// -------Lector de salinidad-------
// Muestra por pantalla los datos recogidos del sensor de salinidad.

void lectorSalinidad( ) {
  // Control y lectura de datos
  int lectura;
  digitalWrite( PIN_ENERGIA_SALINIDAD, HIGH ); // Activamos el sensor
  delay(100); // Esperamos para la lectura
  lectura = ads1115.readADC_SingleEnded(PIN_ANALOG_SALINIDAD);
  digitalWrite( PIN_ENERGIA_SALINIDAD, LOW ); // Desactivamos el sensor
  int salinidad = 100*MIN_SALINIDAD/(MIN_SALINIDAD-MAX_SALINIDAD)-lectura*100/(MIN_SALINIDAD-MAX_SALINIDAD);

  // Mostrar por pantalla
  Serial.println("=== SALINIDAD ===");
  Serial.print("La salinidad de la prueba es: "); Serial.println(lectura);
  Serial.print("El porcentaje de sal es: "); Serial.print(salinidad>100?salinidad=100:salinidad); Serial.println("%");
}
