#include "Arduino.h"
#include <Wire.h>
#include "Sensor.h"
#include "WifiConnection.h"

int contador = 0;
/******************************************************************************
Constantes
******************************************************************************/
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
const int NEO_MIN_TEMPERATURA = 0;
const int MAX_TEMPERATURA = 17600;
const int NEO_MAX_TEMPERATURA = 40;

//CONSTANTES DE ILUMINACION
const int PIN_ANALOG_ILUMINACION = 3;
const int MIN_ILUMINACION = 70;
const int MAX_ILUMINACION = 4500;
/*const int sunny=4500;
const int cloudy=450;
const int lighcloudy=700;
const int fulldarkness=108;*/


/******************************************************************************
ACÉLEROMETRO 
******************************************************************************/

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C
 
#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18
 
#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define PWR_MGMT_1              0x6B
#define PWR_MGMT_2              0x6C
#define ACCEL_CONFIG_2          0x1D
#define INT_ENABLE              0x38 
#define MOST_DETECT_CTRL         0x69
#define WOM_THR                 0x1F
#define LP_ACCEL_ODR            0x1E
#define interruptPin 4 //pin de interrupcion -> 4

int numOfInterrupts=0;
volatile byte interruptCounter = 0;

//Funcion auxiliar lectura
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
   Wire.beginTransmission(Address);
   Wire.write(Register);
   Wire.endTransmission();
 
   Wire.requestFrom(Address, Nbytes);
   uint8_t index = 0;
   while (Wire.available())
      Data[index++] = Wire.read();
}
 
 
// Funcion auxiliar de escritura
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
   Wire.beginTransmission(Address);
   Wire.write(Register);
   Wire.write(Data);
   Wire.endTransmission();
}
 
void handleInterrupt(){
   interruptCounter++;
}

void configurarAcelerometro()
{
  Wire.begin();
  Serial.println("Configurando acelerómetro...");
   // Configurar acelerometro
   I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_16_G);

   // Configurar interrupciones
   I2CwriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0);
   I2CwriteByte(MPU9250_ADDRESS, PWR_MGMT_2, 7);
   I2CwriteByte(MPU9250_ADDRESS,ACCEL_CONFIG_2 ,5 );
   I2CwriteByte(MPU9250_ADDRESS,INT_ENABLE ,64 );
   I2CwriteByte(MPU9250_ADDRESS,0x37 ,128 ); //pin CFG
   I2CwriteByte(MPU9250_ADDRESS,MOST_DETECT_CTRL ,192 );
   I2CwriteByte(MPU9250_ADDRESS,LP_ACCEL_ODR ,1 ); //es la frecuencia
   I2CwriteByte(MPU9250_ADDRESS,WOM_THR ,2 ); //es la sensibilidad
   I2CwriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 32); //2^5

   pinMode(interruptPin, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, FALLING);
/*   // Configurar giroscopio
     Serial.println("Configurando giroscopio...");
   I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS);
   // Configurar magnetometro
   
    Serial.println("Configurando magnetómetro...");
   I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);
   I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01);
   */
}
 
 
void lecturaAcelerometro()
{
  // Interrupciones
   if (interruptCounter>0){

               interruptCounter--;   //para que vuelva a cero

               numOfInterrupts++;

       }
  
  // ---  Lectura acelerometro y giroscopio --- 
   uint8_t Buf[14];
   int FS_ACC = 16;
   int FS_GYRO = 2000;

   I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
 
//   // Convertir registros acelerometro
   float ax = (Buf[0] << 8 | Buf[1]);
   float ay = (Buf[2] << 8 | Buf[3]);
   float az = Buf[4] << 8 | Buf[5];

   ax = (ax*FS_ACC/32768) - 25.57;
   ay = (ay*FS_ACC/32768) - 31.78;
   az = (az*FS_ACC/32768) + 0.94;
 
 /* // Convertir registros giroscopio
   float gx = (Buf[8] << 8 | Buf[9]);
   float gy = (Buf[10] << 8 | Buf[11]);
   float gz = Buf[12] << 8 | Buf[13];
 
   gx = gx*FS_GYRO/32768;
   gy = gy*FS_GYRO/32768;
   gz = gz*FS_GYRO/32768;
// 
// 

//   // ---  Lectura del magnetometro --- 
//   uint8_t ST1;
//   do
//   {
//      I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
//   } while (!(ST1 & 0x01));
// 
//   uint8_t Mag[7];
//   I2Cread(MAG_ADDRESS, 0x03, 7, Mag);
// 
// 
//   // Convertir registros magnetometro
//   float mx = -(Mag[3] << 8 | Mag[2]);
//   float my = -(Mag[1] << 8 | Mag[0]);
//   float mz = -(Mag[5] << 8 | Mag[4]);
// 
// */
//   // --- Mostrar valores ---
// 

//   // Acelerometro
   Serial.println("Lectura Acelerometro");
   Serial.print("AX=");
   Serial.print(ax, 2);
   Serial.print("g");
   Serial.print("\t");
   Serial.print("AY=");
   Serial.print(ay, 2);
   Serial.print("g");
   Serial.print("\t");
   Serial.print("AZ=");
   Serial.print(az, 2);
   Serial.println("g");
//
// 
/*//   // Giroscopio
   Serial.println("Lectura Giroscopio");
   Serial.print("GX=");
   Serial.print(gx, 2);
   Serial.print("º/sec");
   Serial.print("\t");
   Serial.print("GY=");
   Serial.print(gy, 2);
   Serial.print("º/sec");
   Serial.print("\t");
   Serial.print("GZ=");
   Serial.print(gz, 2);
   Serial.println("º/sec");

// 
// 
//   // Magnetometro
//   Serial.println("Lectura Magnetometro");
//   Serial.print("MX=");
//   Serial.print(mx, DEC);
//   Serial.print("uT");
//   Serial.print("\t");
//   Serial.print("MY=");
//   Serial.print(my, DEC);
//   Serial.print("uT");
//   Serial.print("\t");
//   Serial.print("MZ=");
//   Serial.print(mz, DEC);
//   Serial.println("uT");
//
*/
//   
//   // Fin medicion
//   Serial.println("");
//       
}

/******************************************************************************
GPS A2235H 
Conectar RXI al pin 12 del ESP8266
Conectar TXO al pin 13 del ESP8266
Usamos el pim 15 para inicializar el GPS
ATENCION: no poner el GPS sobre el Sparkfun 
******************************************************************************/

#include <TinyGPS++.h>  //Librería del GPS
#include <SoftwareSerial.h>

#define RX_PIN  12 // GPS RXI
#define TX_PIN  13 // GPS TX0
#define INIT_PIN 15 // Pin para  Inicializar el GPS

#define GPS_BAUD  4800  //  velocidad de comunicación serie 

TinyGPSPlus gps; // Definimos el objeto gps

SoftwareSerial ss(RX_PIN,TX_PIN); // Creamos una UART software para comunicación con el GPS


// Función espera 1s para leer del GPS
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while(ss.available())
    {
      gps.encode(ss.read());  // leemos del gps
    }
  } while(millis() - start < ms);
}
 
// Función para encender/apagar mediante un pulso
void switch_on_off()
{
   // Power on pulse
  digitalWrite(INIT_PIN,LOW);
  delay(200);
  digitalWrite(INIT_PIN,HIGH);
  delay(200); 
  digitalWrite(INIT_PIN,LOW);
}

void lecturaGPS(){
  char gpsDate[10]; 
  char gpsTime[10];
  if(gps.location.isValid()){ // Si el GPS está recibiendo los mensajes NMEA

    Serial.println("Fecha      Hora       Latitud   Longitud   Alt    Rumbo   Velocidad");
    Serial.println("(MM/DD/YY) (HH/MM/SS)     (deg)       (deg)  (ft)                   (mph)");
    Serial.println("-------------------------------------------------------------------------"); 
  
    sprintf(gpsDate,"%d/%d/%d", gps.date.month(),gps.date.day(),gps.date.year()); // Construimos string de datos fecha
    sprintf(gpsTime,"%d/%d/0%d", gps.time.hour(),gps.time.minute(),gps.time.second());  // Construimos string de datos hora

    Serial.print(gpsDate);
    Serial.print('\t');
    Serial.print(gpsTime);
    Serial.print('\t');
    Serial.print(gps.location.lat(),6);
    Serial.print('\t');
    Serial.print(gps.location.lng(),6);
    Serial.print('\t');
    Serial.print(gps.altitude.feet());
    Serial.print('\t');
    Serial.print(gps.course.deg(),2);
    Serial.print('\t');
    Serial.println(gps.speed.mph(),2);
  } else { // Si no recibe los mensajes
    Serial.print("Satellites in view: ");
    Serial.println(gps.satellites.value());
  }
  smartDelay(1000);
}

void conectarGPS(){
   ss.begin(GPS_BAUD); // Inicializar la comunicación con el GPS
 
  pinMode(INIT_PIN,OUTPUT); 
  switch_on_off(); // Pulso para encender el GPS
}

/******************************************************************************
Sensores
******************************************************************************/
// VARIABLES SENSORES
Adafruit_ADS1115 ads1115(0x48);
SFE_BMP180 bmp180;
Sensor salinidad;
Sensor humedad;
Sensor temperatura;
Sensor presion;
Sensor iluminacion;
WifiConnection wifi;

void construirSensores(){
  Sensor _salinidad(PIN_ANALOG_SALINIDAD, MAX_SALINIDAD, MIN_SALINIDAD, PIN_ENERGIA_SALINIDAD, ads1115);
  salinidad = _salinidad;
  Sensor _humedad(PIN_ANALOG_HUMEDAD, MAX_HUMEDAD, MIN_HUMEDAD, ads1115);
  humedad = _humedad;
  Sensor _temperatura(PIN_ANALOG_TEMPERATURA, NEO_MAX_TEMPERATURA, NEO_MIN_TEMPERATURA, ads1115);
  temperatura = _temperatura;
  Sensor _presion(bmp180);
  presion = _presion;
  Sensor _iluminacion(PIN_ANALOG_ILUMINACION, MAX_ILUMINACION, MIN_ILUMINACION, ads1115);
  iluminacion = _iluminacion;
}

void conectarPresion(){
  if (bmp180.begin()) {
    Serial.println("BMP180 iniciado correctamenten");
  } else {
    Serial.println("Error al iniciar el BMP180");
  }
}

/******************************************************************************
Setup
******************************************************************************/
void setup() {
  Serial.begin(9600);
  Serial.println("Inicializando...");
  ads1115.begin(); //Iniciar ads1115
  Serial.println("Ajustando la ganancia...");
  ads1115.setGain(GAIN_ONE);
  Serial.println("Tomando medidas del canal AIN0");
  Serial.println("Rango del ADC: +/- 4.096V (1 bit=2mV)");

  construirSensores();

  conectarPresion();
  
  conectarGPS();

configurarAcelerometro(); 
  
  Serial.println("Inicializamos el dispositivo wifi...");

  wifi.connectWiFi();
}

/******************************************************************************
Funcion para controlar los sensores
******************************************************************************/
void leerDatos() {
  // Control de la humedad
  // Muestra por pantalla los datos recogidos del sensor de humedad.
  Serial.println("\n=== HUMEDAD ===");
  humedad.leer();
  humedad.convertirPorcentaje();
  humedad.mostrar();
  
  // Control de la salinidad
  // Muestra por pantalla los datos recogidos del sensor de salinidad.
  Serial.println("\n=== SALINIDAD ===");
  salinidad.leerInterruptor();
  salinidad.convertirPorcentaje();
  salinidad.mostrar();

  // Control de temperatura
  // Muestra por pantalla los datos recogidos del sensor de temperatura.
  Serial.println("\n=== TEMPERATURA ===");
  temperatura.leerTemperatura();
  temperatura.convertirPorcentaje();
  temperatura.mostrar();

  // Control de presion
  // Muestra por pantalla los datos recogidos del sensor de presion.
  Serial.println("\n=== PRESION === (NOT IMPLEMENTED)");
  
  // Control de iluminación
  // Muestra por pantalla los datos recogidos del sensor de iluminación.
  Serial.println("\n=== ILUMINACION ===");
  iluminacion.leer();
  iluminacion.convertirPorcentaje();
  iluminacion.mostrar();
  iluminacion.mostrarIluminacion();
  
  // Acelerómetro
  // Muestra por pantalla los datos recogidos del GPS.
  Serial.println("\n=== ACELERÓMETRO ===");
  lecturaAcelerometro();

  // GPS
  // Muestra por pantalla los datos recogidos del GPS.
  Serial.println("\n=== GPS ===");
  lecturaGPS();

}

void loop() {
  Serial.print("\n\nPrueba ");
  Serial.print(contador++);
  if (wifi.conexion){
    Serial.println("\t\t\tConectado a GTI1! ");
  }else {
    Serial.println("");
  }
  leerDatos();
  //Refresco de los datos
  delay(1000);
}
