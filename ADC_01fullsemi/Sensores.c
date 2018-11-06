// -------Lector de salinidad-------
// Muestra por pantalla los datos recogidos del sensor de salinidad.
void lectorSalinidad(int MAX_SALINIDAD, int MIN_SALINIDAD, int PIN_ENERGIA_SALINIDAD, int PIN_ANALOG_SALINIDAD) {
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


// -------Lector de humedad-------
// Muestra por pantalla los datos recogidos del sensor de humedad.
void lectorHumedad(int MAX_HUMEDAD, int MIN_HUMEDAD, int PIN_ANALOG_HUMEDAD){
  // Control y lectura de datos
  int lectura;
  lectura = ads1115.readADC_SingleEnded(PIN_ANALOG_HUMEDAD);
  int humedad = 100*MIN_HUMEDAD/(MIN_HUMEDAD-MAX_HUMEDAD)-lectura*100/(MIN_HUMEDAD-MAX_HUMEDAD);

  // Mostrar por pantalla
  Serial.println("=== HUMEDAD ===");
  Serial.print("Humedad de la prueba: "); Serial.println(lectura);
  Serial.print("Rango de humedad (%): "); Serial.print(humedad>100?humedad=100:humedad); Serial.println("%");
}
