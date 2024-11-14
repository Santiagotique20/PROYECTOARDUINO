#include "I2Cdev.h"
#include "MPU6050.h"
#include <Wire.h>
#include <WiFi.h>         // Librería para conexión WiFi
#include <HTTPClient.h>   // Librería para hacer solicitudes HTTP

// Configuración de red WiFi
const char* ssid = "Santiago20";        // Reemplaza con tu SSID de WiFi
const char* password = "Santiago2020"; // Reemplaza con tu contraseña de WiFi

// Configuración de ThingSpeak
const char* server = "https://api.thingspeak.com/update?api_key=VNYZOWMLE216VXJ9&field1=0";
const char* apiKey = "V4HYKA5AF1RUFC0H";   // Reemplaza con tu clave de API de ThingSpeak

// Definir pines
const int pinSensorSonido = A16;     // Pin analógico donde está conectado el sensor de sonido
int valorSonido = 16;                // Variable para almacenar el valor leído del sensor

// Valores RAW del acelerómetro en los ejes x, y, z
MPU6050 sensor;
int16_t ax, ay, az;

void setup() {
  // Inicializar el puerto serial para mostrar datos
  Serial.begin(57600);

  // Conectar a la red WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConectado a WiFi");

  // Inicializar I2C y el sensor MPU6050
  Wire.begin();
  sensor.initialize();

  // Verificar la conexión con el sensor
  if (sensor.testConnection()) Serial.println("Sensor MPU6050 iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");

  // Mostrar mensaje de inicio
  Serial.println("Iniciando sensor de sonido...");
}

void loop() {
  // Leer el valor del sensor de sonido
  valorSonido = analogRead(pinSensorSonido);
  
  // Mostrar el valor del sensor de sonido en el monitor serial
  Serial.print("Nivel de sonido: ");
  Serial.println(valorSonido);

  // Leer las aceleraciones del sensor MPU6050
  sensor.getAcceleration(&ax, &ay, &az);
  
  // Calcular los ángulos de inclinación
  float accel_ang_x = atan(ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);
  float accel_ang_y = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14);
  
  // Mostrar los ángulos de inclinación
  Serial.print("Inclinación en X: ");
  Serial.print(accel_ang_x);
  Serial.print("\tInclinación en Y: ");
  Serial.println(accel_ang_y);

  // Enviar datos a ThingSpeak
  if (WiFi.status() == WL_CONNECTED) { // Verifica si la conexión WiFi está activa
    HTTPClient http;
    String url = String(server) + "/update?api_key=" + apiKey + "&field1=" + String(valorSonido) + "&field2=" + String(accel_ang_x) + "&field3=" + String(accel_ang_y);
    http.begin(url);           // Inicia la conexión HTTP
    int httpResponseCode = http.GET(); // Realiza la solicitud GET

    if (httpResponseCode > 0) {
      Serial.print("Datos enviados a ThingSpeak: ");
      Serial.println(httpResponseCode);
    } else {
      Serial.print("Error al enviar datos: ");
      Serial.println(httpResponseCode);
    }
    http.end(); // Termina la conexión
  } else {
    Serial.println("Error de conexión WiFi");
  }

  // Pequeño retardo para evitar envíos muy rápidos (cada 15 segundos mínimo para ThingSpeak)
  delay(20000);
}
