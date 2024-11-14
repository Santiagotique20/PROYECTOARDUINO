#include <WiFi.h>
#include <PubSubClient.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// Configuración de la red Wi-Fi
const char* ssid = "Xiaomi 11T";  // Cambia esto por el nombre de tu red WiFi
const char* password = "Sofia2019";  // Cambia esto por la contraseña de tu red WiFi

// Configuración del broker EMQX
const char* mqtt_server = "192.168.112.115";  // Cambia esto por la IP o el dominio de tu broker EMQX
const int mqtt_port = 8083;  // Puerto MQTT (1883 es el puerto por defecto para MQTT sin SSL)
const char* mqtt_user = "";  // Usuario MQTT (si es necesario)
const char* mqtt_password = "";  // Contraseña MQTT (si es necesario)

// Cliente WiFi y MQTT
WiFiClient espClient;
PubSubClient client(espClient);


// Pines del sensor de sonido
const int pinSensorSonido = A4;  // Pin analógico donde está conectado el sensor de sonido
int valorSonido = 0;  // Variable para almacenar el valor leído del sensor de sonido

// Sensor MPU6050
MPU6050 sensor;
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Función para conectarse a Wi-Fi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("Conectado a WiFi");
  Serial.println("Dirección IP: ");
  Serial.println(WiFi.localIP());
}

// Función para reconectar al broker MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("Conectado al broker MQTT");
    } else {
      Serial.print("Fallo, rc=");
      Serial.print(client.state());
      Serial.println(" intentando de nuevo en 5 segundos");
      delay(5000);
    }
  }
}

// Función para enviar un mensaje MQTT
void sendMessage(const char* topic, const char* message) {
  if (client.connected()) {
    client.publish(topic, message);
    Serial.println("Mensaje enviado:");
    Serial.print("Tópico: ");
    Serial.println(topic);
    Serial.print("Mensaje: ");
    Serial.println(message);
  } else {
    Serial.println("Error: No conectado al broker MQTT.");
  }
}

void setup() {
  // Inicializar el puerto serial
  Serial.begin(115200);

  // Inicializar la conexión Wi-Fi
  setup_wifi();
  
  // Configurar MQTT
  client.setServer(mqtt_server, mqtt_port);

  // Inicializar el sensor MPU6050 y el bus I2C
  Wire.begin();
  sensor.initialize();

  // Verificar la conexión con el MPU6050
  if (sensor.testConnection()) {
    Serial.println("Sensor MPU6050 iniciado correctamente");
  } else {
    Serial.println("Error al iniciar el sensor MPU6050");
    while (1);  // Detener la ejecución si no hay conexión con el sensor
  }

  Serial.println("Iniciando sensor de sonido...");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Leer el valor del sensor de sonido
  valorSonido = analogRead(pinSensorSonido);

  // Leer las aceleraciones y velocidades angulares del MPU6050
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  // Formatear los datos en un mensaje JSON
  char mensaje[200];
  snprintf(mensaje, sizeof(mensaje),
           "{\"sonido\": %d, \"acelerometro\": {\"x\": %d, \"y\": %d, \"z\": %d}, \"giroscopio\": {\"x\": %d, \"y\": %d, \"z\": %d}}",
           valorSonido, ax, ay, az, gx, gy, gz);

  // Enviar el mensaje a través de MQTT al tópico "sensor/datos"
  sendMessage("sensor/datos", mensaje);

  // Mostrar las lecturas en el monitor serial
  Serial.println(mensaje);

  // Retardo entre envíos
  delay(2000);
}
