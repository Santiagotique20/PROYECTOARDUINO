/*preuba*/
#include <WiFi.h>
#include <PubSubClient.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// Configuración WiFi
const char* ssid = "Santiago20";     // Nombre de la red WiFi
const char* password = "Santiago2020";   // Contraseña de la red WiFi

// Configuración del broker MQTT
const char* mqtt_server = "192.168.176.115";  // Dirección IP del broker EMQX
const int mqtt_port = 1883;                 // Puerto del broker MQTT
const char* mqtt_topic = "test/sensors";    // Tópico para publicar datos del sensor

WiFiClient espClient;
PubSubClient client(espClient);

// Pines para el sensor de sonido
const int pinSensorSonido = A4;  
int valorSonido = 0;           

// Definir sensor MPU6050
MPU6050 sensor;

// Valores RAW del MPU6050
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Función para conectarse a WiFi
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

  Serial.println();
  Serial.println("Conexión WiFi establecida");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());
}

// Función callback para mensajes MQTT (no se usa en este ejemplo)
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensaje recibido en tópico: ");
  Serial.println(topic);
  Serial.print("Mensaje: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// Reconectar al broker MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect("ESP32Client")) {
      Serial.println("Conectado al broker MQTT");
      client.subscribe(mqtt_topic);
    } else {
      Serial.print("fallido, rc=");
      Serial.print(client.state());
      Serial.println(" Intentando en 5 segundos...");
      delay(5000);
    }
  }
}

void setup() {
  // Inicializar puerto serial
  Serial.begin(115200);

  // Conectar a WiFi
  setup_wifi();

  // Configurar el cliente MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Inicializar I2C y el sensor MPU6050
  Wire.begin();
  sensor.initialize();

  if (sensor.testConnection()) {
    Serial.println("Sensor MPU6050 iniciado correctamente");
  } else {
    Serial.println("Error al iniciar el sensor MPU6050");
    while (1);  // Detener si falla la conexión con el sensor
  }

  Serial.println("Iniciando sensor de sonido...");
}

void loop() {
  // Conexión y mantenimiento del cliente MQTT
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Leer valor del sensor de sonido
  valorSonido = analogRead(pinSensorSonido);
  Serial.print("Nivel de sonido: ");
  Serial.println(valorSonido);

  // Leer datos del acelerómetro y giroscopio del MPU6050
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  // Mostrar datos del MPU6050
  Serial.print("Acelerómetro [X Y Z]:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.println();

  Serial.print("Giroscopio [X Y Z]:\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);

  // Crear un mensaje JSON con los datos de los sensores
  String msg = "{\"sonido\": " + String(valorSonido) + 
               ", \"acelerometro\": {\"x\": " + String(ax) + ", \"y\": " + String(ay) + ", \"z\": " + String(az) + "}," + 
               "\"giroscopio\": {\"x\": " + String(gx) + ", \"y\": " + String(gy) + ", \"z\": " + String(gz) + "}}";
  
  // Publicar el mensaje en el tópico MQTT
  client.publish(mqtt_topic, msg.c_str());
  Serial.println("Mensaje publicado: ");
  Serial.println(msg);

  // Retardo entre lecturas
  delay(5000);
}
