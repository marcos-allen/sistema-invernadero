/* Codigo fuente del sistema de control de agua
Elaborado por el equipo 3 (Marcos Allen, Sebastian, Karol, Edwin)
Reto: Sistema de IoT para gestion, monitoreo y control de un invernadero */

// Importacion de librerias
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "GravityTDS.h"
#include <EEPROM.h>


// Definicion de variables a utilizar para las mediciones, sensor y actuadores
#define TdsSensorPin A0
GravityTDS gravityTds;
float calidad;
const int bombaAguaZona1 = 7;
const int bombaAguaZona2 = 8;
int field4_value = 0;
int field7_value = 0;
float field2_value = 0;
float field5_value = 0;

// Credenciales para la conexion a la red WiFi
const char* ssid = ""; 
const char* password = "";


// Servidor MQTT del broker
const char* server = "mqtt3.thingspeak.com";

// Seleccion para conexion a red, entre segura y no segura
//#define USESECUREMQTT             // Comentar esta linea si se usa una conexion no segura
#ifdef USESECUREMQTT
  #include <WiFiClientSecure.h>
  #define mqttPort 8883
  WiFiClientSecure client; 
#else
  #include <WiFiS3.h>
  #define mqttPort 1883
  WiFiClient client;
#endif

// Credenciales que permiten la conexion (publicar y suscribir) al canal de ThingSpeak
const char mqttUserName[]   = ""; 
const char clientID[]       = "";
const char mqttPass[]       = "";

// ID del Canal de ThingSpeak
#define channelID 0000000

// Certificado de ThingSpeak utilizado para establecer la conexion segura
const char * PROGMEM thingspeak_ca_cert = \
  "-----BEGIN CERTIFICATE-----\n" \
  "MIIDxTCCAq2gAwIBAgIQAqxcJmoLQJuPC3nyrkYldzANBgkqhkiG9w0BAQUFADBs\n" \
  "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
  "d3cuZGlnaWNlcnQuY29tMSswKQYDVQQDEyJEaWdpQ2VydCBIaWdoIEFzc3VyYW5j\n" \
  "ZSBFViBSb290IENBMB4XDTA2MTExMDAwMDAwMFoXDTMxMTExMDAwMDAwMFowbDEL\n" \
  "MAkGA1UEBhMCVVMxFTATBgNVBAoTDERpZ2lDZXJ0IEluYzEZMBcGA1UECxMQd3d3\n" \
  "LmRpZ2ljZXJ0LmNvbTErMCkGA1UEAxMiRGlnaUNlcnQgSGlnaCBBc3N1cmFuY2Ug\n" \
  "RVYgUm9vdCBDQTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAMbM5XPm\n" \
  "+9S75S0tMqbf5YE/yc0lSbZxKsPVlDRnogocsF9ppkCxxLeyj9CYpKlBWTrT3JTW\n" \
  "PNt0OKRKzE0lgvdKpVMSOO7zSW1xkX5jtqumX8OkhPhPYlG++MXs2ziS4wblCJEM\n" \
  "xChBVfvLWokVfnHoNb9Ncgk9vjo4UFt3MRuNs8ckRZqnrG0AFFoEt7oT61EKmEFB\n" \
  "Ik5lYYeBQVCmeVyJ3hlKV9Uu5l0cUyx+mM0aBhakaHPQNAQTXKFx01p8VdteZOE3\n" \
  "hzBWBOURtCmAEvF5OYiiAhF8J2a3iLd48soKqDirCmTCv2ZdlYTBoSUeh10aUAsg\n" \
  "EsxBu24LUTi4S8sCAwEAAaNjMGEwDgYDVR0PAQH/BAQDAgGGMA8GA1UdEwEB/wQF\n" \
  "MAMBAf8wHQYDVR0OBBYEFLE+w2kD+L9HAdSYJhoIAu9jZCvDMB8GA1UdIwQYMBaA\n" \
  "FLE+w2kD+L9HAdSYJhoIAu9jZCvDMA0GCSqGSIb3DQEBBQUAA4IBAQAcGgaX3Nec\n" \
  "nzyIZgYIVyHbIUf4KmeqvxgydkAQV8GK83rZEWWONfqe/EW1ntlMMUu4kehDLI6z\n" \
  "eM7b41N5cdblIZQB2lWHmiRk9opmzN6cN82oNLFpmyPInngiK3BD41VHMWEZ71jF\n" \
  "hS9OMPagMRYjyOfiZRYzy78aG6A9+MpeizGLYAiJLQwGXFK3xPkKmNEVX58Svnw2\n" \
  "Yzi9RKR/5CYrCsSXaQ3pjOLAEFe4yHYSkVXySGnYvCoCWw9E1CAx2/S6cCZdkGCe\n" \
  "vEsXCS+0yx5DaMkHJ8HSXPfqIbloEpw8nL+e/IBcm2PN7EeqJSdnoDfzAIJ9VNep\n" \
  "+OkuE6N36B9K\n" \
  "-----END CERTIFICATE-----\n";


// Estado inicial de la conexion Wifi
int status = WL_IDLE_STATUS; 

// El cliente MQTT esta vinculado a la conexion WiFi
PubSubClient mqttClient( client );

// Variables definidas para controlar el tiempo de conexiones y para definir la
// frecuencia de actualizacion de las lecturas de los sensores (en milisegundos)
int connectionDelay    = 4;    // Retraso (s) entre intentos de conexion a WiFi
long lastPublishMillis = 0;    // Para almacenar el valor de la ultima llamada a la funcion millis()
int updateInterval     = 15;   // Las lecturas de los sensores se publican cada 15 segundos

/*
* Prototipos de funciones, agrupadas por funcionalidad y dependencias
**/

// Funcion para conectar a WiFi.
void connectWifi();

// Funcion para conectar al servidor MQTT, es decir, mqtt3.thingspeak.com
void mqttConnect();

// Funcion para suscribirse al canal de ThingSpeak para obtener actualizaciones.
void mqttSubscribe( long subChannelID );

// Funcion para manejar los mensajes de la suscripcion MQTT al broker de ThingSpeak.
void mqttSubscriptionCallback( char* topic, byte* payload, unsigned int length );

// Funcion para publicar mensajes a un canal de ThingSpeak.
void mqttPublish(long pubChannelID, String message);


void setup() {
  // Establecer la tasa de transmision serial y configurar la comunicacion
  Serial.begin( 115200 );
  EEPROM.begin();
  // Algunos retrasos para permitir la configuracion del serial
  delay(3000);     

  // Configuracion del sensor de calidad de agua
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(3.3);
  gravityTds.setAdcRange(1024);
  gravityTds.begin();

  // Configuracion de las bombas de agua
  pinMode(bombaAguaZona1, OUTPUT);
  pinMode(bombaAguaZona2, OUTPUT);
  digitalWrite(bombaAguaZona1, LOW);
  digitalWrite(bombaAguaZona2, LOW);

  // Conectar a la red Wi-Fi especificada.
  connectWifi();
  
  // Configurar el cliente MQTT para conectar con el broker de ThinkSpeak
  mqttClient.setServer( server, mqttPort ); 
  
  // Establecer la funcion manejadora de los mensajes MQTT
  mqttClient.setCallback( mqttSubscriptionCallback );
  // Establecer el tamaño del buffer para manejar el JSON retornado.
  // NOTA: Un desbordamiento del buffer de mensajes resultara en que la funcion de callback no sea invocada
  mqttClient.setBufferSize( 2048 );
  
  // Usar conexiones MQTT seguras si esta definido
  #ifdef USESECUREMQTT
    // Manejar las diferencias de funcionalidad de la libreria WiFiClientSecure para diferentes placas
      client.setCACert(thingspeak_ca_cert);
  #endif
}


void loop() {
  // After everythins is set up, go the perception-action loop
  // Reconnect to WiFi if it gets disconnected.
  if (WiFi.status() != WL_CONNECTED) {
      connectWifi();
  }
  
  // Connect if MQTT client is not connected and resubscribe to channel updates.
  // ThinkSpeak broaker server : suscribe to the specified channel
  if (!mqttClient.connected()) {
     mqttConnect(); 
     mqttSubscribe( channelID );
  }
  
  // Call the loop to maintain connection to the server.
  mqttClient.loop(); 
  
  // Update ThingSpeak channel periodically according to the specified rate. 
  // The update results in the message to the subscriber.
  if ( (millis() - lastPublishMillis) > updateInterval*1000) {
    
    if (field2_value != -1 && field5_value != -1) {
      float promedio = (field2_value + field5_value) / 2;
      gravityTds.setTemperature(promedio);
    } else {
      gravityTds.setTemperature(20);
    }
    gravityTds.update();
    calidad = gravityTds.getTdsValue();
    Serial.print(F("Quality: "));
    Serial.print(calidad, 2);
    Serial.println("ppm");


    //calidad < 500 okay
    //calidad > 500 no
    if (calidad < 500){
      Serial.println("La calidad del agua es excelente.");
    } else if (calidad >= 500) {
      Serial.println("El agua presenta indicios de contaminación (no recomendable).");
    }

    //mqttPublish( channelID, (String("field1=")+String(WiFi.RSSI())) );
    mqttPublish( channelID, (String("field1=")+String(calidad)));
    lastPublishMillis = millis();

    if (calidad >= 500) {
      // Desactivar ambas bombas si la calidad del agua no es aceptable
      digitalWrite(bombaAguaZona1, LOW);
      digitalWrite(bombaAguaZona2, LOW);
      Serial.println("Ambas bombas desactivadas debido a mala calidad del agua.");
    }
    else {
      // Controlar las bombas basándose en los valores de humedad del suelo
      if (field4_value < 1200 && field7_value < 1200 && field4_value != 0 && field7_value != 0 ) {
        digitalWrite(bombaAguaZona1, HIGH);
        digitalWrite(bombaAguaZona2, HIGH);
        Serial.println("Ambas bombas activadas porque field4 y field7 < 1200.");
      }
      else if (field4_value < 1200 && field4_value != 0  ) {
        digitalWrite(bombaAguaZona1, HIGH);
        digitalWrite(bombaAguaZona2, LOW);
        Serial.println("Bomba de Zona 1 activada porque field4 < 1200.");
      }
      else if (field7_value < 1200 && field7_value != 0 ) {
        digitalWrite(bombaAguaZona1, LOW);        
        digitalWrite(bombaAguaZona2, HIGH);
        Serial.println("Bomba de Zona 2 activada porque field4 < 1200.");
      }
      else {
        digitalWrite(bombaAguaZona1, LOW);
        digitalWrite(bombaAguaZona2, LOW); 
        Serial.println("Ambas bombas desactivadas porque field4 y field7 >= 1200 o Datos erroneos (0)");
      } 
    }
  }  
}

// Function to connect to WiFi.
void connectWifi()
{
  Serial.println( "Connecting to Wi-Fi..." );
  // Loop until WiFi connection is successful
    while ( WiFi.status() != WL_CONNECTED ) {
    //WiFi.begin( ssid, pass );
    WiFi.begin(ssid, password);
    delay( connectionDelay*1000 );
    Serial.println( WiFi.status() ); 
  }
  Serial.println( "Connected to Wi-Fi." );
}

// Function to connect to MQTT server.
void mqttConnect() {
  // Loop until the client is connected to the server.
  while ( !mqttClient.connected() )
  {
    // Connect to the MQTT broker.
    if ( mqttClient.connect( clientID, mqttUserName, mqttPass ) ) {
      Serial.print( "MQTT to " );
      Serial.print( server );
      Serial.print (" at port ");
      Serial.print( mqttPort );
      Serial.println( " successful." );
    } else {
      Serial.print( "MQTT connection failed, rc = " );
      // See https://pubsubclient.knolleary.net/api.html#state for the failure code explanation.
      Serial.print( mqttClient.state() );
      Serial.println( " Will try the connection again in a few seconds" );
      delay( connectionDelay*1000 );
    }
  }
}

// Function to subscribe to ThingSpeak channel for updates.
void mqttSubscribe( long subChannelID ){
  String myTopic = "channels/"+String( subChannelID )+"/subscribe";
  mqttClient.subscribe(myTopic.c_str());
}

// Agregar la función processJsonMessage
void processJsonMessage(const char* jsonMessage) {
  // Crear un objeto DynamicJsonDocument para almacenar el JSON
  DynamicJsonDocument doc(2048);

  // Intentar deserializar el JSON
  DeserializationError error = deserializeJson(doc, jsonMessage);

  // Verificar si hubo errores en la deserialización
  if (error) {
    Serial.print("Error al analizar JSON: ");
    Serial.println(error.c_str());
    return;
  }

  // Verificar y extraer los valores de field2, field4, field5 y field7
  if (doc.containsKey("field2") && !doc["field2"].isNull()) {
    field2_value = doc["field2"].as<float>();
  } 

  if (doc.containsKey("field4") && !doc["field4"].isNull()) {
    field4_value = doc["field4"].as<int>();
  } 

  if (doc.containsKey("field5") && !doc["field5"].isNull()) {
    field5_value = doc["field5"].as<float>();
  }

  if (doc.containsKey("field7") && !doc["field7"].isNull()) {
    field7_value = doc["field7"].as<int>();
  }

  // Imprimir los resultados en el monitor serial
  Serial.print("Valor de field2: ");
  Serial.println(field2_value);
  Serial.print("Valor de field4: ");
  Serial.println(field4_value);
  Serial.print("Valor de field5: ");
  Serial.println(field5_value);
  Serial.print("Valor de field7: ");
  Serial.println(field7_value);
}

void mqttSubscriptionCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.println("Mensaje recibido: " + message);

  // Llamar a la nueva función para procesar el mensaje JSON
  processJsonMessage(message.c_str());
}

// Function to publish messages to a ThingSpeak channel.
void mqttPublish(long pubChannelID, String message) {
  String topicString ="channels/" + String( pubChannelID ) + "/publish";
  mqttClient.publish( topicString.c_str(), message.c_str() );
}
