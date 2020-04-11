/*

      Programa para transmision de datos de MoSimPa en formato Json a un broker mqtt
       Ayudar_WiFi_PubSubClient_V3.6

   Para placa ESP8266
   Compatible con Raspberry Pi 3 y 4

   Realizado:
   1) Creacion de Cliente WiFi y conexión automática si se provee SSID y PASSWORD de la Red WiFi
   2) Conexión automática con el broker mqtt y reconexión en caso de que se pierda. Hay que proveer dirección y puerto del broker
   3) Deerialización de los datos recibidos por el puerto serie y Serialización y publicación del mismo.
   Si el Arduino pierde conexión serial con el ESP8266, se mantendran los valores del Json doc dentro del esp8266 y se enviará siempre el mismo archivo.
   Es decir, si se pierde la conexión SERIAL, no hay manera de saberlo por parte del broker.
   El ESP8266 tiene un Json doc que solo cambia de valor si lo que recibe por el puerto serial es notNull. Siempre va a enviar este mismo Json cada intervalo determinado.


  Autor: Marcos Vallasciani
  Contacto: marcos.vallasciani@gmail.com
*/









#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#define MQTT_MAX_PACKET_SIZE 1024;
//-----------------------------Datos server --------------------//

// Configuración de la WiFi generada
const char* APssid = "MoSimPa1";
const char* APpass= "1234";

IPAddress AP_IP(192,168,1,100);
IPAddress AP_gateway(192,168,1,1);
IPAddress AP_subnet(255,255,255,0);

//------------------------ Datos cliente y servidor-----------//

const char* ssid = "Casa Valla Arriba";        // Completar datos
const char* password = "Valla-valla1";   //Completar datos
const char* mqtt_server = "192.168.1.114";  //Completar datos

bool mqttConectado;

//--------------- Definicion de cliente-------------------//
WiFiClient espClient;            //Cliente WiFi
PubSubClient client(espClient);


//clienteMqtt.setClient(espClient);//Cliente mqtt
long lastMsg = 0;
char msg[50];
char msg2[50];
int value = 0;


int reconexiones = 0;
int limite = 1;
//--------------- Definicion de Servidor-------------------//
WiFiServer wifiServer(23);
bool clienteConectado;
WiFiClient clienteMovil;


//-------------------------------Setup  ---------------------------//

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP_STA);

  client.setServer(mqtt_server, 1883);
  //  client.setCallback(callback);
//  WiFi.softAPConfig (AP_IP, AP_gateway, AP_subnet);
  WiFi.softAP(APssid, APpass);



    setup_wifi();
  Serial.print("Iniciado AP ");
  Serial.println(ssid);
  Serial.print("IP address:\t");
  Serial.println(WiFi.softAPIP());
      delay(1000);

}
//---------------------------------------JSON------------------------------------------//
char buffer2[1024];

char bufferJson2[1024];

uint32_t spo2_0_time; //
uint16_t spo2_0_SpO2; //
uint16_t spo2_0_R; //

uint32_t bloodP_0_time ; //
uint8_t bloodP_0_sys ; //
uint8_t bloodP_0_dia; //

uint32_t heartR_0_time ; // 1
uint16_t heartR_0_heartR ; // 753
bool heartR_0_HR_AR; // "false"

uint32_t bodyT_0_time ; // 1
uint16_t bodyT_0_temp; // 10496


//------------------------ Conexion WiFi---------------------------------------------------------------//
void setup_wifi() {

  delay(10);
  // Verificación de conexión por puerto Serial. Usado solo para testear conexión
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {  // Espero conexión WiFi
    delay(500);
    Serial.print(".");
  }

  //  Serial.println("");
  Serial.println("WiFi conectado");
  //  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  wifiServer.begin();


}


//----------------------------  CallBack de MQTT ------------------------------------------------------------------//
// Callback de mqtt, puede servir para establecer horario con Unix Epoch enviado desde el servidor
/*
  void callback(char* topic, byte* payload, unsigned int length) {
  for (int i = 0; i < length; i++) {
    //mensaje recibido
  }
   if ((char)payload[0] == '1') {
             //Opcional
  } else {
             //Opcional
  }
  }
*/

//-------------------------  Reconexión del Broker--------------------------------------------------------------------//
void reconnect() {

  // Repetir hasta que se establezca conexión
  while (!client.connected() and reconexiones < limite ) {
    // Intentar conexión
    Serial.println("reconectando Mqtt");
    if (client.connect("ESP8266Client")) {
      // Si se conecta, publicar mensaje de conexión en el broker (opcional)
      //client.publish("reads/{5ccf7f10fdd0}", "Conectado");   //Corregir Topico
      mqttConectado = true;
      reconexiones = 0;
    } else {
      // Esperar 5 segundos antes de reintentar
      delay(5000);
      reconexiones++;
      
    }
  }
}

//----------------------------  Loop principal-------------------------//

void loop() {


  if (!client.connected() and !clienteConectado) {  //Reestablecer conexión en caso de no existir
  mqttConectado=false;
  reconnect();
  reconexiones=0;
  }
  client.loop();

  //----------------------Server WiFi-----------------------//




  if (clienteMovil.connected()) {
    clienteConectado = true;
    /*
    if (clienteMovil.available() > 0) {
      // read the bytes incoming from the client:
      char thisChar = clienteMovil.read();
      // echo the bytes back to the client:
      wifiServer.write(thisChar);
      // echo the bytes to the server as well:
      Serial.write(thisChar);
    }
    */
  }
    if (!clienteMovil.connected() and !mqttConectado) {
    clienteConectado = false;
    clienteMovil = wifiServer.available();
  }
  


  //------------------------------Deserializacion del mensaje Json recibido------------------------//

  //Establezco tamaño del documento Json
  const size_t capacity = 4 * JSON_ARRAY_SIZE(1) + JSON_OBJECT_SIZE(2) + 3 * JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(4) + 100;
  DynamicJsonDocument doc2(capacity);

  // Ejemplo del mensaje Json a recibir con todos sus valores iguales a "111" o "false"
 const char* json = "{\"spo2\":[{\"time\":111,\"SpO2\":111,\"R\":111}],\"bloodP\":[{\"time\":111,\"sys\":111,\"dia\":111}],\"heartR\":[{\"time\":111,\"heartR\":111,\"HR_AR\":false}],\"bodyT\":[{\"time\":111,\"temp\":111}]}";


  //Deserializo Json recibido a través del puerto serial
  deserializeJson(doc2, Serial);  //Deserializar mensaje Json recibido en puerto Serial (proveniente de Arduino Nano)

  JsonObject spo2_0 = doc2["spo2"][0];
  spo2_0_time = spo2_0["time"]; // Unidad: millis()
  spo2_0_SpO2 = spo2_0["SpO2"]; // Unidad: 10x SpO2 value.
  spo2_0_R = spo2_0["R"];        // Unidad: 1000x actual R value

  JsonObject bloodP_0 = doc2["bloodP"][0];
  bloodP_0_time = bloodP_0["time"]; // Unidad: millis()
  bloodP_0_sys = bloodP_0["sys"];  // Unidad: sin especificar.
  bloodP_0_dia = bloodP_0["dia"];  // Unidad: sin especificar.


  JsonObject heartR_0 = doc2["heartR"][0];
  heartR_0_time = heartR_0["time"];     // Unidad: millis()
  heartR_0_heartR = heartR_0["heartR"]; // Unidad: 10x heart rate.
  heartR_0_HR_AR = heartR_0["HR_AR"];   // 1 persona no descansando, HR > 95bpm

  bodyT_0_time = doc2["bodyT"][0]["time"]; // Unidad: millis()
  bodyT_0_temp = doc2["bodyT"][0]["temp"];  // Unidad: x°C /0.00390625


  if (!doc2.isNull()) {               // verifico haber recibido un documento Json en el puerto serial
    serializeJson(doc2, bufferJson2); // Serializo el Json recibido en un Buffer
    memcpy(buffer2, bufferJson2, sizeof(bufferJson2));
  }


  //-------------------------Envío de datos por MQTT  --------------------//

  long now = millis();
  if (now - lastMsg > 2000) {  //Determino frecuencia de publicación de datos en= 1 publicación cada 2 segundos
    lastMsg = now;

    // Funciones para testeo
    //   Serial.print("doc2: ");
    //   serializeJson(doc2, bufferJson2);
    //   Serial.println(bufferJson2);

    client.publish("reads/5ccf7f10fdd0", bufferJson2);  //Publico el documento Json contenido dentro del Buffer

if (clienteConectado){
    clienteMovil.println(String(buffer2));
   clienteMovil.flush();
   delay(100);
   
}   
    //clienteMovil.write("Hello,4");
  //  wifiServer.println("Hello Server");
  }

}
