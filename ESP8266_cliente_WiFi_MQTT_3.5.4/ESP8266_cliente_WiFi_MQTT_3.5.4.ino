#include <ArduinoJson.h>


 
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define MQTT_MAX_PACKET_SIZE 1024;

 
//------------------------ Datos cliente y servidor-----------//
const char* ssid = "xxxxx";
const char* password = "xxxxxxxx";


const char* mqtt_server = "192.168.1.123";

 //--------------- Definicion de cliente-------------------//
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
char msg2[50];
int value = 0;    



//-------------------------------Setup  ---------------------------//
 
void setup() {

  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  


}
  //---------------------------------------JSON------------------------------------------//
  

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
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);
 
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 
  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


//----------------------------  CallBack de MQTT ------------------------------------------------------------------// 

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
 
  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
  //  digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
   Serial.println("1");
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    //digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }
 
}

//-------------------------  Reconexión del Broker--------------------------------------------------------------------// 
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Intentando conexion MQTT ...");
   Serial.print(mqtt_server);
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("reads/{5c:cf:7f:10:fd:d0}", "Enviando el primer mensaje");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("fallo, rc=");
      Serial.print(client.state());
      Serial.println(" intentando de nuevo en 5 segundos");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//----------------------------  Loop principal-------------------------//
 
void loop() {
 
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
//-----------------------guardado de datos-------------------------------//
 


//------------------------------Deserialize------------------------//

  

 const size_t capacity = 4*JSON_ARRAY_SIZE(1) + JSON_OBJECT_SIZE(2) + 3*JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(4) + 100;
DynamicJsonDocument doc2(capacity);

const char* json = "{\"spo2\":[{\"time\":111,\"SpO2\":111,\"R\":111}],\"bloodP\":[{\"time\":111,\"sys\":111,\"dia\":111}],\"heartR\":[{\"time\":111,\"heartR\":111,\"HR_AR\":false}],\"bodyT\":[{\"time\":111,\"temp\":111}]}";

deserializeJson(doc2, Serial);

JsonObject spo2_0 = doc2["spo2"][0];
 spo2_0_time = spo2_0["time"]; // 1
 spo2_0_SpO2 = spo2_0["SpO2"]; // 110
spo2_0_R = spo2_0["R"]; // 25

JsonObject bloodP_0 = doc2["bloodP"][0];
bloodP_0_time = bloodP_0["time"]; // 10
 bloodP_0_sys = bloodP_0["sys"]; // 84
 bloodP_0_dia = bloodP_0["dia"]; // 60


JsonObject heartR_0 = doc2["heartR"][0];
 heartR_0_time = heartR_0["time"]; // 1
 heartR_0_heartR = heartR_0["heartR"]; // 753
 heartR_0_HR_AR = heartR_0["HR_AR"]; // "false"

 bodyT_0_time = doc2["bodyT"][0]["time"]; // 1
bodyT_0_temp = doc2["bodyT"][0]["temp"]; // 10496


   
   JsonObject object = doc2.as<JsonObject>();
 
if (!doc2.isNull()){
  serializeJson(doc2, bufferJson2);
}


//-------------------------Ejemplo Hello world  MQTT  --------------------//

  long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
  //  ++value;
  //  snprintf (msg, 75, "hello world #%ld", value);
    

  //  Serial.print("Publish message: ");
  //  Serial.println(msg);
   // client.publish("read", msg);
  //  delay(10);
    
    
 //   Serial.print("doc2: ");
 //   serializeJson(doc2, bufferJson2);
 //   Serial.println(bufferJson2);


  //  client.publish("read", "bufferJson2->");
    client.publish("reads/5ccf7f10fdd0", bufferJson2);

  }
  
}
