#include <ArduinoJson.h>

/*

      Programa para monitoreo de cabecera de pacientes
       Ayudar_V1.8

   Realizado:
   1)Cliente bluetooth con hc-06.
   2)Lectura del max30100, promediado de variables.
   3)Lectura del sensor de temperatura,  promediado de variables.
   4)Lectura de sensor de estiramiento con deteccion de flancos y calculo de pulsasiones por minuto. Se incluye pequeña función
     que detecta que el nivel de lectura se encuentre en un rango correcto para ajustar la banda detectora de pulsos correctamente.
   5)Impresión de valores obtenidos mediante puerto serial bluetooth
    
*/
#include <SoftwareSerial.h>

//--------------definicion de pines del HC06-----------------------//
SoftwareSerial BTserial(3, 4); // RX | TX
// Connect the HC-06 TX to the Arduino RX on pin 3. 
// Connect the HC-06 RX to the Arduino TX on pin 4 through a voltage divider.

//----------definicion pines wifi----------------------//


SoftwareSerial wifiSerial(5, 6);      // RX, TX for ESP8266



 unsigned long ceroMillis7= 0 ;   // tiempo de muestreo para promediar
 const long intervalo7 = 1000;

//-----------------Variables Json --------------------//
uint16_t SpO2_Json;
uint16_t bpm_Json;
uint32_t temp_Json;
uint32_t tiempo_Json;


//--------------------------Variables temperatura------------------/
 unsigned long ceroMillis1= 0 ;  // toma muestra de temperatura para calcular promedio
 const long intervalo1 = 5;  // intervalo de muestreo para sensor de temperatura
 
 int  tempEntrada = A6; //sensor A6 lectura temperatura
 int indexTemp = 0;
 float sumaTemp ;
int sumaTemp2 ;

  uint32_t promedioTemp2 ; 

 //--------------------Variables estiramiento-------------------//

 unsigned long ceroMillis3= 0 ;   // analisis de valores minimo y maximo
 const long intervalo3 = 100;

 unsigned long ceroMillis4= 0 ;  // calculo de pulsos por minuto
 const long intervalo4 = 15000;

  int indiceEst = 0;
  const int bufferEst = 5; // cantidad de muestras
  int valorEst[bufferEst];  
  float estiraEntrada = A7;
  bool estiraOK;  // nivel de estiramiento aceptable
  bool estiraAlto;  // nivel de estiramiento supera el limite
  bool estiraBajo; // nivel de estiramiento no alcanza el minimo
  int estiraMax;  // medicion de estiramiento en el punto mas alto
  int estiraMin;  // medicion de estiramiento en el pulso mas bajo
  bool pulsoInhala; // 
  bool pulsoExhala;
  int pulsosResp;  // conteo de pulsos cada vez que se activa "pulsoInhala"
  int pulsosMinut; // calculo de pulsos por minuto

//---------------------Variables para impresion Serial--------------------//

 unsigned long ceroMillis2= 0 ;   // acuse de impresion x puerto serial
 const long intervalo2 = 5000;


//------------------ inclusion del Max30100----------------------------//

 unsigned long ceroMillis6= 0 ;   // tiempo de muestreo para promediar
 const long intervalo6 = 2000;
 
int indexCardio;
int sumaCardio;
int sumaSpO2;

#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

#define REPORTING_PERIOD_MS     1000   // periodo de consulta de valores
uint32_t tsLastReport = 0;

#define MAX30100_SPC_PW_1600US_16BITS;
#define MAX30100_SAMPRATE_100HZ;
int heartRate;
int SpO2;
int beats;
int actualCardio;
int actualSpO2;

PulseOximeter pox;  // maximo nivel de interfaz. deteccion de beat, cardio y SpO2

/*
void onBeatDetected()  //función cada vez que se detecta un pulso
{
   // Serial.println("Beat!");  // Default del programa de la libreria
}
*/
//---------------------------Send command----------------------//



//--------------------------- Protocolo Json------------------------------//




//----------------------Inicio del Setup----------------------------------------//

void setup() 
{
     analogReference(EXTERNAL);  // definir referencia externa para lectura ok de termocupla
    pinMode(13,OUTPUT);  //led interno


     //-------------Setup serial-------------------//
   // Serial.begin(9600);  
  //  Serial.println("Inicializando pulse oximeter..");

    
//---------------- inicialización del sistema si dfunciona bien el oxímetro----------------//

  if (!pox.begin()) {  // si el oxímetro no responde entonces no inicia la com. serial
    //    Serial.println("FAILED");
       
        for(;;);
    } else {
    //    Serial.println("SUCCESS");

    }

   

  //  pox.setOnBeatDetectedCallback(onBeatDetected);
  

 // -------------Setup bluetooth---------//
    BTserial.begin(9600);  
    BTserial.println("Inicializando pulse oximeter..");


    //----------------wifi----------------------//
    

    wifiSerial.begin(115200);
 
}   // fin Void Setup


 
void loop()
{   
  //---------------------------Lectura del Max30100 y promedio de valores de bpm--------------------------------------------//
  
    pox.update();
    int muestrasCardio=10;  // cantidad de muestras a tomar por el oxímetro  20 muestras / 500 ms = 10 segundos de muestreo, 
                            //va a tardar 2 segundos en comenzar a leer el Max30100 por definicion de librería
   
  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {       
        SpO2 = pox.getSpO2();   // la variable de SpO2 la leo directamente cada "REPORTING_PERIOD_MS" (500ms)
        actualCardio = pox.getHeartRate();
        sumaCardio += actualCardio;        
        //    sumaSpO2 += actualSpO2;  // queda en reserva        
        indexCardio ++; //aumento indice     
        tsLastReport = millis();  // actualizo tiempo de muestra
    }
    
     if (indexCardio >= muestrasCardio) { //verifica si ya tomó X muestras de temperatura
        heartRate = (sumaCardio / indexCardio);  // promedio
       // SpO2 = (sumaSpO2 / indexCardio);         //promedio   , queda en reserva           
         sumaCardio =0;  //reseteo valores
       // sumaSpO2 =0; // queda en reserva      
         indexCardio =0;
     }
        
  //------------------------comunicacion hc-06------------------------------------------//
 /*
    // Leer del HC06 y mandar al puerto serial del Arduino
    if (BTserial.available())
    {  
        Serial.write(BTserial.read());
    }
 */
  /*
    // Leer Arduino serial y enviar comandos AT al HC-06
    if (Serial.available())
    {
        Serial.println("Enter AT commands:");
        BTserial.write(Serial.read());
    }
 */
    //------------------------Control de temperatura------------------------------//

       //-------define lectura temperatura-----//
    
    int tempLectura =  analogRead(tempEntrada);

   tempLectura = map(tempLectura , 0, 1024,  0,  50) -2.5;  // hay un corrimiento de -2.5 en la lectura. debe ser la termocupla. 
   
    unsigned long actualMillis1 = millis();
    int muestrasTemp=500;

    
  if (actualMillis1 - ceroMillis1 >= intervalo1) {
    ceroMillis1 = actualMillis1 ;
    
  if (indexTemp < muestrasTemp){
    sumaTemp += tempLectura;
    indexTemp ++;
  }
     
  }

float promedioTemp;

int voltajeTemp; //debug
   
      if (indexTemp >= muestrasTemp) { //verifica si ya tomó X muestras de temperatura
        promedioTemp = (sumaTemp / muestrasTemp) ;// / 10.24  ;
        
 promedioTemp2 = round(promedioTemp * 100);


  //      voltajeTemp = analogRead(tempEntrada) ; //variable temporal

        
     // promedioTemp = map(promedioTemp , 0.0, 512.0, 0.0, 50.0);      
      //promedioTemp = round((((temp0 + temp1 + temp2 + temp3 + temp4) / 5) * 50 / 1000 )*10.0)/10.0 ; //redondea de (ej) 35,45 a 35,40
       // BTserial.print(promedioTemp,1);  // el " , 1 " especifica la cantidad de decimales despues de la coma que se imprimen
      //  BTserial.println("°C");
        //Serial.print(promedioTemp,1);
        //Serial.println("°C");

      indexTemp = 0;
      sumaTemp =0;

    }

    //------------------Control estiramiento--------------------------------------------------------------------------------------------//

   int estiraLectura ;//   = analogRead(estiraEntrada) ;
    

   int estiro;
   estiro = estiraLectura;
 
    if ( estiro < 100){ //determino que el nivel de lectura es muy bajo
      estiraOK=false;
    estiraBajo=true;
    estiraAlto=false;
    }
    else if (estiro > 800){ //determino que el nivel de lectura es muy alto
    estiraOK=false;
    estiraBajo=false;
    estiraAlto=true;
    }
    else if (estiro > 100 and estiro < 800){  //determino rango de lectura adecuado
    estiraOK=true;
    estiraBajo=false;
    estiraAlto=false;
    }

//------Analizo onda de estiramiento------------------------------------------------------------------------------------------------//


  if (estiraOK){
    
  unsigned long actualMillis3 = millis();

 if (actualMillis3 - ceroMillis3 >= intervalo3) {  //-- muestreo de estiramiento
    ceroMillis3 = actualMillis3 ;
    

valorEst[indiceEst] = estiro;  // guardo valor actual

  if (estiro > valorEst[indiceEst - 2]){   // verifico si el valor actual es mayor que la anteúltima muestra
    estiraMax = estiro ;                   // cargo valor
  }
    if (estiro < valorEst[indiceEst - 2]){ // verifico si el valor actual es menor que la anteúltima muestra
      estiraMin = estiro ;                 // cargo valor
    }

   if (valorEst[indiceEst] + 20 < estiraMax and !pulsoInhala){  //--- determino cuando termina de inhalar y activo el pulso
    pulsoInhala = true;
    pulsoExhala = false; 
    pulsosResp ++;  //incremento la cantidad de pulsos de respiración para posterior analisis
//    Serial.println ("Inhala");  // imprimo "inhala" para debug
   }
      if (valorEst[indiceEst] -20 > estiraMin and !pulsoExhala){  //----- determino cuando terminó de exhalar y activo el pulso
    pulsoInhala = false;
    pulsoExhala = true; 
//    Serial.println ("Exhala"); // imprimo "exhala" para debug
   }
   
  
  indiceEst = (indiceEst +1) % bufferEst;  //incremento indice 
  
  }//-- fin muestreo de estiramiento
  } //-- mientras el estirado este correcto

    if(!estiraOK){  // si el estirado no es correcto reseteo valores de minimo y maximo
    estiraMax=0;
    estiraMin=0;
  }


  //---------- medicion de pulso respiratorio por minuto------------------------------------------------------------------------------------//
  
    unsigned long actualMillis4 = millis();

 if (actualMillis4 - ceroMillis4 >= intervalo4) {  //-- muestreo de estiramiento  cada 15 segundos
    ceroMillis4 = actualMillis4 ;
    
  pulsosMinut = pulsosResp * 4;  // estimo valor para 1 minuto 
    pulsosResp=0;    // reseteo el contador
    
    }

  
   //-------------imprimo valores---------------------------------------------------------------------------------------------------------------//

     unsigned long actualMillis2 = millis();
     
      if (actualMillis2 - ceroMillis2 >= intervalo2) { //
    ceroMillis2 = actualMillis2 ;
    
   
/*
 
BTserial.print(" A-Rd: ");
BTserial.print(voltajeTemp);
 
BTserial.print(" Temp: ");
BTserial.print(promedioTemp,1);  // el " , 1 " especifica la cantidad de decimales despues de la coma que se imprimen
BTserial.print("°C");
BTserial.print(" - R/m: ");
BTserial.print(pulsosMinut);
BTserial.print(" - HR: ");
BTserial.print(heartRate);
BTserial.print(" bpm");
BTserial.print(" - SpO2: ");
BTserial.print(SpO2);
BTserial.println("%");
*/
// Serial.print(" AnRe: ");
// Serial.print(voltajeTemp);
// Serial.print(" Temp: ");
// Serial.print(promedioTemp,1);
// Serial.println("°C");
// Serial.print(" - Pulso resp/min: ");
// Serial.print(pulsosMinut);
// Serial.print(" - Cardio: ");
// Serial.print(heartRate);
// Serial.print(" bpm");
// Serial.print(" - SpO2: ");
// Serial.print(SpO2);
// Serial.println("%");

    /*
 if (estiraOK){
    Serial.println("valor OK: ");
    //-------->Prender led de sensor OK
    }
  else if (estiraBajo){
    Serial.println("valor Bajo");
     //----->prender led de sensor flojo
    }
  else if (estiraAlto){
    Serial.println("valor Alto");
    //----->prender led de sensor muy apretado
    }
*/

    
   } //fin de seccion de impresion serial


//--------------------------------------------------Variables Json----------------------------------//
bool hrArJson; 
if (heartRate > 95){
  hrArJson = true;
}
else if (heartRate <= 95){
 hrArJson = false;
}
 SpO2_Json= SpO2 * 10;
 bpm_Json= heartRate *10;
temp_Json= promedioTemp2;
 
 tiempo_Json = millis();



//-------------------------Serializing program-------------------------------------//

const size_t capacity = 4*JSON_ARRAY_SIZE(1) + JSON_OBJECT_SIZE(2) + 3*JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(4);
DynamicJsonDocument doc(capacity);

JsonArray spo2 = doc.createNestedArray("spo2");

JsonObject spo2_0 = spo2.createNestedObject();
spo2_0["time"] = tiempo_Json;
spo2_0["SpO2"] = SpO2_Json;
spo2_0["R"] = 0;


JsonArray bloodP = doc.createNestedArray("bloodP");

JsonObject bloodP_0 = bloodP.createNestedObject();
bloodP_0["time"] = tiempo_Json;
bloodP_0["sys"] = 00;
bloodP_0["dia"] = 00;

JsonArray heartR = doc.createNestedArray("heartR");

JsonObject heartR_0 = heartR.createNestedObject();
heartR_0["time"] = tiempo_Json;
heartR_0["heartR"] = bpm_Json;
heartR_0["HR_AR"] = hrArJson;

JsonArray bodyT = doc.createNestedArray("bodyT");

JsonObject bodyT_0 = bodyT.createNestedObject();
bodyT_0["time"] = tiempo_Json;
bodyT_0["temp"] = temp_Json;

char bufferJson[capacity+90];



    unsigned long actualMillis7 = millis();

 if (actualMillis7 - ceroMillis7 >= intervalo7) {  //-- muestreo de estiramiento  cada 15 segundos
    ceroMillis7 = actualMillis7 ;
    
    
    
   // serializeJson(doc, wifiSerial);

    
   serializeJson(doc, bufferJson);
//   Serial.println(bufferJson);
   wifiSerial.println(bufferJson);
 
    }

  
  }//fin del loop




 
