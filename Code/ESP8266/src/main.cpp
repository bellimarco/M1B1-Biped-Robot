#include <Arduino.h>

#define USE_TEENSY

#define Log
#ifdef Log
    #ifdef USE_TEENSY
        #define LogPrint(x)         \
        {                           \
            Serial.print("#");      \
            Serial.println(x);        \
        }
    #else
        #define LogPrint(x)         \
        {                           \
            Serial.print("#: ");      \
            Serial.println(x);        \
        }
    #endif
#else
    #define LogPrint(x)
#endif

#define LED_BUILTIN 2


#include <Gcode.h>

#include <ESP8266WiFi.h>
#include <indexhtml.h>
#include <webcomm.h>




void setup(){
    pinMode(LED_BUILTIN,OUTPUT);
    digitalWrite(LED_BUILTIN,HIGH);

    Serial.begin(115200);
    delay(500);
    LogPrint("\nHello World");


    WebCommSetup();
    for(byte i=0; i<8; i++){ digitalWrite(LED_BUILTIN,LOW); delay(13); digitalWrite(LED_BUILTIN,HIGH); delay(25); }


    #ifdef USE_TEENSY
    bool ready=false;
    while(!ready){
        if(Serial.available()){ if(Serial.read() == '\n'){ ready=true; }}
        else{Serial.print("READY");}
        digitalWrite(LED_BUILTIN,LOW); delay(4);
        digitalWrite(LED_BUILTIN,HIGH); delay(22);
    }
    while(Serial.available()){ Serial.read(); }  //flush serial buffer
    for(byte i=0; i<8; i++){ digitalWrite(LED_BUILTIN,LOW); delay(13); digitalWrite(LED_BUILTIN,HIGH); delay(25); }
    #endif


    LogPrint("Setup Done, IP address: "+WiFi.localIP().toString()); //usually its 192.168.1.59
    digitalWrite(LED_BUILTIN,LOW);
}




String serialIn = "";
//check for serial data
void CheckSerial(){
    while(Serial.available()){
        char c=Serial.read();
        if(c!='\n'){ serialIn+=c; }
        else{
            if(serialIn.indexOf("c/")>=0){
                CheckGcode(serialIn.substring(2));
            }
            else if(serialIn.indexOf("e/")>=0){
                ExecuteGcode(serialIn.substring(2));
            }
            else if(serialIn.indexOf("data/")>=0){
                //do whatever with serialIn.substring(5)
            }
            else if(serialIn.indexOf("full")>=0){
                TeensyBufferFull = true;
            }
            else if(serialIn.indexOf("empty")>=0){
                TeensyBufferFull = false;
            }

            serialIn="";
        }
    }
}


void loop() {

    CheckSerial();

    CheckClients();

    delay(1);
}