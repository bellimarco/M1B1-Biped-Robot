
String wifiName = "Sunrise_2.4GHz_362F40";
String wifiPassword = "jzXt73W1zq6F";
WiFiServer server(80);  // Set web server port number to 80
WiFiClient client;

// variables used during client requests
String header="";
String response="";
// timeout timer during client request
const uint32_t timeoutTime =  2000;
uint32_t timeoutBegin = 0;


void WebCommSetup(){


    LogPrint("Connecting");
    WiFi.begin(wifiName,wifiPassword);
    while (WiFi.status() != WL_CONNECTED){
        digitalWrite(LED_BUILTIN,LOW);
        delay(10);
        digitalWrite(LED_BUILTIN,HIGH);
        delay(40);
    }
    server.begin();

    //insert the ip address in the html file, so that the client will know the ip of the esp
    String s = "var MCU_IP_ADDRESS = ";
    int n = indexHTML.indexOf(s)+s.length();                           //index before the first doublequote
    String t = indexHTML.substring(0,n+1) + WiFi.localIP().toString(); //from 0 to the first doublequote, plus the ip
    indexHTML = t + indexHTML.substring(n+1);    // plus from the second doublequote to the end

}



//check clients and handle their ge requests
void CheckClients(){
  client = server.available();     // Listen for incoming clients

  if (client) {                  // if new client connects
    timeoutBegin = millis();
    while (client.connected() && millis()-timeoutBegin<timeoutTime){
      if (client.available()) {    // if there's bytes to read from the client
        char c = client.read();
        if (c != '\n'){ header += c; }
        else{                      // if the byte is a newline character
            digitalWrite(LED_BUILTIN,HIGH);

            int n = header.indexOf("/Gcode/");
            if(n>=0){
                String gcode = header.substring(n+7,header.indexOf(" ",n));
                if(!TeensyBufferFull){
                    LogPrint("client/processed Gcode: "+gcode);
                    response = "ESP/Processed Gcode: "+gcode;
                    Serial.println(gcode);
                }else{
                    response = "ESP/TeensyBufferFull, didn't process gcode";
                    LogPrint("client/TeensyBufferFull, didn't process gcode");
                }
            }
            else if(header.indexOf("/DisplayUpdate")>=0){
                response = "ESP/Display response";
            }
            else if(header.indexOf("/GcodeUpdate")>=0){
                //send gcode buffers, Gexecute [tail...head], Gexecuted [tail.........]
                byte i=GcodeExecuteTail;
                while(i!=GcodeExecuteHead){
                    i=(i+GcodeBuffersSize-1)%GcodeBuffersSize;
                    response+=GcodeExecute[i]+",";
                }
                response+=";";
                i=GcodeExecutedTail;
                while(i!=(GcodeExecutedTail+1)%GcodeBuffersSize){
                    i=(i+GcodeBuffersSize-1)%GcodeBuffersSize;
                    response+=GcodeExecuted[i]+",";
                }
            }
            else if(header.indexOf("GET / HTTP/1.1")>=0){
                response = indexHTML;
                LogPrint("client/ requested page");
            }
            else{ 
                response = "ESP/ address not found: "+header;
                LogPrint("client/ address not found: "+header);
            }


            //compile headers and body of response
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Content-length:"+String(response.length()));
            client.println("Connection: close");
            client.println();
            client.println(response);
            client.println();
            response = "";

            digitalWrite(LED_BUILTIN,LOW);
            break;
        }
      }
    }
    //client disconnected or timeout or responded successfully
    header = "";
    client.stop();
  }
}