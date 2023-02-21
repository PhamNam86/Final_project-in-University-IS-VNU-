#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>

const char* ssid = "SMART VACCUUM CLEAN";
//const char* password = "NodeMCU";

ESP8266WebServer server(80);

void setup(){
  
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT); //ERROR
  // Connecting WiFi
  WiFi.mode(WIFI_AP);//SETTING AP IP MODE
  WiFi.softAP(ssid);//SETTING AP MODE
  //IPAddress myIP = WiFi.softAPIP();
  // Starting
     server.on ( "/", HTTP_handleRoot );
     server.onNotFound ( HTTP_handleRoot );
     server.begin();    
     
}
void loop(){
  digitalWrite(LED_BUILTIN, LOW);//ERROR
  server.handleClient();
  delay(10);
}
void HTTP_handleRoot(void) 
{
if( server.hasArg("State") ){
      Serial.println(server.arg("State"));
  }
  server.send(200,"text/html","" );
  delay(1);
}
