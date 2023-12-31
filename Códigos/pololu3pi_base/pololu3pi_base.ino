#include <WiFi.h>
#include <tinycbor.h>
 
const char* ssid = "Robotat";
const char* password =  "iemtbmcit116";
 
WiFiServer wifiServer(8888);
 

 
void 
setup() 
{
 
  Serial.begin(115200);
  Serial2.begin(115200);
 
  delay(1000);
 
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
 
  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());
 
  wifiServer.begin();
}
 
void 
loop() 
{
 
  WiFiClient client = wifiServer.available();
 
  if (client) 
  {
    while (client.connected()) 
    {
 
      while (client.available() > 0) 
      {
        char c = client.read();
        //client.write(c);
        Serial2.write(c);
      }
 
      delay(10);
    }
 
    client.stop();
    Serial.println("Client disconnected");
 
  }
}