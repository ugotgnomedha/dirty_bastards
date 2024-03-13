#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

const char* ssid = "iPhone Jaska";
const char* password = "jasu12345";

AsyncWebServer server(80);

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: " + WiFi.localIP().toString());

    // Serve files from SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial.println("An error occurred while mounting SPIFFS");
        return;
    }

    server.onNotFound([](AsyncWebServerRequest *request){
        Serial.println("Not found: " + request->url());
        request->send(404, "text/plain", "Not Found");
    });
 

    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html").setCacheControl("max-age=600");
    server.serveStatic("/index.css", SPIFFS, "/index.css").setCacheControl("max-age=600");
    server.serveStatic("/index.js", SPIFFS, "/index.js").setCacheControl("max-age=600");

    

    server.on("/move", HTTP_POST, [](AsyncWebServerRequest *request){
    String moveData;
      if (request->hasArg("data")) {
       moveData = request->arg("data");
      }
    Serial.println("Move command received: " + moveData);
    // Process moveData as needed, e.g., send it to Arduino Mega via Serial1
    Serial1.println(moveData);
    request->send(200, "text/plain", "Move command received successfully");
    });

    //Lidar stuff
    server.on("/getLidarDistance", HTTP_GET, [](AsyncWebServerRequest *request){
    // Logic to retrieve LIDAR distance
    float lidarDistance = 0;
    request->send(200, "text/plain", String(lidarDistance));
  });

    server.on("/sendLidarDistance", HTTP_POST, [](AsyncWebServerRequest *request){
    String lidarData;
    if (request->hasArg("distance")) {
      lidarData = request->arg("distance");
    }
    Serial.println("LIDAR distance received: " + lidarData);
    // Process lidarData as needed, e.g., send it to Arduino Mega via Serial1
    Serial1.println(lidarData);
    request->send(200, "text/plain", "LIDAR distance received successfully");
    });

  // Compass stuff

     server.on("/rotate", HTTP_POST, [](AsyncWebServerRequest *request){
    String rotateData;
    if (request->hasArg("data")) {
      rotateData = request->arg("data");
    }
    Serial.println("Rotate to compass command received: " + rotateData);
    // Process rotateData as needed, e.g., send it to Arduino Mega via Serial1
    Serial1.println(rotateData);
    request->send(200, "text/plain", "Rotate command received successfully");
    });

  server.on("/getCompassValue", HTTP_GET, [](AsyncWebServerRequest *request){
    // Logic to retrieve compass value
    float compassValue = 0;
    request->send(200, "text/plain", String(compassValue));
    });
  
    server.begin();
}



  

void loop() {
    // Check WiFi connection status every 5 seconds
    static unsigned long lastWifiCheckTime = 0;
    const unsigned long wifiCheckInterval = 5000;  // 5 seconds

    unsigned long currentMillis = millis();
    if (currentMillis - lastWifiCheckTime >= wifiCheckInterval) {
        lastWifiCheckTime = currentMillis;

        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("WiFi is still connected. IP: " + WiFi.localIP().toString());
        } else {
            Serial.println("WiFi is disconnected. Reconnecting...");
            WiFi.reconnect();
        }
    }

}








      
