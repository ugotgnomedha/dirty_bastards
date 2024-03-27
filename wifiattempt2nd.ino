#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

#define ESP32_TX 17
#define ESP32_RX 16

const char* ssid = "iPhone_";
const char* password = "gagarin11";

AsyncWebServer server(80);

void setup() {
    Serial.begin(9600);
    Serial2.begin(9600, SERIAL_8N1, ESP32_RX, ESP32_TX);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial2.print(".");
    }

    Serial2.println("");
    Serial2.println("WiFi connected.");
    Serial2.println("IP address: " + WiFi.localIP().toString());

    // Serve files from SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial2.println("An error occurred while mounting SPIFFS");
        return;
    }

    server.onNotFound([](AsyncWebServerRequest *request){
        Serial2.println("Not found: " + request->url());
        request->send(404, "text/plain", "Not Found");
    });
 

    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html").setCacheControl("max-age=600");
    server.serveStatic("/index.css", SPIFFS, "/index.css").setCacheControl("max-age=600");
    server.serveStatic("/index.js", SPIFFS, "/index.js").setCacheControl("max-age=600");


    
    //POST METHODS ______________________________________________________________
    
    server.on("/sendMoveDistance", HTTP_POST, [](AsyncWebServerRequest *request){
    String moveData;
      if (request->hasArg("move")) {
       moveData = request->arg("move");
      }
    Serial2.println("Move distance received: " + moveData);
    // Process moveData as needed, e.g., send it to Arduino Mega via Serial1
    Serial2.println(moveData);
    request->send(200, "text/plain", "Move command received successfully");
    });


    server.on("/sendLidarDistance", HTTP_POST, [](AsyncWebServerRequest *request){
    String lidarData;
    if (request->hasArg("distance")) {
      lidarData = request->arg("distance");
    }
    Serial2.println("LIDAR distance received: " + lidarData);
    // Process lidarData as needed, e.g., send it to Arduino Mega via Serial1
    Serial2.println(lidarData);
    request->send(200, "text/plain", "LIDAR distance received successfully");
    });


     server.on("/sendCompassValue", HTTP_POST, [](AsyncWebServerRequest *request){
    String compassData;
    if (request->hasArg("degrees")) {
      compassData = request->arg("degrees");
    }
    Serial2.println("Compass value received: " + compassData);
    // Process rotateData as needed, e.g., send it to Arduino Mega via Serial1
    Serial2.println(compassData);
    request->send(200, "text/plain", "Rotate command received successfully");
    });


    // GET METHODS _____________________________________________________________
    
      server.on("/getSensorData", HTTP_GET, [](AsyncWebServerRequest *request){
    // Read Lidar and Compass data
    Serial2.println("GET_SENSOR_DATA");
          String response = "";
    while (Serial2.available()) {
      char c = Serial2.read();
      response += c;
      
    }

    Serial.println("Response from Arduino Mega: " + response);
    
    request->send(200, "text/plain", response);
  });


  // START AND STOP 

  server.on("/startDriving", HTTP_POST, [](AsyncWebServerRequest *request){
    Serial2.println("START");
    request->send(200, "text/plain", "Started driving successfully");
  });

  server.on("/stopDriving", HTTP_POST, [](AsyncWebServerRequest *request){
    Serial2.println("STOP");
    request->send(200, "text/plain", "Stopped driving successfully");
  });

    
    // START THE SERVER
    server.begin();
}



  

void loop() 
{
  
    // Check WiFi connection status every 5 seconds
    static unsigned long lastWifiCheckTime = 0;
    const unsigned long wifiCheckInterval = 5000;  // 5 seconds

    unsigned long currentMillis = millis();
    if (currentMillis - lastWifiCheckTime >= wifiCheckInterval) 
      {
        lastWifiCheckTime = currentMillis;

        if (WiFi.status() == WL_CONNECTED) 
        {
            Serial2.println("WiFi is still connected. IP: " + WiFi.localIP().toString());
        } else {
            Serial2.println("WiFi is disconnected. Reconnecting...");
            WiFi.reconnect();
        }
      }
  
}








      
