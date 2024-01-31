#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const int serverPort = 80;

ESP8266WebServer server(serverPort);

void setup() {
  Serial.begin(115200);
  delay(10);

  WiFi.begin("iPhone_", "gagarin11"); // Replace with your Wi-Fi credentials
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/your_endpoint", HTTP_POST, handleData); // Define an endpoint for data reception
  server.on("/move", HTTP_POST, handleMove);       // Define an endpoint for the 'move' command
  server.on("/rotate", HTTP_POST, handleRotate);   // Define an endpoint for the 'rotate' command
  server.on("/lidar_stop_dist", HTTP_POST, handleRotate);   // Define an endpoint for the 'rotate' command

  server.begin();
}

void loop() {
  server.handleClient();
}

void handleData() {
  String dataReceived = server.arg("data");
  Serial.println("Received data: " + dataReceived);
  Serial1.println(dataReceived); // Send data to Arduino Mega via Serial1
  server.send(200, "text/plain", "Data received successfully"); // Send a response to the client
}

void handleMove() {
  String moveData = server.arg("data");
  Serial.println("Move command received: " + moveData);
  Serial1.println(moveData); // Send data to Arduino Mega via Serial1
  server.send(200, "text/plain", "Move command received successfully"); // Send a response to the client
}

void handleLidarStopDist() {
  String moveData = server.arg("data");
  Serial.println("Lidar stop dist command received: " + moveData);
  Serial1.println(moveData); // Send data to Arduino Mega via Serial1
  server.send(200, "text/plain", "Lidar stop dist command received successfully"); // Send a response to the client
}

void handleRotate() {
  String rotateData = server.arg("data");
  Serial.println("Rotate command received: " + rotateData);
  Serial1.println(rotateData); // Send data to Arduino Mega via Serial1
  server.send(200, "text/plain", "Rotate command received successfully"); // Send a response to the client
}
