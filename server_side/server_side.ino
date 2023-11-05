#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const int serverPort = 80;

ESP8266WebServer server(serverPort);

void setup() {
  Serial.begin(115200);
  delay(10);

  WiFi.begin("iPhone_", "gagarin11"); // Replace with your Wi-Fi credentials
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_POST, handleData); // Define an endpoint for data reception
  server.begin();
}

void loop() {
  server.handleClient();
}

void handleData() {
  String dataReceived = server.arg("data");
  Serial.println("Received data: " + dataReceived);
  // Process the received data as needed

  server.send(200, "text/plain", "Data received successfully"); // Send a response to the client
}
