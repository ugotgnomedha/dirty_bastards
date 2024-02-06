#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

const char* serverAddress = "http://172.20.10.4"; // Replace with the IP or domain of your server

void setup() {
  Serial.begin(115200);
  delay(10);
  // Connect to Wi-Fi
  WiFi.begin("iPhone_", "gagarin11"); // Replace with your Wi-Fi credentials
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');

    // Send data to the server using an HTTP POST request
    if (sendDataToServer(data)) {
      Serial.println("Data sent to the server successfully: " + data);
    } else {
      Serial.println("Failed to send data to the server");
    }
  }
}

bool sendDataToServer(String data) {
  HTTPClient http;
  http.begin(serverAddress);

  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  // Prepare the data to be sent
  String postData = "data=" + data;

  int httpCode = http.POST(postData);

  if (httpCode == HTTP_CODE_OK) {
    return true; // Data sent successfully
  } else {
    Serial.println("HTTP POST request failed with error code: " + httpCode);
    return false; // Failed to send data
  }

  http.end();
}
