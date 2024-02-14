#include <AsyncTCP.h>

#include <AsyncEventSource.h>
#include <AsyncJson.h>
#include <WebHandlerImpl.h>
#include <ESPAsyncWebServer.h>
#include <WebAuthentication.h>
#include <AsyncWebSynchronization.h>
#include <AsyncWebSocket.h>
#include <WebResponseImpl.h>
#include <StringArray.h>

#include <WiFi.h>
#include <AsyncTCP.h>
#include <SPIFFS.h> // You might need to replace this with LittleFS depending on your preference
#include <Wire.h>

AsyncWebServer server(80);
String from_mega = "";
int is;
char buf[40];

const char* ssid = "iPhone Jaska";
const char* password = "jasu12345";

void setup() {
  Serial.begin(115200);
  Serial.println();

  if (!SPIFFS.begin(true)) {
    Serial.println("An error occurred while mounting SPIFFS");
    return;
  }

  Serial.println("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/jquery-1.11.3.min.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/jquery-1.11.3.min.js", "text/javascript");
  });

  server.on("/roundslider.min.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/roundslider.min.js", "text/javascript");
  });

  server.on("/roundslider.min.css", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/roundslider.min.css", "text/css");
  });

  server.on("/from_MEGA", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/plain", string_to_JS().c_str());
  });

  server.begin();
}

void loop() {
  if (Serial.available() > 0) {
    serial_read();
    from_mega = buf;
  }
}

void serial_read() {
  boolean done = 0;
  while (done == 0) {
    while (Serial.available() > 0) {
      char char_in = Serial.read();
      if (char_in > 13) {
        buf[is] = char_in;
        is++;
      }
      if (char_in == 10) {
        buf[is] = 0;
        is = 0;
        done = 1;
      }
    }
    if (done == 1) { 
      done = 0; 
      break; 
    }
  }
}

String processor(const String& var) {
  // Your code for processing template variables if any
  return String();
}

String string_to_JS() {
  return from_mega;
}
                                          // end of serial read
