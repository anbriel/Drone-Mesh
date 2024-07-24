// router_base
#include <WiFi.h> //for ESP32
#include <WebServer.h> //for ESP32
#include <EEPROM.h>
#include <painlessMesh.h> // Include the painlessMesh library

#include <painlessMesh.h>
#include "PixhawkArduinoMAVLink.h" //has mavlink.h
#include "base64.hpp"

#define BUTTON_PIN 2

WebServer server(80); //for ESP32
int baudrate = 921600;
painlessMesh mesh;
#define BUFFER_SIZE 1024  // You can change this value as needed
#define MESH_PACKET_SIZE 128

bool startAP = false;
String userId, password, number;
uint32_t nodeId;
uint16_t meshPort; // Variable to store the converted port number

void loadCredentials();
void setup();
void startAccessPoint();
void startMesh();
void storeNodeId(uint32_t id);
void loadNodeId();
void handleRoot(); 
void handleFormSubmit();
void storeCredentials();


void sendMessageToMesh(uint8_t* buf, int len) {
    int offset = 0;

    while (offset < len) {
        int packetSize = min(MESH_PACKET_SIZE, len - offset);

        uint8_t message[MESH_PACKET_SIZE];
        memcpy(message, buf + offset, packetSize);

        // Calculate the size of the Base64 encoded message buffer  
        uint8_t encodedMessage[MESH_PACKET_SIZE * 4 / 3 + 4];
        encode_base64(message, packetSize, encodedMessage);

        mesh.sendBroadcast((char*)encodedMessage);

        offset += packetSize;
    }
}

void receivedCallback(uint32_t from, String &msg) {
    static uint8_t reassemblyBuffer[BUFFER_SIZE];
    static int receivedLength = 0;

    // Calculate the size of the Base64 decoded message buffer
    uint8_t decoded[MESH_PACKET_SIZE * 4 / 3 + 4];
    int decodedLength = decode_base64((uint8_t*)msg.c_str(), msg.length(), decoded);
    if (receivedLength + decodedLength <= BUFFER_SIZE) {
        memcpy(reassemblyBuffer + receivedLength, decoded, decodedLength);
        receivedLength += decodedLength;

        // Here, we assume the whole MAVLink message is received
        mavlink_message_t message;
        mavlink_status_t status;

        for (int i = 0; i < receivedLength; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, reassemblyBuffer[i], &message, &status)) {
                // Successfully parsed a MAVLink message
                uint8_t send_buf[MAVLINK_MAX_PACKET_LEN];
                int send_len = mavlink_msg_to_send_buffer(send_buf, &message);
                Serial.write(send_buf, send_len);
                receivedLength = 0; // Reset buffer for next message
                break;
            }
        }
    } else {
        // Handle buffer overflow error
        Serial.println("Buffer overflow detected. Message too large.");
        receivedLength = 0; // Reset buffer
    }
}

void serialFlushRx(void) {
    while (Serial.available() > 0) { Serial.read(); }
}


void setup() {
  delay(500);
  size_t rxbufsize = Serial.setRxBufferSize(2 * 1024);
  size_t txbufsize = Serial.setTxBufferSize(BUFFER_SIZE);
  Serial.begin(baudrate);
  pinMode(BUTTON_PIN, INPUT_PULLUP);


  // Initialize EEPROM
  EEPROM.begin(512);

  if (digitalRead(BUTTON_PIN) == LOW) {
    startAP = true;
    loadCredentials();
    startAccessPoint();
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
  } else {
    loadCredentials();
    startMesh();
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    storeNodeId(mesh.getNodeId());
    Serial.println("Stored Credentials:");
    Serial.println("MESH_ID: " + userId);
    Serial.println("MESH_PWD: " + password);
    Serial.println("PORT: " + number);
    Serial.println("Node ID: " + String(nodeId));
  }
}

void loop() {
    if (startAP) {
    server.handleClient();
  } else {
      mesh.update();
    uint8_t buf[BUFFER_SIZE];
    int packetSize = Serial.available();
    if (packetSize > 0) {
        int len = Serial.readBytes(buf, packetSize); // Read the bytes into the buffer    

        mavlink_message_t msg;
        mavlink_status_t status;

        // Parse the received MAVLink messages
        for (int i = 0; i < len; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                // Successfully parsed a MAVLink message
                uint8_t send_buf[MAVLINK_MAX_PACKET_LEN];
                int send_len = mavlink_msg_to_send_buffer(send_buf, &msg);
                sendMessageToMesh(send_buf, send_len); 
            }
        }
    }
  }

}

void startAccessPoint() {
  loadNodeId();
  String apSSID = "Bridge_AP_" + String(nodeId); // Concatenate nodeId with SSID
  WiFi.softAP(apSSID.c_str());
  Serial.println("Access Point Started");
  Serial.println("IP address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/submit", handleFormSubmit);
  server.begin();
  Serial.println("HTTP server started");

  // Wait for the user to finish configuration
  while (true) {
    server.handleClient();
  }
}


void handleRoot() { 
  String html = "<!DOCTYPE html>"
                "<html>"
                "<head>"
                "<title>Router Configuration Inator</title>"
                "<style>"
                "body {"
                "  font-family: Arial, sans-serif;"
                "  margin: 0;"
                "  padding: 0;"
                "  background-color: #f4f4f4;"
                "}"
                ".container {"
                "  width: 50%;"
                "  margin: auto;"
                "  overflow: hidden;"
                "}"
                "#main {"
                "  background: #fff;"
                "  color: #333;"
                "  padding: 20px;"
                "  margin-top: 30px;"
                "  box-shadow: 0 0 10px rgba(0, 0, 0, 0.1);"
                "}"
                "input[type='text'], input[type='submit'] {"
                "  width: 100%;"
                "  padding: 10px;"
                "  margin: 10px 0;"
                "  box-sizing: border-box;"
                "}"
                "input[type='submit'] {"
                "  background-color: #5cb85c;"
                "  color: white;"
                "  border: none;"
                "  cursor: pointer;"
                "}"
                "input[type='submit']:hover {"
                "  background-color: #4cae4c;"
                "}"
                "</style>"
                "</head>"
                "<body>"
                "<div class='container'>"
                "<div id='main'>"
                "<h1>Router Configuration Inator</h1>"
                "<p>Root ID: " + String(nodeId) + "</p>"
                "<form action='/submit' method='POST'>"
                "<label for='userid'>Mesh ID:</label>"
                "<input type='text' id='userid' name='userid' value='" + userId + "' required>"
                "<label for='password'>Password:</label>"
                "<input type='text' id='password' name='password' value='" + password + "' required>"
                "<label for='number'>Port:</label>"
                "<input type='text' id='number' name='number' value='" + number + "' required>"
                "<input type='submit' value='Submit'>"
                "</form>"
                "</div>"
                "</div>"
                "</body>"
                "</html>";
  server.send(200, "text/html", html);
}

void handleFormSubmit() {
  if (server.hasArg("userid") && server.hasArg("password") && server.hasArg("number")) {
    userId = server.arg("userid");
    password = server.arg("password");
    number = server.arg("number");
    storeCredentials();
    server.send(200, "text/html", "Credentials saved. You can now close this page.");
    ESP.restart(); // Restart the ESP to apply changes
  } else {
    server.send(400, "text/html", "Missing fields");
  }
}

void storeCredentials() {
  EEPROM.writeString(0, userId);
  EEPROM.writeString(100, password); // Adjust the address as needed
  EEPROM.writeString(200, number); // Adjust the address as needed
  EEPROM.commit();
}

void loadCredentials() {
  userId = EEPROM.readString(0);
  password = EEPROM.readString(100); // Adjust the address as needed
  number = EEPROM.readString(200); // Adjust the address as needed
}

void storeNodeId(uint32_t id) {
  EEPROM.put(300, id); // Store nodeId starting at address 300
  EEPROM.commit();
}

void loadNodeId() {
  EEPROM.get(300, nodeId); // Load nodeId from address 300
}

void startMesh() {
  // Convert the number to uint16_t
  meshPort = (uint16_t) number.toInt();

  // Initialize the mesh network
  mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);  // set before init() so that you can see startup messages
  mesh.init(userId.c_str(), password.c_str(), meshPort);
  mesh.onReceive(&receivedCallback); // writes data from node

  serialFlushRx();
}


