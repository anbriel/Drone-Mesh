#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <ardupilotmega/mavlink.h>
#include "base64.hpp"
#include <painlessMesh.h>
#include "PixhawkArduinoMAVLink.h"
bool is_connected;
unsigned long is_connected_tlast_ms;
unsigned long serial_data_received_tfirst_ms;
int baudrate = 921600;
#define BUTTON_PIN 2
#define Drone Serial2

WebServer server(80);
painlessMesh mesh;

String userId, password, number, number2;
uint32_t nodeId, num2;
uint16_t meshPort;
bool startAP = false;

#define BUFFER_SIZE 1024
#define MESH_PACKET_SIZE 128
//pixhawk serial def
HardwareSerial &hs = Serial2;
PixhawkArduinoMAVLink mav(hs);


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

        uint8_t encodedMessage[MESH_PACKET_SIZE * 4 / 3 + 4];
        encode_base64(message, packetSize, encodedMessage);
        uint32_t value = strtoul(number2.c_str(), NULL, 10);
        mesh.sendSingle(value, (char*)encodedMessage);
        offset += packetSize;
    }
}

void receivedCallback(uint32_t from, String &msg) {
    static uint8_t reassemblyBuffer[BUFFER_SIZE];
    static int receivedLength = 0;

    uint8_t decoded[MESH_PACKET_SIZE * 4 / 3 + 4];
    int decodedLength = decode_base64((uint8_t*)msg.c_str(), msg.length(), decoded);

    if (receivedLength + decodedLength <= BUFFER_SIZE) {
        memcpy(reassemblyBuffer + receivedLength, decoded, decodedLength);
        receivedLength += decodedLength;

        mavlink_message_t message;
        mavlink_status_t status;

        for (int i = 0; i < receivedLength; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, reassemblyBuffer[i], &message, &status)) {
                uint8_t send_buf[MAVLINK_MAX_PACKET_LEN];
                int send_len = mavlink_msg_to_send_buffer(send_buf, &message);
                Drone.write(send_buf, send_len);
                receivedLength = 0;
                break;
            }
        }
    } else {
        Serial.println("Buffer overflow detected. Message too large.");
        receivedLength = 0;
    }
}

void serialFlushRx(void) {
    while (Drone.available() > 0) { Drone.read(); }
}

void setup() {
    delay(500);
    Serial.begin(115200);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

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
        Serial.println("User ID: " + userId);
        Serial.println("Password: " + password);
        Serial.println("Number: " + number);
        Serial.println("Number2: " + number2);
        Serial.println("Node ID: " + String(nodeId));
    }
}

void loop() {
    if (startAP) {
        server.handleClient();
    } else {
        mesh.update();

        unsigned long tnow_ms = millis();
        uint8_t buf[BUFFER_SIZE];

        if (is_connected && (tnow_ms - is_connected_tlast_ms > 2000)) {
            is_connected = false;
        }

        tnow_ms = millis();
        int avail = Drone.available();
        if (avail <= 0) {
            serial_data_received_tfirst_ms = tnow_ms;
        } else if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > BUFFER_SIZE) {
            serial_data_received_tfirst_ms = tnow_ms;
            int len = Drone.read(buf, sizeof(buf));

            mavlink_message_t msg;
            mavlink_status_t status;

            for (int i = 0; i < len; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
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
    String apSSID = "Node_AP_" + String(nodeId); // Concatenate nodeId with SSID
    WiFi.softAP(apSSID.c_str());
    Serial.println("Access Point Started");
    Serial.println("IP address: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", handleRoot);
    server.on("/submit", handleFormSubmit);
    server.begin();
    Serial.println("HTTP server started");
}

void handleRoot() {
    // Convert number2 to String for pre-filling the form
    String number2Str = String(number2);

    String html = "<!DOCTYPE html>"
                  "<html>"
                  "<head>"
                  "<title>Node Configuration</title>"
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
                  "<h1>Node Configuration Inator</h1>"
                  "<p>Node ID: " + String(nodeId) + "</p>"
                  "<form action='/submit' method='POST'>"
                  "<label for='userid'>Mesh ID:</label>"
                  "<input type='text' id='userid' name='userid' value='" + userId + "' required>"
                  "<label for='password'>Password:</label>"
                  "<input type='text' id='password' name='password' value='" + password + "' required>"
                  "<label for='number'>Port:</label>"
                  "<input type='text' id='number' name='number' value='" + number + "' required>"
                  "<label for='number2'>Root ID:</label>"
                  "<input type='text' id='number2' name='number2' value='" + number2 + "' required>"
                  "<input type='submit' value='Submit'>"
                  "</form>"
                  "</div>"
                  "</div>"
                  "</body>"
                  "</html>";
    server.send(200, "text/html", html);
}


void handleFormSubmit() {
  if (server.hasArg("userid") && server.hasArg("password") && server.hasArg("number") && server.hasArg("number2")) {
    userId = server.arg("userid");
    password = server.arg("password");
    number = server.arg("number");
    number2 = server.arg("number2");
    storeCredentials();
    server.send(200, "text/html", "Credentials saved. You can now close this page and restart the device.");
    ESP.restart(); // Restart the ESP to apply changes
  } else {
    server.send(400, "text/html", "Missing fields");
  }
}

void storeCredentials() {
  EEPROM.writeString(0, userId);
  EEPROM.writeString(100, password); // Adjust the address as needed
  EEPROM.writeString(200, number); // Adjust the address as needed
  EEPROM.writeString(210, number2); // Store number2 starting at address 210 (adjust as needed)
  EEPROM.commit();
}

void loadCredentials() {
  userId = EEPROM.readString(0);
  password = EEPROM.readString(100); // Adjust the address as needed
  number = EEPROM.readString(200); // Adjust the address as needed
  number2 = EEPROM.readString(210); // Load number2 from address 210 (adjust as needed)
}
void storeNodeId(uint32_t id) {
  EEPROM.put(300, id); // Store nodeId starting at address 300
  EEPROM.commit();
}

void loadNodeId() {
  EEPROM.get(300, nodeId); // Load nodeId from address 300
}

void startMesh() {
    meshPort = (uint16_t) number.toInt();

    mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);
    mesh.init(userId.c_str(), password.c_str(), meshPort);
    size_t rxbufsize = Drone.setRxBufferSize(2 * 1024);
    size_t txbufsize = Drone.setTxBufferSize(BUFFER_SIZE);
    Drone.begin(baudrate, SERIAL_8N1, 16, 17);
    mav.Stream();
    delay(2000);
    mesh.onReceive(&receivedCallback);
    is_connected = false;
    is_connected_tlast_ms = 0;
    serial_data_received_tfirst_ms = 0;
    serialFlushRx();
}
