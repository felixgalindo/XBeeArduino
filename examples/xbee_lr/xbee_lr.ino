/**
 * @file example.ino
 * @brief Example application demonstrating the use of the XBeeArduino library on Arduino.
 * 
 * This file contains a sample application that demonstrates how to use the XBeeArduino library 
 * to communicate with XBee modules on an Arduino platform. It showcases basic operations 
 * such as initializing the XBee module, connecting to the network, and transmitting & receiving data.
 * 
 * @version 1.0
 * @date 2024-08-17
 * author Felix Galindo
 * 
 * @license MIT
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <XBeeArduino.h>
#include <config.h>
#include <port.h>
#include <xbee.h>
#include <xbee_api_frames.h>
#include <xbee_at_cmds.h>
#include <xbee_lr.h>
#include <stdarg.h> // Use C-style header instead of C++ <cstdarg>

// Declare a pointer for the serial port
Stream* serialPort;

// Declare XBeeArduino instance
XBeeArduino* xbee;

// XBeeLR payload to send
uint8_t examplePayload[] = {0xC0, 0xC0, 0xC0, 0xFF, 0xEE};
uint16_t payloadLen;
XBeeLRPacket_t packet;

/**
 * @brief Callback function triggered when data is received from the XBee module.
 * 
 * This function is called when the XBee module receives data. It processes the 
 * incoming data, extracts relevant information, and handles it accordingly.
 * 
 * @param[in] data Pointer to the received packet data.
 */
void OnReceiveCallback(void* data) {
    XBeeLRPacket_t* packet = (XBeeLRPacket_t*)data;
    Serial.print("Received Packet: ");
    for (int i = 0; i < packet->payloadSize; i++) {
        Serial.print("0x");
        if (packet->payload[i] < 0x10) {
            Serial.print("0");
        }
        Serial.print(packet->payload[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    Serial.print("Ack: ");
    Serial.println(packet->ack);
    Serial.print("Port: ");
    Serial.println(packet->port);
    Serial.print("RSSI: ");
    Serial.println(packet->rssi);
    Serial.print("SNR: ");
    Serial.println(packet->snr);
    Serial.print("Downlink Counter: ");
    Serial.println(packet->counter);
}

/**
 * @brief Callback function triggered after data is sent from the XBee module.
 * 
 * This function is called when the XBee module completes sending data. It handles 
 * any post-send operations, such as logging the transmission status or updating 
 * the state of the application.
 * 
 * @param[in] data Pointer to the sent packet data.
 */
void OnSendCallback(void* data) {
    XBeeLRPacket_t* packet = (XBeeLRPacket_t*)data;
    switch (packet->status) {
        case 0:
            Serial.print("Send successful (frameId: 0x");
            Serial.print(packet->frameId, HEX);
            Serial.println(")");
            break;
        case 0x01:
            Serial.print("Send failed (frameId: 0x");
            Serial.print(packet->frameId, HEX);
            Serial.println(") (reason: Ack Failed)");
            break;
        case 0x022:
            Serial.print("Send failed (frameId: 0x");
            Serial.print(packet->frameId, HEX);
            Serial.println(") (reason: Not Connected)");
            break;
        default:
            Serial.print("Send failed (frameId: 0x");
            Serial.print(packet->frameId, HEX);
            Serial.print(") (reason: 0x");
            Serial.print(packet->status, HEX);
            Serial.println(")");
            break;
    }
}

void setup() {
    // Initialize the Arduino Serial port for debugging
    Serial.begin(9600);
    serialPort = &Serial1;

    delay(6000);
    Serial.println("");
    Serial.println("----------------------------------------------------------------------");
    Serial.println("XBee LR Example App");

    // Initialize the XBee module
    xbee = new XBeeArduino(serialPort, 9600, XBEE_LORA, OnReceiveCallback, OnSendCallback);
    if (!xbee->begin()) {
        Serial.println("Failed to initialize XBee");
        while (1); // Halt the program on failure
    }

    // Read LoRaWAN DevEUI and print
    char devEui[17];

    // This might take a few trys ... 
    bool status = false; 
    for(int i = 0; i < 3; i++){
        if (xbee->getLoRaWANDevEUI(devEui,sizeof(devEui))){
            status = true;
            break;
        }
        delay(100);
    }
    // if (xbee->getLoRaWANDevEUI(devEui,sizeof(devEui))){
    if (status){
      Serial.print("DEVEUI: ");
      Serial.print((char*)devEui);
    Serial.println();
    } else {
        Serial.println("Failed to retrieve DevEUI");
    }

    // Set LoRaWAN Network Settings
    Serial.println("Configuring...");
    xbee->setLoRaWANAppEUI("37D56A3F6CDCF0A5");
    xbee->setLoRaWANAppKey("CD32AAB41C54175E9060D86F3A8B7F48");
    xbee->setLoRaWANNwkKey("CD32AAB41C54175E9060D86F3A8B7F48");
    xbee->setLoRaWANClass('C');
    xbee->setApiOptions(0x01);
    xbee->writeConfig();
    xbee->applyChanges();

    // Connect to LoRaWAN network
    Serial.println("Connecting...");
    if (!xbee->connect()) {
        Serial.println("Failed to connect.");
    } else {
        Serial.println("Connected!");
    }

    // Initialize the payload
    payloadLen = sizeof(examplePayload) / sizeof(examplePayload[0]);
    packet.payload = examplePayload;
    packet.payloadSize = payloadLen;
    packet.port = 2;
    packet.ack = 0;
}

void loop() {
    // Check if 10 seconds have passed
    static uint32_t startTime = millis();
    if (millis() - startTime >= 10000) {
        // Send data if connected, else reconnect
        if (xbee->isConnected()) {
            Serial.print("Sending 0x");
            for (int i = 0; i < payloadLen; i++) {
                Serial.print(examplePayload[i], HEX);
            }
            Serial.println();

            if (!xbee->sendData(packet)) {
                Serial.println("Failed to send data.");
            } else {
                Serial.println("Data sent successfully.");
            }

            // Update payload
            packet.payload[0] = packet.payload[0] + 1; // change payload
        } else {
            Serial.println("Not connected. Connecting...");
            if (!xbee->connect()) {
                Serial.println("Failed to connect.");
            } else {
                Serial.println("Connected!");
            }
        }
        // Reset the start time
        startTime = millis();
    }
}