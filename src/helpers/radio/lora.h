#include <LoRa.h>

enum PacketType {
    TELEMETRY_PACKET,
    LOG_PACKET,
    COMMAND_PACKET
};

static void initRadio() {
    #ifdef DIAGNOSTIC
        Serial.print("Initializing radio...");
    #endif
    LoRa.setPins(RFM_CS_PIN, RFM_RST_PIN, RFM_IRQ_PIN);
    
    if (!LoRa.begin(915E6)) {
        #ifdef DIAGNOSTIC
            Serial.println("Starting LoRa failed!");
        #endif
        while(true) blinkCode(RADIO_ERROR_CODE, RED); // Block further code execution
    }
    LoRa.setSyncWord(0xF3);
    #ifdef DIAGNOSTIC
        Serial.println("done!");
    #endif
}

/*
Packet structure:
---IMPLICIT HEADERS---
1. Sender UUID
2. Destination UUID
3. Payload type (telemetry, log, command)

---TELEMETRY PAYLOAD---
4a. Telemetry data structure
5a. End packet

---DIAGNOSTIC PAYLOAD---
4b. Log level
5b. Log message
6b. End packet

---COMMAND PAYLOAD---
4c. Command identifier
5c. Command message (if applicable)
6c. End packet
*/

static void sendTelemetryData() {
    LoRa.beginPacket();
    LoRa.write(UUID);
    LoRa.write(RECEIVER_UUID);
    LoRa.write(TELEMETRY_PACKET);
    LoRa.write((uint8_t*) &data, data.packetSize);
    LoRa.endPacket();
    LoRa.receive();
}

static void sendDiagnosticData(LogLevel level, char* msg) {
    LoRa.beginPacket();
    LoRa.write(UUID);
    LoRa.write(RECEIVER_UUID);
    LoRa.write(LOG_PACKET);
    LoRa.write((byte) level);
    LoRa.print(msg);
    LoRa.endPacket();
    LoRa.receive();
}

static void radioCallback(int packetSize) {
    if (packetSize == 0) return;

    // 
    byte sender = LoRa.read();
    byte destination = LoRa.read();
    if (destination != UUID) return; // Ignore received messages if not the intended receiver
    byte packetType = LoRa.read();
    if (packetType != COMMAND_PACKET) return; //Ignore any packets that aren't command packets
    byte cmdIdent = LoRa.read();
    byte cmdMsg;
    if (packetSize != 0) cmdMsg = LoRa.read(); // Remaining data is the command message
    executeCommand((Command) cmdIdent, cmdMsg); // Grab command identifier from radio packet and execute appropriate command
    //Additional processing for command messages???
}