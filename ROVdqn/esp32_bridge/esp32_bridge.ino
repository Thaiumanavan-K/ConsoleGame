/*
 * ╔══════════════════════════════════════════════════════════════════════╗
 * ║  esp32_bridge.ino — ESP32 UDP Bridge (Arduino C++)                   ║
 * ║  Project : Autonomous AI-Driven Plume Tracker (DQN + Triple-MCU)    ║
 * ║  Role    : Bidirectional relay between Pico (UART) ↔ Laptop (UDP)   ║
 * ║  Serial2 : RX=GPIO16, TX=GPIO17 @ 115200 baud                       ║
 * ║  UDP Port: 1234                                                      ║
 * ╚══════════════════════════════════════════════════════════════════════╝
 */

#include <WiFi.h>
#include <WiFiUdp.h>

// ─────────────────────── CONFIGURATION ───────────────────────

// Wi-Fi Credentials (CHANGE THESE)
const char* WIFI_SSID     = "iQOO Neo7";
const char* WIFI_PASSWORD = "Thaiu009";

// Static IP Configuration (CHANGE THESE for your network)
IPAddress local_IP(10, 35, 225, 200);     // ESP32 static IP
IPAddress gateway(10, 35, 225, 254);        // Router gateway
IPAddress subnet(255, 255, 255, 0);       // Subnet mask
IPAddress dns_primary(8, 8, 8, 8);        // Google DNS

// Laptop IP (CHANGE THIS to your laptop's IP)
IPAddress laptop_IP(10, 35, 225, 68);

// UDP Port
const uint16_t UDP_PORT = 1234;

// Serial2 Pins (ESP32 → Pico UART)
#define SERIAL2_RX 16
#define SERIAL2_TX 17
#define SERIAL2_BAUD 115200

// ─────────────────────── GLOBALS ───────────────────────

WiFiUDP udp;
char udpBuffer[512];         // Buffer for incoming UDP packets
char serialBuffer[512];      // Buffer for incoming Serial2 data
int serialBufferIdx = 0;     // Current position in serial buffer

unsigned long lastHeartbeat = 0;
unsigned long lastWiFiCheck = 0;
unsigned long packetsSentToLaptop = 0;
unsigned long packetsSentToPico = 0;
unsigned long bootTime = 0;

// Onboard LED
#define LED_PIN 2

// ─────────────────────── DEBUG LOGGING ───────────────────────

void logMsg(const char* tag, const char* msg) {
    unsigned long elapsed = millis() - bootTime;
    Serial.printf("[ESP32 %8lums] [%s] %s\n", elapsed, tag, msg);
}

void logMsgF(const char* tag, const char* fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    logMsg(tag, buf);
}

// ─────────────────────── WIFI SETUP ───────────────────────

void connectWiFi() {
    logMsg("WIFI", "═══════════════════════════════════════════");
    logMsgF("WIFI", "Connecting to SSID: %s", WIFI_SSID);

    // Configure static IP BEFORE WiFi.begin()
    if (!WiFi.config(local_IP, gateway, subnet, dns_primary)) {
        logMsg("WIFI", "ERROR: Static IP configuration failed!");
    } else {
        logMsg("WIFI", "Static IP configured successfully");
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int retries = 0;
    const int maxRetries = 30;  // 15 seconds timeout

    while (WiFi.status() != WL_CONNECTED && retries < maxRetries) {
        delay(500);
        Serial.print(".");
        retries++;
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        logMsg("WIFI", "═══════════════════════════════════════════");
        logMsgF("WIFI", "Connected! IP: %s", WiFi.localIP().toString().c_str());
        logMsgF("WIFI", "Gateway:     %s", WiFi.gatewayIP().toString().c_str());
        logMsgF("WIFI", "RSSI:        %d dBm", WiFi.RSSI());
        logMsgF("WIFI", "MAC:         %s", WiFi.macAddress().c_str());
        logMsg("WIFI", "═══════════════════════════════════════════");
        digitalWrite(LED_PIN, HIGH);  // Solid LED = connected
    } else {
        logMsg("WIFI", "ERROR: Connection FAILED after 15 seconds!");
        logMsg("WIFI", "Check SSID, password, and signal strength.");
        digitalWrite(LED_PIN, LOW);
    }
}

void checkWiFiReconnect() {
    if (millis() - lastWiFiCheck < 5000) return;  // Check every 5s
    lastWiFiCheck = millis();

    if (WiFi.status() != WL_CONNECTED) {
        logMsg("WIFI", "WARN: Connection lost! Attempting reconnect...");
        digitalWrite(LED_PIN, LOW);
        WiFi.disconnect();
        delay(100);
        connectWiFi();
    }
}

// ─────────────────────── SETUP ───────────────────────

void setup() {
    bootTime = millis();

    // USB Serial for debug logging
    Serial.begin(115200);
    delay(500);

    // Onboard LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.println();
    logMsg("INIT", "╔══════════════════════════════════════════════╗");
    logMsg("INIT", "║  ESP32 UDP BRIDGE — Plume Tracker v1.0       ║");
    logMsg("INIT", "║  Serial2: GPIO16(RX) / GPIO17(TX) @ 115200  ║");
    logMsg("INIT", "║  UDP Port: 1234                              ║");
    logMsg("INIT", "╚══════════════════════════════════════════════╝");

    // Serial2: UART to Pico
    Serial2.begin(SERIAL2_BAUD, SERIAL_8N1, SERIAL2_RX, SERIAL2_TX);
    logMsg("INIT", "Serial2 initialized (to Pico)");

    // Connect WiFi
    connectWiFi();

    // Start UDP listener
    if (udp.begin(UDP_PORT)) {
        logMsgF("INIT", "UDP listening on port %d", UDP_PORT);
    } else {
        logMsg("INIT", "ERROR: UDP begin() failed!");
    }

    logMsg("INIT", "Bridge ready — entering main loop");
    logMsgF("INIT", "Laptop target: %s:%d", laptop_IP.toString().c_str(), UDP_PORT);
}

// ─────────────────────── MAIN LOOP ───────────────────────

void loop() {
    // ── 1. UDP → Serial2 (Laptop → Pico) ──
    // Check for incoming UDP packets from the laptop
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
        int len = udp.read(udpBuffer, sizeof(udpBuffer) - 1);
        if (len > 0) {
            udpBuffer[len] = '\0';

            // Forward to Pico over Serial2
            Serial2.write((uint8_t*)udpBuffer, len);
            packetsSentToPico++;

            logMsgF("UDP→PICO", "Received %d bytes from %s: '%s'",
                     len,
                     udp.remoteIP().toString().c_str(),
                     udpBuffer);
        }
    }

    // ── 2. Serial2 → UDP (Pico → Laptop) ──
    // Read Serial2 data and forward complete lines to laptop
    while (Serial2.available()) {
        char c = Serial2.read();

        // Buffer until newline (packet delimiter)
        if (c == '\n' || serialBufferIdx >= (int)(sizeof(serialBuffer) - 2)) {
            serialBuffer[serialBufferIdx] = '\n';
            serialBuffer[serialBufferIdx + 1] = '\0';

            // Send buffered line to laptop via UDP
            if (WiFi.status() == WL_CONNECTED) {
                udp.beginPacket(laptop_IP, UDP_PORT);
                udp.write((uint8_t*)serialBuffer, serialBufferIdx + 1);
                udp.endPacket();
                packetsSentToLaptop++;

                logMsgF("PICO→UDP", "Sent %d bytes to laptop: '%.*s'",
                         serialBufferIdx + 1,
                         serialBufferIdx,  // Don't print the newline
                         serialBuffer);
            } else {
                logMsg("PICO→UDP", "WARN: WiFi disconnected, packet dropped!");
            }

            serialBufferIdx = 0;
        } else {
            serialBuffer[serialBufferIdx++] = c;
        }
    }

    // ── 3. Periodic Status / Heartbeat ──
    if (millis() - lastHeartbeat >= 10000) {  // Every 10 seconds
        lastHeartbeat = millis();
        logMsg("STATUS", "───────────────────────────────────────");
        logMsgF("STATUS", "Uptime: %lu s | WiFi: %s | RSSI: %d dBm",
                (millis() - bootTime) / 1000,
                WiFi.status() == WL_CONNECTED ? "OK" : "DISCONNECTED",
                WiFi.RSSI());
        logMsgF("STATUS", "Packets → Laptop: %lu | Packets → Pico: %lu",
                packetsSentToLaptop, packetsSentToPico);
        logMsg("STATUS", "───────────────────────────────────────");

        // Blink LED for heartbeat
        digitalWrite(LED_PIN, LOW);
        delay(50);
        digitalWrite(LED_PIN, HIGH);
    }

    // ── 4. WiFi Reconnect Check ──
    checkWiFiReconnect();

    // Small delay to prevent WDT reset on ESP32
    delay(1);
}
