/*
 * ╔══════════════════════════════════════════════════════════════════════╗
 * ║  camera_server.ino — ESP32-CAM MJPEG Streamer                       ║
 * ║  Project : Autonomous AI-Driven Plume Tracker (DQN + Triple-MCU)    ║
 * ║  Role    : OV2640 camera → HTTP MJPEG stream for laptop vision      ║
 * ║  Stream  : http://<ESP32-CAM_IP>:80/stream                          ║
 * ╚══════════════════════════════════════════════════════════════════════╝
 */

#include "esp_camera.h"
#include "esp_timer.h"
#include "fb_gfx.h"
#include "img_converters.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"
#include <WiFi.h>

// ─────────────────────── CONFIGURATION ───────────────────────

// Wi-Fi Credentials (CHANGE THESE — same network as ESP32 Bridge)
const char *WIFI_SSID = "YOUR_WIFI_NAME";
const char *WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// HTTP Server Port
#define SERVER_PORT 80

// ─────────────────────── AI-THINKER ESP32-CAM PIN MAP ───────────────────────
// These pins are specific to the AI-Thinker ESP32-CAM module.
// Do NOT change these unless you have a different board.

#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// Onboard flash LED (GPIO 4 on AI-Thinker)
#define FLASH_LED_PIN 4

// ─────────────────────── GLOBALS ───────────────────────

WiFiServer server(SERVER_PORT);
unsigned long bootTime = 0;

// ─────────────────────── DEBUG LOGGING ───────────────────────

void logMsg(const char *tag, const char *msg) {
  unsigned long elapsed = millis() - bootTime;
  Serial.printf("[CAM %8lums] [%s] %s\n", elapsed, tag, msg);
}

void logMsgF(const char *tag, const char *fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  logMsg(tag, buf);
}

// ─────────────────────── CAMERA INIT ───────────────────────

bool initCamera() {
  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000; // 20MHz XCLK
  config.pixel_format = PIXFORMAT_JPEG;

  // Resolution & quality settings
  // If PSRAM is available, use higher resolution
  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;     // 640x480
    config.jpeg_quality = 12;              // 0-63 (lower = better quality)
    config.fb_count = 2;                   // Double buffering
    config.grab_mode = CAMERA_GRAB_LATEST; // Always grab latest frame
    logMsg("CAM", "PSRAM found — VGA mode, double buffer");
  } else {
    config.frame_size = FRAMESIZE_QVGA; // 320x240
    config.jpeg_quality = 15;
    config.fb_count = 1;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    logMsg("CAM", "No PSRAM — QVGA mode, single buffer");
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    logMsgF("CAM", "ERROR: Camera init failed (0x%x)", err);
    return false;
  }

  // Sensor tuning for indoor/smoke environments
  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_brightness(s, 1);                 // Slightly brighter
    s->set_contrast(s, 1);                   // Slightly more contrast
    s->set_saturation(s, 0);                 // Neutral saturation
    s->set_whitebal(s, 1);                   // Auto white balance ON
    s->set_awb_gain(s, 1);                   // AWB gain ON
    s->set_wb_mode(s, 0);                    // Auto WB mode
    s->set_exposure_ctrl(s, 1);              // Auto exposure ON
    s->set_aec2(s, 1);                       // AEC DSP ON
    s->set_gain_ctrl(s, 1);                  // Auto gain ON
    s->set_agc_gain(s, 0);                   // AGC gain 0
    s->set_gainceiling(s, (gainceiling_t)6); // 64x gain ceiling
    s->set_bpc(s, 1);                        // Bad pixel correction ON
    s->set_wpc(s, 1);                        // White pixel correction ON
    s->set_raw_gma(s, 1);                    // Gamma correction ON
    s->set_lenc(s, 1);                       // Lens correction ON
    s->set_dcw(s, 1);                        // Downsize enable
    logMsg("CAM", "Sensor parameters configured");
  }

  logMsg("CAM", "Camera initialized successfully");
  return true;
}

// ─────────────────────── WIFI SETUP ───────────────────────

void connectWiFi() {
  logMsgF("WIFI", "Connecting to: %s", WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 40) {
    delay(500);
    Serial.print(".");
    retries++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    logMsg("WIFI", "═══════════════════════════════════════════");
    logMsgF("WIFI", "Connected! IP: %s", WiFi.localIP().toString().c_str());
    logMsgF("WIFI", "RSSI: %d dBm", WiFi.RSSI());
    logMsgF("WIFI", "Stream URL: http://%s:%d/stream",
            WiFi.localIP().toString().c_str(), SERVER_PORT);
    logMsgF("WIFI", "Status URL: http://%s:%d/",
            WiFi.localIP().toString().c_str(), SERVER_PORT);
    logMsg("WIFI", "═══════════════════════════════════════════");
  } else {
    logMsg("WIFI", "ERROR: WiFi connection failed!");
  }
}

// ─────────────────────── HTTP HANDLERS ───────────────────────

// MJPEG boundary string
#define BOUNDARY "frame_boundary_plume_tracker"

void handleStream(WiFiClient &client) {
  logMsgF("STREAM", "Client connected from %s",
          client.remoteIP().toString().c_str());

  // Send HTTP response headers for MJPEG stream
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=" BOUNDARY);
  client.println("Access-Control-Allow-Origin: *");
  client.println("Cache-Control: no-cache, no-store, must-revalidate");
  client.println("Connection: keep-alive");
  client.println();

  unsigned long frameCount = 0;
  unsigned long streamStart = millis();

  while (client.connected()) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      logMsg("STREAM", "WARN: Frame capture failed, retrying...");
      delay(10);
      continue;
    }

    // Send MJPEG frame
    client.printf("--%s\r\n", BOUNDARY);
    client.printf("Content-Type: image/jpeg\r\n");
    client.printf("Content-Length: %u\r\n\r\n", fb->len);

    // Write frame data in chunks for reliability
    size_t written = 0;
    const size_t chunkSize = 1024;
    while (written < fb->len) {
      size_t toWrite = min(chunkSize, fb->len - written);
      size_t sent = client.write(fb->buf + written, toWrite);
      if (sent == 0) {
        logMsg("STREAM", "WARN: Write failed, client may have disconnected");
        break;
      }
      written += sent;
    }
    client.print("\r\n");

    esp_camera_fb_return(fb);
    frameCount++;

    // Log FPS every 100 frames
    if (frameCount % 100 == 0) {
      float elapsed = (millis() - streamStart) / 1000.0;
      float fps = frameCount / elapsed;
      logMsgF("STREAM", "Frames: %lu | FPS: %.1f | Uptime: %.0fs", frameCount,
              fps, elapsed);
    }

    // Small delay to limit frame rate (~20 FPS max)
    delay(50);
  }

  float totalElapsed = (millis() - streamStart) / 1000.0;
  logMsgF("STREAM", "Client disconnected. Total frames: %lu in %.1fs",
          frameCount, totalElapsed);
}

void handleRoot(WiFiClient &client) {
  // Simple status page
  String html = "<!DOCTYPE html><html><head>";
  html += "<title>Plume Tracker Camera</title>";
  html += "<style>";
  html +=
      "body { background: #1a1a2e; color: #e0e0e0; font-family: monospace; ";
  html += "display: flex; flex-direction: column; align-items: center; "
          "padding: 20px; }";
  html += "h1 { color: #00ff88; }";
  html +=
      "img { border: 2px solid #00ff88; border-radius: 8px; max-width: 100%; }";
  html += ".info { background: #16213e; padding: 15px; border-radius: 8px; ";
  html += "margin: 10px; border: 1px solid #0f3460; }";
  html += "a { color: #00ff88; }";
  html += "</style></head><body>";
  html += "<h1>🔥 Plume Tracker — ESP32-CAM</h1>";
  html += "<div class='info'>";
  html += "<p>📡 Stream: <a href='/stream'>/stream</a></p>";
  html += "<p>📷 Resolution: " +
          String(psramFound() ? "VGA (640x480)" : "QVGA (320x240)") + "</p>";
  html += "<p>📶 RSSI: " + String(WiFi.RSSI()) + " dBm</p>";
  html += "<p>⏱️ Uptime: " + String((millis() - bootTime) / 1000) + "s</p>";
  html += "</div>";
  html += "<h2>Live Feed</h2>";
  html += "<img src='/stream' alt='Camera Stream' />";
  html += "</body></html>";

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.print("Content-Length: ");
  client.println(html.length());
  client.println();
  client.print(html);
}

void handle404(WiFiClient &client) {
  client.println("HTTP/1.1 404 Not Found");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();
  client.println("404 - Not Found. Try / or /stream");
}

// ─────────────────────── SETUP ───────────────────────

void setup() {
  bootTime = millis();

  // Disable brownout detector (ESP32-CAM draws a lot of power)
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(500);

  // Flash LED setup
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  Serial.println();
  logMsg("INIT", "╔══════════════════════════════════════════════╗");
  logMsg("INIT", "║  ESP32-CAM MJPEG SERVER — Plume Tracker v1.0 ║");
  logMsg("INIT", "║  OV2640 Camera → HTTP MJPEG Stream           ║");
  logMsg("INIT", "╚══════════════════════════════════════════════╝");

  // Initialize camera
  if (!initCamera()) {
    logMsg("INIT", "FATAL: Camera init failed! Halting.");
    while (true) {
      digitalWrite(FLASH_LED_PIN, !digitalRead(FLASH_LED_PIN));
      delay(200); // Fast blink = error
    }
  }

  // Connect to WiFi
  connectWiFi();

  // Start HTTP server
  server.begin();
  logMsgF("INIT", "HTTP server started on port %d", SERVER_PORT);

  // Quick flash to indicate ready
  for (int i = 0; i < 3; i++) {
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(100);
    digitalWrite(FLASH_LED_PIN, LOW);
    delay(100);
  }

  logMsg("INIT", "Camera server ready — waiting for clients");
}

// ─────────────────────── MAIN LOOP ───────────────────────

void loop() {
  WiFiClient client = server.available();

  if (client) {
    String request = "";
    unsigned long timeout = millis() + 3000;

    while (client.connected() && millis() < timeout) {
      if (client.available()) {
        char c = client.read();
        request += c;

        // End of HTTP headers
        if (request.endsWith("\r\n\r\n")) {
          break;
        }
      }
    }

    logMsgF("HTTP", "Request from %s: %s", client.remoteIP().toString().c_str(),
            request.substring(0, min((int)request.length(), 64)).c_str());

    // Route requests
    if (request.indexOf("GET /stream") >= 0) {
      handleStream(client);
    } else if (request.indexOf("GET / ") >= 0 ||
               request.indexOf("GET /index") >= 0) {
      handleRoot(client);
    } else {
      handle404(client);
    }

    client.stop();
  }

  // WiFi reconnect check
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 10000) {
    lastCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      logMsg("WIFI", "WARN: Disconnected! Reconnecting...");
      connectWiFi();
    }
  }
}
