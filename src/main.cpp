/*
   MIT License

   Copyright (c) 2025 Ing. Peter Javorsky
   Copyright (c) 2021 Alessandro Orlando

   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the
   "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute,
   sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
   TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVEN
   SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
   ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
   OR OTHER DEALINGS IN THE SOFTWARE.
   
*/

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <SPIFFS.h>
#include <opus.h>
#include <driver/i2s.h>
#include "wifi_config.h"

// I2S Configuration
#define I2S_WS_PIN 15
#define I2S_SD_PIN 13
#define I2S_SCK_PIN 2
#define I2S_PORT I2S_NUM_0
#define SAMPLE_RATE 48000
#define SAMPLE_BITS 16
#define OPUS_FRAME_SIZE 480
#define OPUS_MAX_PACKET_SIZE 1500

// WebSocket server
WebSocketsServer webSocket = WebSocketsServer(81);
AsyncWebServer server(80);

// OPUS encoder
OpusEncoder* encoder = NULL;
opus_int16* pcm = NULL;
unsigned char* opusData = NULL;

// Audio buffer
const int bufferLen = 1024;
int16_t sBuffer[bufferLen];

void setupI2S() {
    const i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = (i2s_bits_per_sample_t)SAMPLE_BITS,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    const i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD_PIN
    };

    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_PORT, &pin_config);
}

void setupOpus() {
    int error;
    encoder = opus_encoder_create(SAMPLE_RATE, 1, OPUS_APPLICATION_AUDIO, &error);
    if (error != OPUS_OK) {
        Serial.println("Failed to create OPUS encoder");
        return;
    }

    opus_encoder_ctl(encoder, OPUS_SET_BITRATE(64000));
    opus_encoder_ctl(encoder, OPUS_SET_COMPLEXITY(10));
    opus_encoder_ctl(encoder, OPUS_SET_SIGNAL(OPUS_SIGNAL_MUSIC));

    pcm = (opus_int16*)malloc(OPUS_FRAME_SIZE * sizeof(opus_int16));
    opusData = (unsigned char*)malloc(OPUS_MAX_PACKET_SIZE);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED:
            Serial.printf("[%u] Connected\n", num);
            break;
        case WStype_TEXT:
            // Handle WebRTC signaling
            break;
        case WStype_BIN:
            // Handle binary data
            break;
        case WStype_ERROR:
            Serial.printf("[%u] Error!\n", num);
            break;
        case WStype_FRAGMENT_TEXT_START:
        case WStype_FRAGMENT_BIN_START:
        case WStype_FRAGMENT:
        case WStype_FRAGMENT_FIN:
            break;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Configuring WiFi");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to WiFi network with IP Address: ");
    Serial.println(WiFi.localIP());

    // Initialize SPIFFS
    if(!SPIFFS.begin(true)) {
        Serial.println("SPIFFS Mount Failed");
        return;
    }

    // Setup I2S
    setupI2S();
    
    // Setup OPUS encoder
    setupOpus();

    // Setup web server
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/index.html", "text/html");
    });

    server.begin();
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
}

void loop() {
    webSocket.loop();

    // Read I2S data
    size_t bytesIn = 0;
    i2s_read(I2S_PORT, sBuffer, bufferLen * sizeof(int16_t), &bytesIn, portMAX_DELAY);

    if (bytesIn > 0) {
        // Convert to mono and normalize
        for(int i = 0; i < bufferLen; i++) {
            pcm[i] = sBuffer[i];
        }

        // Encode with OPUS
        int encodedBytes = opus_encode(encoder, pcm, OPUS_FRAME_SIZE, opusData, OPUS_MAX_PACKET_SIZE);
        
        if (encodedBytes > 0) {
            // Send encoded audio data through WebSocket
            webSocket.broadcastBIN(opusData, encodedBytes);
        }
    }
}
