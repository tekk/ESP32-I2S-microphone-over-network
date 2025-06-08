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
#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>
#include <LittleFS.h>
#include <WiFiUdp.h>
#include <opus.h>
#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ArduinoJson.h>
#include "wifi_config.h"

// I2S Configuration
#ifdef ESP32S3
#define I2S_WS_PIN GPIO_NUM_45
#define I2S_SD_PIN GPIO_NUM_6
#define I2S_SCK_PIN GPIO_NUM_5
#define I2S_PORT I2S_NUM_0
#elif defined(ESP32)
#define I2S_WS_PIN GPIO_NUM_25
#define I2S_SD_PIN GPIO_NUM_32
#define I2S_SCK_PIN GPIO_NUM_26
#define I2S_PORT I2S_NUM_0
#endif

#define SAMPLE_RATE 48000
#define SAMPLE_BITS I2S_BITS_PER_SAMPLE_16BIT
#define OPUS_FRAME_SIZE 960  // 20ms at 48kHz
#define NUM_BUFFERS 4        // Quad buffering
#define BUFFER_SIZE_SAMPLES (OPUS_FRAME_SIZE * 2)  // 1920 samples (~40ms)
#define UDP_AUDIO_PORT 5555
#define UDP_STATS_PORT 5556
#define TASK_STACK_SIZE 16384
#define AUDIO_SEND_INTERVAL 20  // Send every 20ms (50 packets/sec)
#define STATS_SEND_INTERVAL 500 // Send stats every 500ms
#define AUDIO_MAGIC 0xA0D10001  // Fixed magic number

// UDP Audio packet structure
struct UdpAudioPacket {
    uint32_t magic;         // 0xA0D10001
    uint32_t sequence;
    uint32_t timestamp;
    uint16_t dataSize;
    uint8_t codec;          // 0=PCM, 1=OPUS
    uint8_t channels;
    uint32_t sampleRate;
    uint8_t data[];
} __attribute__((packed));

// Statistics structure
struct AudioStats {
    uint32_t packetsPerSec;
    uint32_t bytesPerSec;
    uint32_t totalPackets;
    uint32_t totalBytes;
    float compressionRatio;
    uint32_t uptime;
    uint16_t rmsLevel;
    uint8_t connectedClients;
    uint32_t udpPacketsDropped;
} stats;

// WebSocket statistics message
struct WsStatsMessage {
    char type[8];           // "stats"
    AudioStats data;
} __attribute__((packed));

// Global variables
AsyncWebServer server(80);
AsyncWebSocket wsStats("/stats");    // WebSocket for statistics
AsyncWebSocket wsAudio("/audio");    // WebSocket for audio (browser fallback)
WiFiUDP udpAudio;
OpusEncoder* encoder;
int16_t* audioBuffers[NUM_BUFFERS];
volatile int writeBuffer = 0;
volatile int readBuffer = 0;
volatile bool bufferReady[NUM_BUFFERS] = {false};
SemaphoreHandle_t bufferSemaphore;

uint32_t packetSequence = 0;
uint32_t lastStatsUpdate = 0;
uint32_t lastAudioSend = 0;
uint32_t sessionStartTime = 0;
uint32_t packetsThisSecond = 0;
uint32_t bytesThisSecond = 0;
uint32_t udpDropped = 0;

// Function declarations
void audioTask(void *parameter);
void transmitTask(void *parameter);
void statsTask(void *parameter);
uint16_t calculateRMS(int16_t* buffer, size_t samples);

esp_err_t setupI2S() {
    const i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = (i2s_bits_per_sample_t)SAMPLE_BITS,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 256
    };

    const i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD_PIN
    };

    esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) return err;
    
    return i2s_set_pin(I2S_PORT, &pin_config);
}

void audioTask(void *parameter) {
    while(1) {
        size_t bytesRead = 0;
        i2s_read(I2S_PORT, audioBuffers[writeBuffer], BUFFER_SIZE_SAMPLES * sizeof(int16_t), &bytesRead, portMAX_DELAY);

        if (bytesRead > 0) {
            bufferReady[writeBuffer] = true;
            writeBuffer = (writeBuffer + 1) % NUM_BUFFERS;
            xSemaphoreGive(bufferSemaphore);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void transmitTask(void *parameter) {
    uint8_t opusBuffer[512];
    
    while(1) {
        if (xSemaphoreTake(bufferSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (bufferReady[readBuffer]) {
                bufferReady[readBuffer] = false;
                
                uint32_t currentTime = millis();
                
                // Rate limiting for audio transmission
                if ((currentTime - lastAudioSend) >= AUDIO_SEND_INTERVAL) {
                    // Calculate RMS for statistics
                    stats.rmsLevel = calculateRMS(audioBuffers[readBuffer], OPUS_FRAME_SIZE);
                    
                    // Encode OPUS frame (960 samples = 20ms)
                    int encodedSize = opus_encode(encoder, audioBuffers[readBuffer], OPUS_FRAME_SIZE, opusBuffer, sizeof(opusBuffer));
                    
                    if (encodedSize > 0) {
                        // Create UDP packet
                        size_t packetSize = sizeof(UdpAudioPacket) + encodedSize;
                        uint8_t* packet = (uint8_t*)malloc(packetSize);
                        UdpAudioPacket* audioPacket = (UdpAudioPacket*)packet;
                        
                        audioPacket->magic = AUDIO_MAGIC;
                        audioPacket->sequence = packetSequence++;
                        audioPacket->timestamp = currentTime;
                        audioPacket->dataSize = encodedSize;
                        audioPacket->codec = 1; // OPUS
                        audioPacket->channels = 1;
                        audioPacket->sampleRate = SAMPLE_RATE;
                        memcpy(audioPacket->data, opusBuffer, encodedSize);
                        
                        // Broadcast via UDP
                        udpAudio.beginPacket(IPAddress(255,255,255,255), UDP_AUDIO_PORT);
                        size_t written = udpAudio.write(packet, packetSize);
                        bool udpSuccess = udpAudio.endPacket();
                        
                        if (!udpSuccess || written != packetSize) {
                            udpDropped++;
                        }
                        
                        // Also send via WebSocket for browser compatibility
                        if (wsAudio.count() > 0) {
                            wsAudio.binaryAll(packet, packetSize);
                        }
                        
                        // Update statistics
                        packetsThisSecond++;
                        bytesThisSecond += packetSize;
                        stats.totalPackets++;
                        stats.totalBytes += packetSize;
                        stats.compressionRatio = (float)(OPUS_FRAME_SIZE * 2) / encodedSize;
                        stats.udpPacketsDropped = udpDropped;
                        
                        free(packet);
                        lastAudioSend = currentTime;
                    }
                }
                readBuffer = (readBuffer + 1) % NUM_BUFFERS;
            }
        }
    }
}

void statsTask(void *parameter) {
    while(1) {
        uint32_t now = millis();
        
        if (now - lastStatsUpdate >= STATS_SEND_INTERVAL) {
            // Update statistics
            stats.packetsPerSec = packetsThisSecond * (1000 / STATS_SEND_INTERVAL);
            stats.bytesPerSec = bytesThisSecond * (1000 / STATS_SEND_INTERVAL);
            stats.uptime = (now - sessionStartTime) / 1000;
            stats.connectedClients = wsStats.count() + wsAudio.count();
            
            // Send statistics via WebSocket
            if (wsStats.count() > 0) {
                WsStatsMessage statsMsg;
                strcpy(statsMsg.type, "stats");
                statsMsg.data = stats;
                wsStats.binaryAll((uint8_t*)&statsMsg, sizeof(statsMsg));
            }
            
            // Reset counters
            packetsThisSecond = 0;
            bytesThisSecond = 0;
            lastStatsUpdate = now;
            
            // Debug output
            Serial.printf("Stats: %d pps, %.1f KB/s, RMS: %d, Clients: %d, UDP drops: %d\n", 
                         stats.packetsPerSec, stats.bytesPerSec/1024.0, stats.rmsLevel, 
                         stats.connectedClients, stats.udpPacketsDropped);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

uint16_t calculateRMS(int16_t* buffer, size_t samples) {
    uint64_t sum = 0;
    for (size_t i = 0; i < samples; i++) {
        int32_t sample = buffer[i];
        sum += sample * sample;
    }
    return (uint16_t)sqrt(sum / samples);
}

void onWsStatsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len) {
    switch(type) {
        case WS_EVT_CONNECT:
            Serial.printf("Stats WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
            // Send initial stats immediately
            if (wsStats.count() > 0) {
                WsStatsMessage statsMsg;
                strcpy(statsMsg.type, "stats");
                statsMsg.data = stats;
                client->binary((uint8_t*)&statsMsg, sizeof(statsMsg));
            }
            break;
        case WS_EVT_DISCONNECT:
            Serial.printf("Stats WebSocket client #%u disconnected\n", client->id());
            break;
        case WS_EVT_DATA:
            // Handle incoming commands if needed
            break;
        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            break;
    }
}

void onWsAudioEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len) {
    switch(type) {
        case WS_EVT_CONNECT:
            Serial.printf("Audio WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
            break;
        case WS_EVT_DISCONNECT:
            Serial.printf("Audio WebSocket client #%u disconnected\n", client->id());
            break;
        case WS_EVT_DATA:
        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            break;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Hybrid Audio Streamer Starting...");
    Serial.println("UDP Audio + WebSocket Statistics");
    
    // Initialize WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\nConnected to WiFi: %s\n", WiFi.localIP().toString().c_str());

    // Initialize LittleFS
    if(!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed");
        return;
    }

    // Setup I2S
    if (setupI2S() != ESP_OK) {
        Serial.println("Failed to setup I2S");
        return;
    }
    Serial.println("I2S initialized successfully");

    // Initialize OPUS encoder
    int error;
    encoder = opus_encoder_create(SAMPLE_RATE, 1, OPUS_APPLICATION_AUDIO, &error);
    if (!encoder || error != OPUS_OK) {
        Serial.printf("Failed to create OPUS encoder: %d\n", error);
        return;
    }
    opus_encoder_ctl(encoder, OPUS_SET_BITRATE(128000));
    opus_encoder_ctl(encoder, OPUS_SET_COMPLEXITY(7));
    opus_encoder_ctl(encoder, OPUS_SET_SIGNAL(OPUS_SIGNAL_MUSIC));
    Serial.println("OPUS encoder initialized (128kbps, complexity 7)");

    // Allocate audio buffers
    for (int i = 0; i < NUM_BUFFERS; i++) {
        audioBuffers[i] = (int16_t*)malloc(BUFFER_SIZE_SAMPLES * sizeof(int16_t));
        if (!audioBuffers[i]) {
            Serial.printf("Failed to allocate buffer %d\n", i);
            return;
        }
    }

    // Create semaphore
    bufferSemaphore = xSemaphoreCreateBinary();
    if (!bufferSemaphore) {
        Serial.println("Failed to create semaphore");
        return;
    }

    // Initialize UDP for audio broadcasting
    udpAudio.begin(UDP_AUDIO_PORT);
    Serial.printf("UDP audio broadcasting on port %d\n", UDP_AUDIO_PORT);

    // Setup HTTP server routes
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(LittleFS, "/index.html", "text/html");
    });

    // Configuration endpoint
    server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest *request){
        String response = "{";
        response += "\"sampleRate\":" + String(SAMPLE_RATE);
        response += ",\"frameSize\":" + String(OPUS_FRAME_SIZE);
        response += ",\"udpAudioPort\":" + String(UDP_AUDIO_PORT);
        response += ",\"udpStatsPort\":" + String(UDP_STATS_PORT);
        response += ",\"buffers\":" + String(NUM_BUFFERS);
        response += ",\"audioInterval\":" + String(AUDIO_SEND_INTERVAL);
        response += ",\"statsInterval\":" + String(STATS_SEND_INTERVAL);
        response += ",\"magic\":\"0x" + String(AUDIO_MAGIC, HEX) + "\"";
        response += "}";
        request->send(200, "application/json", response);
    });

    // Serve static files
    server.serveStatic("/", LittleFS, "/");

    // Setup WebSockets
    wsStats.onEvent(onWsStatsEvent);
    wsAudio.onEvent(onWsAudioEvent);
    server.addHandler(&wsStats);
    server.addHandler(&wsAudio);
    
    server.begin();
    Serial.println("HTTP Server and WebSockets started");
    Serial.println("Statistics WebSocket: /stats");
    Serial.println("Audio WebSocket: /audio (browser fallback)");

    // Initialize statistics
    sessionStartTime = millis();
    lastStatsUpdate = sessionStartTime;
    lastAudioSend = sessionStartTime;
    memset(&stats, 0, sizeof(stats));

    // Create tasks
    xTaskCreate(audioTask, "AudioCapture", TASK_STACK_SIZE, NULL, 3, NULL);
    xTaskCreate(transmitTask, "AudioTransmit", TASK_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(statsTask, "StatsTransmit", TASK_STACK_SIZE, NULL, 1, NULL);
    
    Serial.println("All tasks started!");
    Serial.printf("Audio: UDP broadcast on port %d + WebSocket /audio\n", UDP_AUDIO_PORT);
    Serial.printf("Stats: WebSocket /stats every %dms\n", STATS_SEND_INTERVAL);
}

void loop() {
    // Cleanup WebSocket connections
    wsStats.cleanupClients();
    wsAudio.cleanupClients();
    vTaskDelay(pdMS_TO_TICKS(1000));
}
