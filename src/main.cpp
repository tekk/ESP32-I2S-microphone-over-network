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
#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
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

#define SAMPLE_RATE 96000
#define SAMPLE_BITS I2S_BITS_PER_SAMPLE_16BIT
#define AUDIO_BUFFER_SIZE 4800  // Larger buffer: ~50ms worth of audio (96000 * 0.05)
#define NUM_BUFFERS 2           // Double buffering
#define TASK_STACK_SIZE 16384   // Reduced stack size
#define WEBSOCKET_INTERVAL 50   // Send WebSocket data every 50ms (20 packets/sec)

// WebSocket and HTTP server
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Double buffering for audio
int16_t* audioBuffers[NUM_BUFFERS];
volatile int currentBuffer = 0;
volatile bool bufferReady[NUM_BUFFERS] = {false, false};
SemaphoreHandle_t bufferSemaphore;

// Function declarations
void audioTask(void *parameter);
void mainLoopTask(void *parameter);

esp_err_t setupI2S() {
    
    esp_err_t err = ESP_OK;

    const i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = (i2s_bits_per_sample_t)SAMPLE_BITS,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S | I2S_COMM_FORMAT_STAND_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 16, // Maximum DMA buffers for smoothest audio
        .dma_buf_len = 128   // Maximum DMA buffer length
    };

    const i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD_PIN
    };

    err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

    if (err != ESP_OK) {
        Serial.printf("Failed to install I2S driver: %d\n", err);
        return err;
    }

    err = i2s_set_pin(I2S_PORT, &pin_config);

    if (err != ESP_OK) {
        Serial.printf("Failed to set I2S pins: %d\n", err);
        return err;
    }

    return err;
}

void audioTask(void *parameter) {
    while(1) {
        // Read I2S data into current buffer
        size_t bytesRead = 0;
        int bufferIndex = currentBuffer;
        i2s_read(I2S_PORT, audioBuffers[bufferIndex], AUDIO_BUFFER_SIZE * sizeof(int16_t), &bytesRead, portMAX_DELAY);

        if (bytesRead > 0) {
            // Mark buffer as ready
            bufferReady[bufferIndex] = true;
            
            // Switch to next buffer
            currentBuffer = (currentBuffer + 1) % NUM_BUFFERS;
            
            // Signal that a buffer is ready
            xSemaphoreGive(bufferSemaphore);
        }
        
        // Small delay to prevent tight loop
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len) {
    switch(type) {
        case WS_EVT_CONNECT:
            Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
            break;
        case WS_EVT_DISCONNECT:
            Serial.printf("WebSocket client #%u disconnected\n", client->id());
            break;
        case WS_EVT_DATA:
            // Handle incoming WebSocket data
            break;
        case WS_EVT_PONG:
        case WS_EVT_ERROR:
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
    Serial.println();
    Serial.print("Connected to WiFi network with IP Address: ");
    Serial.println(WiFi.localIP());

    // Initialize LittleFS
    if(!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed");
        return;
    } else {
        Serial.println("LittleFS Mounted");
    }

    // Setup I2S
    esp_err_t err = setupI2S();
    if (err != ESP_OK) {
        Serial.printf("Failed to setup I2S: %d\n", err);
        return;
    } else {
        Serial.println("I2S Setup Success");
    }
    
    // Allocate double buffers for raw PCM data
    for (int i = 0; i < NUM_BUFFERS; i++) {
        audioBuffers[i] = (int16_t*)malloc(AUDIO_BUFFER_SIZE * sizeof(int16_t));
        if (audioBuffers[i] == NULL) {
            Serial.printf("Failed to allocate memory for audio buffer %d\n", i);
            return;
        }
    }
    
    // Create semaphore for buffer synchronization
    bufferSemaphore = xSemaphoreCreateBinary();
    if (bufferSemaphore == NULL) {
        Serial.println("Failed to create buffer semaphore");
        return;
    }
    
    Serial.println("Double audio buffers allocated successfully");

    // Setup web server routes
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(LittleFS, "/index.html", "text/html");
    });

    // Serve any other files from LittleFS
    server.serveStatic("/", LittleFS, "/");

    // Setup WebSocket
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    
    server.begin();

    // Create audio capture task (higher priority)
    xTaskCreate(
        audioTask,       // Task function
        "AudioCapture",  // Task name
        TASK_STACK_SIZE, // Stack size
        NULL,           // Task parameters
        2,              // Higher priority for audio capture
        NULL            // Task handle
    );
    
    // Create main processing task
    xTaskCreate(
        mainLoopTask,    // Task function
        "MainLoop",      // Task name
        TASK_STACK_SIZE, // Stack size
        NULL,           // Task parameters
        1,              // Lower priority for processing
        NULL            // Task handle
    );
}

void mainLoopTask(void *parameter) {
    TickType_t lastWebSocketTime = 0;
    size_t totalBytesProcessed = 0;
    
    while(1) {
        // Wait for a buffer to be ready
        if (xSemaphoreTake(bufferSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
            TickType_t currentTime = xTaskGetTickCount();
            
            // Find and process ready buffer
            for (int i = 0; i < NUM_BUFFERS; i++) {
                if (bufferReady[i]) {
                    bufferReady[i] = false;
                    
                    // Send data if we have connected clients and enough time has passed
                    if (ws.count() > 0 && (currentTime - lastWebSocketTime) >= pdMS_TO_TICKS(WEBSOCKET_INTERVAL)) {
                        size_t dataSize = AUDIO_BUFFER_SIZE * sizeof(int16_t);
                        
                        // Check if WebSocket can accept more data (simple backpressure)
                        bool canSend = true;
                        if (ws.count() > 0) {
                            // Simple check: if we have clients, limit sending frequency
                            static uint32_t skipCounter = 0;
                            skipCounter++;
                            if (skipCounter % 2 == 0) {  // Send every other buffer to reduce load
                                canSend = true;
                            } else {
                                canSend = false;
                            }
                        }
                        
                        if (canSend) {
                            ws.binaryAll((uint8_t*)audioBuffers[i], dataSize);
                            lastWebSocketTime = currentTime;
                            totalBytesProcessed += dataSize;
                            
                            // Print debug info occasionally
                            if (totalBytesProcessed % (SAMPLE_RATE * 2) == 0) {  // Every second
                                Serial.printf("Audio streaming: %u bytes/sec, %d clients, latency: ~%dms\n", 
                                             totalBytesProcessed, ws.count(), (AUDIO_BUFFER_SIZE * 1000) / SAMPLE_RATE);
                                totalBytesProcessed = 0;
                            }
                        } else {
                            // Skip this buffer to prevent queue overflow
                            Serial.println("Skipping audio packet - WebSocket queue full");
                        }
                    }
                    break;
                }
            }
        }
        
        // WebSocket cleanup
        ws.cleanupClients();
    }
}

// Empty loop() since we're using a separate task
void loop() {

}
