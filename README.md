# ESP32-S3 I2S Microphone WebRTC Streamer

A modern web-based audio streaming solution using ESP32-S3 and I2S MEMS microphone. This project streams audio directly to a web browser using WebRTC and OPUS compression.

## Features

- Real-time audio streaming using WebRTC
- OPUS audio compression for efficient bandwidth usage
- Web interface hosted from ESP32-S3
- Support for ESP32-S3 with PSRAM
- I2S MEMS microphone support (INMP441)

## Hardware Requirements

- ESP32-S3 board
- INMP441 I2S MEMS Microphone

## Wiring

Connect the INMP441 microphone to the ESP32 following this:

### ESP32 (esp-wrover-kit, lolin_d32)

| INMP441 | ESP32    |
|---------|----------|
| VDD     | 3.3V     |
| GND     | GND      |
| SD      | GPIO32   |
| WS      | GPIO25   |
| SCK     | GPIO26   |
| L/R     | not connected |

### ESP32-S3 (esp32-s3-devkitc-1)

| INMP441 | ESP32-S3 |
|---------|----------|
| VDD     | 3.3V     |
| GND     | GND      |
| SD      | GPIO6    |
| WS      | GPIO45   |
| SCK     | GPIO5    |
| L/R     | not connected |

## Software Requirements

- PlatformIO IDE (VS Code extension)
- ESP32-S3 board support package
- Required libraries (automatically installed by PlatformIO)

## Installation

1. Clone this repository:
```bash
git clone https://github.com/tekk/ESP32-I2S-microphone-over-network.git
cd ESP32-I2S-microphone-over-network
```

2. Configure WiFi:
- Edit file `src/wifi_config.h` with your WiFi credentials:
```cpp
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
```

3. Upload the web interface to SPIFFS:
```bash
pio run -t uploadfs
```

4. Build and upload the firmware:
```bash
pio run -t upload
```

## Usage

1. After uploading, open the Serial Monitor (115200 baud) to see the ESP32's IP address
2. Open a web browser and navigate to the ESP32's IP address
3. Click "Start Stream" to begin audio streaming
4. The audio will play directly in your browser

## Web Interface

The web interface provides:
- Start/Stop streaming controls
- Connection status indicator
- Automatic WebRTC connection handling
- Mobile-friendly design

### Benefits:
✅ Reliable: Pure JavaScript, works in all browsers
✅ Real-time: Direct PCM streaming with minimal latency
✅ Simpler code: Much easier to debug and maintain
✅ Better compatibility: Works everywhere without dependencies

Performance:
✅ Lower latency: ~10.7ms audio chunks vs 200ms
✅ Real-time performance: Separated capture and transmission tasks
✅ No audio gaps: Double buffering ensures continuous stream
✅ Better responsiveness: 10ms WebSocket intervals

System Efficiency:
✅ Non-blocking capture: Audio never stops for WebSocket transmission
✅ Priority-based: Audio capture gets higher CPU priority
✅ Memory efficient: Only 2KB total audio buffer memory
✅ Thread-safe: Semaphore prevents race conditions

Audio Quality:
✅ 96kHz sample rate: High-quality audio maintained
✅ Continuous streaming: No stuttering or dropouts
✅ Low jitter: Consistent timing with double buffering

## Technical Details

- Audio Format:
  - Sample Rate: 48kHz
  - Bit Depth: 16-bit
  - Channels: Mono
  - Compression: OPUS (64kbps)

- WebRTC Configuration:
  - WebSocket port: 81
  - HTTP port: 80

## Troubleshooting

1. If the web interface doesn't load:
   - Check if SPIFFS upload was successful
   - Verify the ESP32's IP address in Serial Monitor
   - Ensure you're on the same network as the ESP32

2. If audio doesn't stream:
   - Check microphone connections
   - Verify I2S pin configuration
   - Check browser console for WebRTC errors

3. If WiFi doesn't connect:
   - Verify WiFi credentials in wifi_config.h
   - Check if the ESP32 is in range of the WiFi network
   - Try power cycling the ESP32

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Original UDP streaming code by @aleiei
- WebRTC implementation based on ESP32 WebRTC examples
- OPUS codec implementation by sh123

## Donate

- Written by @tekk

<form action="https://www.paypal.com/donate" method="post" target="_top">
<input type="hidden" name="hosted_button_id" value="5SPJW8Y4G2CKJ" />
<input type="image" src="https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif" border="0" name="submit" title="PayPal - The safer, easier way to pay online!" alt="Donate with PayPal button" />
<img alt="" border="0" src="https://www.paypal.com/en_SK/i/scr/pixel.gif" width="1" height="1" />
</form>
