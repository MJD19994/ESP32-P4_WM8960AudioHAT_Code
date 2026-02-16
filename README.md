# WM8960 Audio HAT - ESP32-P4 Functional Code

Functional code to get a Raspberry Pi compatible WM8960 Audio HAT working with the ESP32-P4 development board with matching GPIO header. This project was developed using VS Code with ESP-IDF.

## Overview

This example demonstrates how to interface the Waveshare WM8960 Audio HAT (or compatible boards) with the ESP32-P4 using I2S for digital audio and I2C for codec configuration. The code provides:

- **Audio Playback**: Square wave tone generation for testing DAC output
- **Audio Recording**: Microphone input monitoring via ADC
- **Audio Loopback**: Real-time microphone to speaker echo (60-second test)

The ESP32-P4 acts as the I2S master, generating BCLK and LRCLK clocks, while the WM8960 operates in peripheral (slave) mode.

## Hardware Requirements

- **ESP32-P4 Development Board** with Raspberry Pi compatible GPIO header
- **WM8960 Audio HAT** (Waveshare or compatible)
- Speakers or headphones connected to the HAT
- Microphone (if using recording/loopback features)
- USB cable for programming and debugging

## GPIO Pin Mapping

| Function    | ESP32-P4 GPIO | WM8960 HAT Pin | Description                           |
|-------------|---------------|----------------|---------------------------------------|
| I2C SDA     | GPIO 7        | SDA            | I2C data for codec configuration      |
| I2C SCL     | GPIO 8        | SCL            | I2C clock for codec configuration     |
| I2S BCLK    | GPIO 22       | BCLK           | Bit clock (ESP32 master output)       |
| I2S LRCLK   | GPIO 53       | LRCLK/WS       | Left/Right clock / Word Select        |
| I2S DOUT    | GPIO 6        | I2S_DAC        | Audio data output (ESP32 → DAC)       |
| I2S DIN     | GPIO 27       | I2S_ADC        | Audio data input (ADC → ESP32)        |

**Note:** The WM8960 I2C address is `0x1A`.

## Software Requirements

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/get-started/) (v5.x recommended)
- VS Code with ESP-IDF extension (recommended) or command line tools
- WM8960 ESP-IDF component/driver

## Project Setup

### 1. Install ESP-IDF

Follow the official ESP-IDF installation guide for your operating system:
- [ESP-IDF Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/get-started/)

### 2. Clone This Repository

```bash
git clone https://github.com/MJD19994/WM8960_HAT_Functional_Code.git
cd WM8960_HAT_Functional_Code
```

### 3. Add WM8960 Component

This project requires the WM8960 ESP-IDF driver. Add it as a component in your project's `components` directory or via the ESP Component Registry.

### 4. Configure the Project

Open the project in VS Code with ESP-IDF extension or use:

```bash
idf.py set-target esp32p4
idf.py menuconfig
```

### 5. Build and Flash

```bash
idf.py build
idf.py -p [PORT] flash monitor
```

Replace `[PORT]` with your serial port (e.g., `/dev/ttyUSB0` on Linux, `COM3` on Windows).

## Code Structure

```
WM8960_Example.c
├── I2C Initialization      - Sets up I2C master for codec control
├── WM8960 Codec Init       - Configures all codec registers for I2S audio
│   ├── Power Management    - VREF, VMID, MIC bias
│   ├── Input Signal Path   - Microphone PGA, boost mixer
│   ├── Clock Configuration - PLL setup for 44.1kHz sample rate
│   ├── I2S Mode Setup      - Peripheral mode, 16-bit, Philips format
│   ├── DAC/ADC Enable      - Digital converters and volume
│   └── Output Configuration- Speaker amp, headphone amp, mixer routing
├── I2S Initialization      - Sets up I2S in standard (Philips) mode
├── Audio Tasks
│   ├── audio_playback_task - Square wave generator for DAC testing
│   ├── audio_recording_task- Microphone level monitoring
│   └── audio_loopback_task - Real-time mic → speaker echo
└── Debug Utilities         - WM8960 register dump for troubleshooting
```

## Key Configuration Settings

| Setting          | Value   | Description                              |
|------------------|---------|------------------------------------------|
| Sample Rate      | 44100 Hz| CD-quality audio                         |
| Bit Depth        | 16-bit  | Standard audio resolution                |
| I2S Format       | Philips | Standard I2S format (FORMAT=10)          |
| I2C Frequency    | 100 kHz | Standard mode I2C                        |
| PLL Mode         | Enabled | Internal PLL for clock generation        |

## Troubleshooting

### No Audio Output

1. **Check DAC Mute**: Ensure `wm8960_disableDacMute()` is called
2. **Verify Routing**: Check that `LD2LO` and `RD2RO` are enabled (DAC to output mixer)
3. **Power Management**: Confirm VREF, VMID, and DAC power are enabled
4. **Check Volumes**: DAC and speaker/headphone volumes should be non-zero

### No Microphone Input

1. **Enable MIC Bias**: Required for electret microphones
2. **Check Input Path**: LMIC/RMIC, LMN1/RMN1, and input mutes should be configured
3. **Boost Settings**: Adjust `LMICBOOST` and `RMICBOOST` if signal is too weak
4. **ADC Power**: Ensure ADC left/right are enabled

### Clock Issues

1. **PLL Configuration**: Must be configured BEFORE enabling
2. **CLKSEL**: Should be set to PLL (value = 1) after PLL is enabled
3. **SYSCLKDIV**: Check divider settings match your sample rate requirements

### GPIO Diagnostic

The code includes GPIO diagnostic tests at startup to verify pin functionality. Check serial output for voltage measurements.

## Resources

- [WM8960 Datasheet](https://www.cirrus.com/products/wm8960/)
- [ESP32-P4 Technical Reference](https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/)
- [ESP-IDF I2S Driver Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/api-reference/peripherals/i2s.html)
- [Waveshare WM8960 Audio HAT Wiki](https://www.waveshare.com/wiki/WM8960_Audio_HAT)

## License

This code is provided as-is for educational and development purposes.

## Contributing

Feel free to submit issues and pull requests for improvements.
