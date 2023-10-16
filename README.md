# SwarMBots Firmware Installation Guide

## Prerequisites
- Install [VS Code](https://code.visualstudio.com/download)
- Install [Espressif IDF VS Code extension](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension)

## Installation Steps
### Configure Espressif IDF VS Code Extension
1. Open the Espressif IDF VS Code extension and select the latest version of esp-idf (v5.1.1).
2. Choose the location where you would like to store the IDF and its various submodules on your computer.
3. Select the Python environment you want esp-idf to use.

### Install, build, and upload firmware
1. Clone the SwarMBots-Firmware repository and open it in VS Code.
2. Open an ESP-IDF terminal by pressing the "Open ESP-IDF Terminal" button at the bottom of the VS Code window.
3. Run the setup script.
   ```
   chmod +x setup.sh
   source ./setup.sh
   ```
4. Run the build script
   ```
   chmod +x build.sh
   ./build.sh [client|host|tests|all]
   ```
5. Upload the image to the ESP32 device.
   ```
   cd [client|host|tests/*]
   idf.py -p [PORT] flash
   ```
