# SwarMBots Firmware Installation Guide

## Prerequisites
- [VS Code](https://code.visualstudio.com/download) installed
- [Espressif IDF VS Code extension](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension) installed

## Installation Steps
1. Open the Espressif IDF VS Code extension and select the latest version of esp-idf (v5.1.1).
2. Choose the location where you would like to store the IDF and its various submodules on your computer.
3. Select the Python environment you want esp-idf to use.
4. Clone the SwarMBots-Firmware repository and open it in VS Code.

## Build and Upload
1. Open an ESP-IDF terminal by pressing the "Open ESP-IDF Terminal" button at the bottom of the VS Code window.
2. Go to the directory of the project you want to flash to the board, then run the ESP build and upload scripts.
```
cd [client|host|tests/*]
idf.py set-target esp32s3
idf.py build
idf.py -p [PORT] flash
```
3. If changes are made, you only need to rebuild and upload. Do not reset the target or the script will recompile all of the ESP-IDF files and it will take several minutes.
```
idf.py build
idf.py -p [PORT] flash
```