# STM32-Curve25519-UART-ESP8266  

The project aims to develop a prototype system for secure key exchange based on Curve25519 for STM32 and ESP8266 using UART.  

## Description  

Currently, the partial functionality implemented allows:  
- Sending data via ESP8266.  
- Sending data via UART to a PC.  
- Generating a public key based on a private key (currently fixed value).  

Later, key exchange between ESP8266 and STM32 will be implemented.  

## Building  

### stm32  
The project uses **libopencm3** for working with the STM32 microcontroller. A Makefile is used to manage the build process.  

1. Clone the repository.  
```bash
git clone --recursive git@github.com:RenChan18/STM32-Soil-Moisture-Monitor-with-Local-Server.git
```
2. Make sure the following dependencies are installed:  
   - GCC for ARM (arm-none-eabi)  
   - Make  
3. Build the project using the command:  
```bash
make
```

### esp8266  
The work is carried out using the [ESP8266_RTOS_SDK](https://github.com/espressif/ESP8266_RTOS_SDK).  
Follow these steps to properly configure and use the SDK.  

#### Setting up and configuring a virtual environment  
It is convenient to use a virtual environment to isolate dependencies.  
1. Create a virtual environment:  
```bash
python3.11 -m venv ~/esp8266-venv
```
2. Activate the virtual environment:  
```bash
source ~/esp8266-venv/bin/activate
```

#### Cloning ESP8266_RTOS_SDK and setting up the environment  
1. Clone the SDK repository:  
```bash
git clone --recursive https://github.com/espressif/ESP8266_RTOS_SDK.git
```
2. Then, please follow the instructions in the [ESP8266_RTOS_SDK](https://github.com/espressif/ESP8266_RTOS_SDK).  

### Building the firmware  
Build the project by running:  
```bash
make
```

### Uploading firmware to ESP8266 (Flashing)  
Upload the compiled firmware to the device:  
```bash
make flash
```

### Monitoring the output (Running)  
To view the output from the device after flashing, use the command:  
```bash
make monitor
```
- To exit the monitor, press `Ctrl + ]`.  

## Connections  

- **TTL-USB Converter**: Provides serial communication between STM32 and the computer or other devices.  

#### **STM32F411VET & TTL-USB Converter (Serial Communication)**  
| **STM32F411VET** | **TTL-USB Converter** |
| :--------------: | :-------------------: |
|       TX (PA9)   |           RX          |
|       RX (PA10)  |           TX          |
|       GND        |           GND         |

