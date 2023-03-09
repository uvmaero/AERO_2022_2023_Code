# AERO_2022-2023
All of the code that drives each board and runs the car for the year 2022-2023!

## Overview
Each board is managed by an ESP32-WROOM, in the form of the official Dev-Kit-C V4 from Espressif. Its operations are managed by FreeRTOS and is clocked at 240Mhz.

## Boards
### Front Board
#### Description
Located in the front of the car, this board manages all driver input, the remote data aquisition system, and wireless communications with the steering wheel.
#### I/O
##### Inputs
- Accelorator pedal
- Brake pedal 
- Ready to drive button
- front wheel hall effect sensors
- front wheel ride height potentiometers
- CAN bus
##### Outputs
- CAN bus
- LoRa Radio
- ESP NOW data to wheel board
- Ready to drive LED
- Ready to drive buzzer 
#### Libraries
- LoRa

### Rear Board
#### Description
#### I/O
#### Libraries

### Wheel Board
#### Description
#### I/O
#### Libraries
