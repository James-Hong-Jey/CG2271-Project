# CG2271 Real-Time Operating Systems
This project's goal is to design a remote-controlled car to navigate an obstacle course in the fastest time possible while performing certain tasks like making lights flash in a pattern and playing music. The car is built using an ESP32 microcontroller and KL25Z board, which runs on an open source Real-Time Operating System (RTOS) known as RTX, and uses ARM architecture. 

## Demo Video for CG2271
[![Watch the video](https://img.youtube.com/vi/fmYNUrjkqvc/0.jpg)](https://www.youtube.com/watch?v=fmYNUrjkqvc)

## Navigation
`Project Specifications` contains details about what the project is supposed to achieve.

`ESP32_Bluepad_Controller` contains a `.ino` file for the ESP32 to connect to a generic controller over bluetooth and subsequently communicate to the KL25Z over UART.

`Integration` contains the final code for our run shown in the video.

`Archive` contains the minimally viable code for the KL25Z, compiled with Keil uVision software. The final version of the code is in `Integration`. The archive also contains the template code and previous labs' work.

## Acknowledgements
[Bluepad32 by Ricardo Quesada](https://github.com/ricardoquesada/bluepad32)