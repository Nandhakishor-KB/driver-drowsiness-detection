# Driver Drowsiness Detection Project

## Overview
This project is designed to detect driver drowsiness using a Raspberry Pi and a camera module. It uses OpenCV's Haar Cascade classifiers to detect the face and eyes, then calculates the Eye Aspect Ratio (EAR) to determine if the driver is getting drowsy. If drowsiness is detected, it triggers an alarm buzzer and controls a motor driver for additional alerts.

## Features
- Real-time face and eye detection using Haar Cascades
- Eye Aspect Ratio (EAR) calculation to estimate drowsiness
- Alarm buzzer activation upon detecting drowsiness
- Motor control using L298N driver for physical alerting
- Runs on Raspberry Pi with camera module

## Hardware Required
- Raspberry Pi 4 (or similar)
- Raspberry Pi Camera Module
- Buzzer
- L298N Motor Driver Module
- DC Motor
- Jumper wires
- Power supply (5V 3A adapter or good quality power bank)

## How to Run This Code on Raspberry Pi

### 1. Raspberry Pi OS Installation

- Download the official Raspberry Pi OS from https://www.raspberrypi.com/software/
- Use Raspberry Pi Imager or a tool like Balena Etcher to flash the OS image to a microSD card (16GB or more recommended).
- Insert the microSD card into your Raspberry Pi and power it on.
- Follow the on-screen instructions to complete the initial setup (language, Wi-Fi, username, password, etc.).

### 2. Required Libraries and Setup

Open the terminal on your Raspberry Pi and run these commands to install the necessary software:

```bash
sudo apt update
sudo apt install python3-pip python3-opencv python3-picamera2 python3-scipy python3-rpi.gpio
pip3 install numpy
```

These packages are required for the project:

- python3-opencv for image processing
- python3-picamera2 to access the Raspberry Pi camera module
- python3-scipy for calculations like distance measurement
- python3-rpi.gpio for controlling GPIO pins (buzzer, motor)
- numpy for handling arrays and math

### 3.Haar Cascade XML Files
The project uses Haar cascade files (haarcascade_frontalface_default.xml and haarcascade_eye.xml) for face and eye detection. These files are usually included with OpenCV, but if not, you can download them from the official OpenCV GitHub repository. Place the XML files in the same directory as your Python code or update the code to point to their location

### 4. Hardware Connections

#### Raspberry Pi Camera Module
- Connect the official Raspberry Pi Camera Module to the CSI port on the Raspberry Pi.
- Enable the camera interface by running:
  ```bash
  sudo raspi-config
  ```
  Then navigate to Interface Options > Camera and enable it.
- Reboot if prompted.

#### Buzzer Connection
- Connect the buzzer positive pin to GPIO18 (physical pin 12).
- Connect the buzzer negative pin to any GND (ground) pin on the Raspberry Pi.

#### Motor Driver (L298N) Connection
- Connect IN3 to GPIO23 (physical pin 16).
- Connect IN4 to GPIO24 (physical pin 18).
- Connect EN (enable) pin to GPIO25 (physical pin 22).
- Connect the motor power supply and motor terminals as per the L298N datasheet.
- Connect the L298N ground to the Raspberry Pi ground.

### 5. Powering the Raspberry Pi

- Use a 5V, 3A power adapter for a stable power supply.
- You can also use a good quality power bank with 5V output and at least 2.5A current capacity.
- Avoid powering the Pi from a USB port on other devices as it might not provide enough current.

## How to Run the Script

- Save the main Python script (for example, as `drowsiness_detection.py`).
- Open the terminal and navigate to the folder where the script is saved.
- Run the script using:
  ```bash
  python3 drowsiness_detection.py
  ```
- The camera window will open and start detecting drowsiness.
- Press 'q' to quit the program.

---

If you face any issues with permissions or camera access, make sure your user is part of the `video` group or run the script with `sudo` as needed.
