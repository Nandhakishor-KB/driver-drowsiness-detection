# Driver Drowsiness Detection and Alert System

## Overview

This project is designed to detect signs of drowsiness in a driver using computer vision and raise alerts to prevent accidents. The system uses OpenCV and Haar Cascade classifiers to monitor the driver's eyes through a live camera feed.

## Features

- Real-time eye monitoring
- Drowsiness detection using Haar Cascade Classifier
- Audio alert when drowsiness is detected
- Lightweight and suitable for running on Raspberry Pi

## Hardware Used

- Raspberry Pi 4 (or any model with camera support)
- Raspberry Pi Camera Module or USB webcam
- Buzzer (for audio alert)
- Optional: LCD or display for status messages

## Software and Tools

- Python 3
- OpenCV
- Haar Cascade Classifier XML files
- Pygame (for playing sound)
- OS: Raspberry Pi OS / Linux / Windows

## How It Works

1. The camera continuously monitors the driver's eyes.
2. Eye detection is done using Haar Cascade classifiers.
3. If the eyes are closed for a certain number of frames, the system detects drowsiness.
4. An alarm is triggered through a buzzer or speaker.

## Usage

- Run the Python script on the Raspberry Pi (or your system with camera access).
- Make sure required libraries like OpenCV and Pygame are installed.
- The system will start monitoring and alert on drowsiness.

## Future Improvements

- Add face orientation tracking
- Use deep learning models for better accuracy
- Store data logs for analysis

## Credits

Developed by Nandhakishor K B  
Final year project under the Electronics and Communication Engineering stream.
