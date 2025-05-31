Driver Drowsiness Detection and Alert System

This project is designed to detect signs of driver fatigue using computer vision and alert the driver in real time. The goal is to help reduce accidents caused by drowsy driving by continuously monitoring the driver’s eye activity.

---

Overview

Driver drowsiness is a serious concern, especially during long drives or at night. This system uses a camera to detect whether the driver's eyes are open or closed. If the eyes remain closed for a certain period, the system assumes the driver is drowsy and triggers an alert to wake them up. This is a non-invasive and low-cost solution that can be integrated into various types of vehicles.

---

Features

- Real-time video processing using OpenCV
- Eye state detection using Haar Cascade Classifier
- Continuous monitoring to detect eye closure duration
- Buzzer alert if drowsiness is detected
- Simple Python-based implementation

---

Hardware Used

- Raspberry Pi (or any computer capable of running Python and a webcam)
- USB Webcam or Pi Camera module
- Buzzer for alert output
- Jumper wires and breadboard (optional for buzzer connection)

---

Software Requirements

- Python 3
- OpenCV (`opencv-python` library)
- Haar Cascade XML file for eye detection
- GPIO library if using Raspberry Pi for buzzer control

---

How It Works

1. The camera captures video frames in real time.
2. Each frame is analyzed to detect the driver's eyes using Haar Cascade classifiers.
3. The system checks whether the eyes are open or closed.
4. A counter keeps track of how many consecutive frames the eyes are closed.
5. If the eyes remain closed for a predefined threshold (e.g., 20 frames), a buzzer is triggered to alert the driver.

---

Installation and Setup

1. **Install dependencies:**

   ```bash
   pip install opencv-python
