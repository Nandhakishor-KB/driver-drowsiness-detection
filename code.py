import cv2
import numpy as np
from picamera2 import Picamera2
from scipy.spatial import distance
import time
import os
import RPi.GPIO as GPIO
import threading

# ----------------------------

# 1. Haar Cascade Helper Function

# ----------------------------

def get\_haar\_path(cascade\_name):
paths = \[
'/usr/share/opencv4/haarcascades/',
'/usr/local/share/opencv4/haarcascades/',
'/usr/share/opencv/haarcascades/',
os.path.dirname(os.path.abspath(*file*)) + '/'
]

try:
    if hasattr(cv2, 'data') and hasattr(cv2.data, 'haarcascades'):
        paths.insert(0, cv2.data.haarcascades)
except Exception as e:
    print(f"Warning: Couldn't access cv2.data.haarcascades: {e}")

for path in paths:
    full_path = os.path.join(path, cascade_name)
    if os.path.exists(full_path):
        print(f"Found cascade at: {full_path}")
        return full_path

available_files = []
for path in paths:
    if os.path.exists(path):
        available_files.extend(os.listdir(path))

raise FileNotFoundError(
    f"Could not find {cascade_name} in:\n{paths}\n"
    f"Available files in these directories:\n{available_files}\n"
    "Try: sudo apt install opencv-data"
)


# ----------------------------

# 2. Main Drowsiness Detector Class

# ----------------------------

class DrowsinessDetector:
def *init*(self):
self.EYE\_AR\_THRESH = 0.25
self.DROWSINESS\_THRESHOLD = 6.0
self.MAX\_DROWSINESS = 20.0
self.INCREASE\_RATE = 0.1
self.DECREASE\_RATE = 0.1
self.FRAME\_RATE = 30

try:
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"size": (640, 480)})
        self.picam2.configure(config)

        self.drowsiness_timer = 0.0
        self.last_update_time = time.time()
        self.ALARM_ON = False
        self.MAX_REACHED = False

        face_path = get_haar_path('haarcascade_frontalface_default.xml')
        eye_path = get_haar_path('haarcascade_eye.xml')

        self.face_cascade = cv2.CascadeClassifier(face_path)
        self.eye_cascade = cv2.CascadeClassifier(eye_path)

        if self.face_cascade.empty() or self.eye_cascade.empty():
            raise ValueError("Failed to load Haar cascades (empty classifiers)")

        self.picam2.start()
        print("Drowsiness detector initialized successfully")

    except Exception as e:
        print(f"Initialization failed: {e}")
        self.cleanup()
        raise

def cleanup(self):
    if hasattr(self, 'picam2'):
        self.picam2.stop()
    cv2.destroyAllWindows()

def eye_aspect_ratio(self, eye):
    A = distance.euclidean((eye[0], eye[1]+eye[3]//4), (eye[0], eye[1]+3*eye[3]//4))
    B = distance.euclidean((eye[0]+eye[2]//4, eye[1]+eye[3]//2), 
                         (eye[0]+3*eye[2]//4, eye[1]+eye[3]//2))
    C = distance.euclidean((eye[0], eye[1]+eye[3]//2), (eye[0]+eye[2], eye[1]+eye[3]//2))
    return (A + B) / (2.0 * C)

def update_drowsiness_timer(self, face_detected, eyes_detected, ear):
    current_time = time.time()
    time_elapsed = current_time - self.last_update_time
    self.last_update_time = current_time

    if self.MAX_REACHED:
        return

    if face_detected and (not eyes_detected or ear < self.EYE_AR_THRESH):
        self.drowsiness_timer += self.INCREASE_RATE * time_elapsed * self.FRAME_RATE
    else:
        self.drowsiness_timer = max(0, self.drowsiness_timer - self.DECREASE_RATE * time_elapsed * self.FRAME_RATE)

    if self.drowsiness_timer >= self.MAX_DROWSINESS:
        self.drowsiness_timer = self.MAX_DROWSINESS
        self.MAX_REACHED = True
        self.ALARM_ON = False
        print("MAX DROWSINESS REACHED - SYSTEM DISABLED")

    elif self.drowsiness_timer >= self.DROWSINESS_THRESHOLD and not self.ALARM_ON:
        self.ALARM_ON = True
        print("DROWSINESS DETECTED!")
    elif self.drowsiness_timer < self.DROWSINESS_THRESHOLD:
        self.ALARM_ON = False

def run(self):
    try:
        while True:
            frame = self.picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
            face_detected = len(faces) > 0
            eyes_detected = False
            current_ear = 0

            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
                roi_gray = gray[y:y+h, x:x+w]

                eyes = self.eye_cascade.detectMultiScale(roi_gray, scaleFactor=1.1, minNeighbors=5, minSize=(20, 20))

                if len(eyes) == 2:
                    eyes = sorted(eyes, key=lambda e: e[0])
                    left_ear = self.eye_aspect_ratio(eyes[0])
                    right_ear = self.eye_aspect_ratio(eyes[1])
                    current_ear = (left_ear + right_ear) / 2.0
                    eyes_detected = True

                    for (ex, ey, ew, eh) in eyes:
                        cv2.rectangle(frame, (x+ex, y+ey), (x+ex+ew, y+ey+eh), (0, 255, 0), 2)

            self.update_drowsiness_timer(face_detected, eyes_detected, current_ear)

            status_text = "MAX REACHED (20.0)" if self.MAX_REACHED else f"{self.drowsiness_timer:.1f}/20.0"
            cv2.putText(frame, f"EAR: {current_ear:.2f}", (10, 30), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Drowsiness: {status_text}", (10, 60), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            if self.ALARM_ON and not self.MAX_REACHED:
                cv2.putText(frame, "ALERT: DROWSY!", (10, 90), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            elif self.MAX_REACHED:
                cv2.putText(frame, "SYSTEM DISABLED", (10, 90), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.imshow("Drowsiness Detection", frame)
            if cv2.waitKey(1) == ord('q'):
                break
    finally:
        self.cleanup()


# ----------------------------

# 3. Buzzer Controller Class

# ----------------------------

class BuzzerController:
def *init*(self, detector, buzzer\_pin=18):
self.detector = detector
self.buzzer\_pin = buzzer\_pin
GPIO.setmode(GPIO.BCM)
GPIO.setup(self.buzzer\_pin, GPIO.OUT)
self.buzzer = GPIO.PWM(self.buzzer\_pin, 2000)
self.buzzer.start(0)

def run(self):
    try:
        while True:
            if self.detector.ALARM_ON and not self.detector.MAX_REACHED:
                self.buzzer.ChangeDutyCycle(50)
                time.sleep(0.5)
                self.buzzer.ChangeDutyCycle(0)
                time.sleep(0.5)
            else:
                self.buzzer.ChangeDutyCycle(0)
                time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        self.buzzer.stop()
        GPIO.cleanup()


# ----------------------------

# 4. Motor Controller Class (L298N)

# ----------------------------

class MotorController:
def *init*(self, detector, in3=23, in4=24, enable\_pin=25):
self.detector = detector
self.in3 = in3
self.in4 = in4
self.enable = enable\_pin

GPIO.setmode(GPIO.BCM)
    GPIO.setup(self.in3, GPIO.OUT)
    GPIO.setup(self.in4, GPIO.OUT)
    GPIO.setup(self.enable, GPIO.OUT)

    self.pwm = GPIO.PWM(self.enable, 1000)  # 1kHz
    self.pwm.start(0)

def run(self):
    try:
        while True:
            if not self.detector.MAX_REACHED:
                GPIO.output(self.in3, GPIO.HIGH)
                GPIO.output(self.in4, GPIO.LOW)
                self.pwm.ChangeDutyCycle(40)  # Slow speed
            else:
                self.pwm.ChangeDutyCycle(0)
                GPIO.output(self.in3, GPIO.LOW)
                GPIO.output(self.in4, GPIO.LOW)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        self.pwm.stop()
        GPIO.cleanup()


# ----------------------------

# 5. Main Program

# ----------------------------

if *name* == "*main*":
try:
detector = DrowsinessDetector()
buzzer = BuzzerController(detector)
motor = MotorController(detector)

threading.Thread(target=detector.run, daemon=True).start()
    threading.Thread(target=motor.run, daemon=True).start()

    buzzer.run()

except Exception as e:
    print(f"Fatal error: {e}")
finally:
    print("Program terminated")
