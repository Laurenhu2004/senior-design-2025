import os
import torch
import torchvision

device = torch.device('cuda')

import time
import pyaudio
import cv2
import tensorflow as tf
from google.oauth2 import service_account
from google.cloud import speech
from transformers import TFAutoModelForSequenceClassification, AutoTokenizer
from ultralytics import YOLO

import warnings
from transformers import logging

start_time = time.time()

warnings.filterwarnings("ignore")
logging.set_verbosity_error()

LABELS = {0: "cup", 1: "bottle", 2: "fork", 3: "cell phone", 4: "neutral"}
YOLO_CLASSES = {"cup": 41, "bottle": 39, "fork": 42, "cell phone": 67}

client_file = 'text_classification/speech_api_keys.json'
credentials = service_account.Credentials.from_service_account_file(client_file)

RATE = 16000
CHUNK = int(RATE / 10)
audio_interface = pyaudio.PyAudio()

config = speech.RecognitionConfig(
    encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
    sample_rate_hertz=RATE,
    language_code='en-US'
)
streaming_config = speech.StreamingRecognitionConfig(config=config, interim_results=True)

def generate_audio_chunks(stream):
    """Generator that yields audio chunks from the microphone."""
    for _ in range(100):
        data = stream.read(CHUNK, exception_on_overflow=False)
        if not data:
            break
        yield speech.StreamingRecognizeRequest(audio_content=data)
        time.sleep(0.1)

sentence = ""
try:
    stream = audio_interface.open(format=pyaudio.paInt16, channels=1, rate=RATE, input=True, frames_per_buffer=CHUNK)
    print("Listening... Say the object to detect.")
    with speech.SpeechClient(credentials=credentials) as client:
        responses = client.streaming_recognize(config=streaming_config, requests=generate_audio_chunks(stream))
        for response in responses:
            for result in response.results:
                if result.is_final:
                    sentence = result.alternatives[0].transcript
                    print("Transcript:", sentence)
                    raise StopIteration
except StopIteration:
    print("Processing the detected speech...")
except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    if stream.is_active():
        stream.stop_stream()
    stream.close()
    audio_interface.terminate()

model_save_path = 'text_classification/models/trained_v10'
model = TFAutoModelForSequenceClassification.from_pretrained(model_save_path)
tokenizer = AutoTokenizer.from_pretrained(model_save_path)

inputs = tokenizer(sentence, return_tensors="tf")
outputs = model(**inputs)
logits = outputs.logits
predicted_class_id = tf.argmax(logits, axis=-1).numpy()[0]

detected_label = LABELS.get(predicted_class_id, "neutral")
print(f"Predicted label: {detected_label}")

if detected_label == "neutral":
    print("Neutral detected. No object detection needed.")
    exit()

# Load the YOLO model
yolo_model = YOLO('yolov8s.pt')

# Move the YOLO model to the specified device (GPU or CPU)
yolo_model.to(device)  # This line will move the model to the appropriate device

cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_BRIGHTNESS, 10)
# cap.set(cv2.CAP_PROP_CONTRAST, 100)
# cap.set(cv2.CAP_PROP_SATURATION, 50)
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

object_detected = False
target_class = YOLO_CLASSES.get(detected_label, -1)

print(f"Looking for: {detected_label}")
while not object_detected:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame.")
        break
    
    results = yolo_model(frame, augment=True)
    
    for result in results:
        for box in result.boxes:
            cls = int(box.cls.item())
            if cls == target_class:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, detected_label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                print(f"{detected_label.capitalize()} detected at: ({x1}, {y1}) to ({x2}, {y2})")
                print(f"Center of bounding box: ({center_x}, {center_y})")
                object_detected = True
                break
        if object_detected:
            break
    
    cv2.imshow('YOLO Object Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.waitKey(1000)
cv2.destroyAllWindows()
cv2.waitKey(1)

end_time = time.time()
elapsed_time = end_time-start_time

del model

# Write the center coordinates to a file
with open('elapsed_time.txt', 'w') as f:
    f.write(f"Time: {elapsed_time}\n")

with open('center_coordinates.txt', 'w') as f:
    f.write(f"Center of bounding box: ({center_x}, {center_y})\n")

# Inside text_and_obj.py
with open('script_done.txt', 'w') as f:
    f.write('Done\n')