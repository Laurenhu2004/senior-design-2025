import multiprocessing.managers
import multiprocessing.synchronize
from transformers import logging
from transformers import TFAutoModelForSequenceClassification, AutoTokenizer
from ultralytics import YOLO
import cv2
import flask
import librosa
import multiprocessing
import multiprocessing.sharedctypes
import numpy as np
import pyaudio
import tensorflow as tf
import threading
import time
import warnings
import wave
import whisper
import json

# HARDCORE debug mode
DEBUG = False

# Record audio from the microphone using PyAudio until a pause is detected
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
SILENCE_THRESHOLD = 500  # Adjust this value based on microphone sensitivity
SILENCE_CHUNKS = 50      # Number of consecutive silent chunks to detect a pause


LABELS = {0: 'cup', 1: 'bottle', 2: 'fork', 3: 'cell phone', 4: 'neutral'}
YOLO_CLASSES = {'cup': 41, 'bottle': 39, 'fork': 42, 'cell phone': 67}


def run_workflow(
    start_event: multiprocessing.synchronize.Event,
    status_message: multiprocessing.sharedctypes.Synchronized
):
    # instantiate the models at startup
    status = 'Loading models, please wait...'
    status_message.value = status
    print(status)
    start_time = time.time()
    whisper_model = whisper.load_model('turbo')
    model_save_path = 'text_classification/models/trained_v10'
    text_model = TFAutoModelForSequenceClassification.from_pretrained(
        model_save_path)
    tokenizer = AutoTokenizer.from_pretrained(model_save_path)
    yolo_model = YOLO('yolov8s.pt')
    cap = cv2.VideoCapture(0)
    end_time = time.time()
    status = f'Models loaded! Models loaded in {end_time - start_time} seconds'
    print(status)
    status_message.value = status

    # How can I receive a command from the flask server to begin the workflow?
    # TODO
    # start_event.wait()

    p = pyaudio.PyAudio()
    info = p.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')
    end_time = time.time()

    if DEBUG:
        for i in range(0, numdevices):
            if (p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
                print('Input Device id ', i, ' - ',
                      p.get_device_info_by_host_api_device_index(0, i).get('name'))
    stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE,
                    input=True, frames_per_buffer=CHUNK, input_device_index=1)
    frames = []
    status = 'Recording... Please speak into the microphone.'
    print(status)
    status_message.value = status
    silence_counter = 0
    while True:
        data = stream.read(CHUNK)
        print(data) if DEBUG else None
        frames.append(data)
        # rms = audioop.rms(data, 2)  # Calculate RMS to measure audio energy
        samples = np.frombuffer(data, dtype=np.int16).astype(np.float32)
        rms = np.sqrt(np.mean(samples**2))
        if rms < SILENCE_THRESHOLD:
            silence_counter += 1
        else:
            silence_counter = 0
        if silence_counter > SILENCE_CHUNKS:
            break
    status = 'Recording complete.'
    print(status)
    status_message.value = status
    stream.stop_stream()
    stream.close()
    p.terminate()

    if DEBUG:
        temp_audio_file = 'temp.wav'
        wf = wave.open(temp_audio_file, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()

    # Convert recorded audio bytes to a NumPy array and normalize to [-1, 1]
    raw_audio = b''.join(frames)
    audio_np = np.frombuffer(
        raw_audio, dtype=np.int16
    ).astype(np.float32) / 32768.0

    # Resample the audio from the original RATE (44100 Hz) to 16000 Hz, as expected by Whisper
    audio_np = librosa.resample(audio_np, orig_sr=RATE, target_sr=16000)
    print(
        f'Shape of the audio sample: {audio_np.shape} | audio sample: {audio_np}') if DEBUG else ModuleNotFoundError

    # Load the Whisper model and transcribe the recorded audio
    # whisper_model = whisper.load_model('turbo')

    start_time = time.time()
    result = whisper_model.transcribe(audio_np)
    end_time = time.time()
    print(f'Transcription took {end_time - start_time} seconds')
    print('Transcript:', result['text'])
    sentence = result['text'].strip()

    # model = TFAutoModelForSequenceClassification.from_pretrained(
    #     model_save_path
    # )
    # tokenizer = AutoTokenizer.from_pretrained(model_save_path)

    start_time = time.time()
    inputs = tokenizer(sentence, return_tensors='tf')
    outputs = text_model(**inputs)
    logits = outputs.logits
    predicted_class_id = tf.argmax(logits, axis=-1).numpy()[0]
    end_time = time.time()
    print(f'Text classification took {end_time - start_time} seconds')

    detected_label = LABELS.get(predicted_class_id, 'neutral')
    print(f'Predicted label: {detected_label}')

    if detected_label == 'neutral':
        print('Neutral detected. No object detection needed.')
        exit()

    if not cap.isOpened():
        print('Error: Could not open camera.')
        exit()

    object_detected = False
    target_class = YOLO_CLASSES.get(detected_label, -1)

    print(f'Looking for: {detected_label}')
    while not object_detected:
        ret, frame = cap.read()
        if not ret:
            print('Error: Failed to capture frame.')
            break

        results = yolo_model(frame, verbose=False)

        for result in results:
            for box in result.boxes:
                cls = int(box.cls.item())
                if cls == target_class:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, detected_label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    print(
                        f'{detected_label.capitalize()} detected at: ({x1}, {y1}) to ({x2}, {y2})'
                    )
                    print(f'Center of bounding box: ({center_x}, {center_y})')
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


if __name__ == '__main__':

    # set some warning levels
    warnings.filterwarnings('ignore')
    logging.set_verbosity_error()

    # root = tk.Tk()
    # root.title('Workflow Starter')

    # run the workflow in a separate thread to avoid blocking the GUI
    # start_button = tk.Button(
    #     root,
    #     text='Start Workflow',
    #     command=threading.Thread(target=run_workflow).start()
    # )
    # start_button.pack(padx=20, pady=20)

    # root.mainloop()

    start_event = multiprocessing.Event()
    status_message = multiprocessing.Manager().Value('s', 'Workflow not started')
    # multiprocessing.Process(
    #     target=run_workflow, args=(start_event, status_message, )
    # ).start()
    run_workflow(start_event, status_message)

    """
    app = flask.Flask(__name__)

    @app.route('/')
    def index():
        return '''
        <html>
        <head><title>Workflow Starter</title></head>
        <body>
            <h1>Workflow Starter</h1>
            <button onclick="startWorkflow()">Start Workflow</button>
            <div id="message" style="margin-top: 20px; font-weight: bold;"></div>
            <div id="status" style="margin-top: 20px; font-weight: bold;"></div>
            <script>
                function startWorkflow() {
                    fetch('/start_workflow', {method: 'POST'})
                    .then(response => response.json())
                    .then(data => {
                        <!-- alert(data.message); -->
                        document.getElementById('message').innerText = data.message;
                    });
                }
                // Poll the status every second
                setInterval(function() {
                    fetch('/status')
                    .then(response => response.json())
                    .then(data => {
                        document.getElementById('status').innerText = data.status;
                    });
                }, 1000);
            </script>
        </body>
        </html>
        '''

    @app.route('/start_workflow', methods=['POST'])
    def start_workflow():
        # threading.Thread(target=run_workflow).start()
        start_event.set()
        return json.dumps({'message': 'Workflow started!', 'status': status_message.value})
    
    @app.route('/status')
    def get_status():
        return json.dumps({'status': status_message.value})

    app.logger.setLevel('ERROR')
    app.run(host='0.0.0.0', port=8080, debug=False)"
    """
    
    # input('Press Enter to start the workflow...')
    # start_event.set()

    # threading.Thread(target=run_workflow).start()
    # run_workflow()"

    # Write the center coordinates to a file
    # with open('elapsed_time.txt', 'w') as f:
    #     f.write(f"Time: {elapsed_time}\n")

    # with open('center_coordinates.txt', 'w') as f:
    #     f.write(f"Center of bounding box: ({center_x}, {center_y})\n")

    # # Inside text_and_obj.py
    # with open('script_done.txt', 'w') as f:
    #     f.write('Done\n')