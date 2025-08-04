#!/usr/bin/env python3
import sounddevice as sd
import vosk
import pyttsx3
import queue
import json
import os
import sys

# === Set model path relative to script ===
base_path = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(base_path, "vosk-model-small-en-us-0.15")

if not os.path.exists(MODEL_PATH):
    print("Please download the model from:")
    print("https://alphacephei.com/vosk/models and unzip it here as 'vosk-model-small-en-us-0.15'")
    sys.exit(1)

# === Initialize TTS ===
tts_engine = pyttsx3.init()
tts_engine.setProperty("rate", 150)

# === Initialize Vosk model ===
model = vosk.Model(MODEL_PATH)
q = queue.Queue()

def callback(indata, frames, time, status):
    if status:
        print(status, flush=True)
    q.put(bytes(indata))

def say_message(message):
    tts_engine.say(message)
    tts_engine.runAndWait()

def record_and_get_name():
    say_message("Please say your name")
    print("Please say your name:")
    with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16',
                           channels=1, callback=callback):
        rec = vosk.KaldiRecognizer(model, 16000)
        while True:
            data = q.get()
            if rec.AcceptWaveform(data):
                result = json.loads(rec.Result())
                name = result.get("text", "").strip()
                if name:
                    print(f"Captured Name: {name}")
                    return name

if __name__ == "__main__":
    name = record_and_get_name()
    say_message(f"Hello {name}")
    print(f"Final Name: {name}")
