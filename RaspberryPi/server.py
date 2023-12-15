#!/usr/bin/env python
from flask import Flask, render_template, Response
from camera import Camera
import Adafruit_DHT
import datetime
from firebase import firebase
import threading
import time


app = Flask(__name__)


sensor = Adafruit_DHT.DHT11
pin = 4

thread = None

firebase = firebase.FirebaseApplication('https://cctv-project-609d3-default-rtdb.firebaseio.com/', None)

def read_sensor():
    humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)
    return humidity, temperature
    
def background_thread():
	count = 0
	while True:
		humidity, temperature = read_sensor()
		firebase.patch('/sensor', {'temp': temperature, 'hum': humidity})
		time.sleep(30)

@app.route('/')
def index():
    global thread
    if thread is None:
        thread = threading.Thread(target=background_thread)
        thread.start()
    return render_template('index.html')

def gen(camera):
   while True:
       frame = camera.get_frame()
       yield (b'--frame\r\n'
              b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
   return Response(gen(Camera()),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
   app.run(host='0.0.0.0', debug=True, threaded=True)
