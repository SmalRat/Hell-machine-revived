from flask import Flask, Response
from scan import create_frames
import cv2

vid = cv2.VideoCapture(0) 

app = Flask("hello")

@app.route("/")
def index():
    return Response(create_frames(vid),mimetype='multipart/x-mixed-replace; boundary=frame')

app.run(host="0.0.0.0")
vid.release() 
cv2.destroyAllWindows() 
