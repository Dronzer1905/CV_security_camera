#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import cv2
import time
import numpy as np
from flask import Flask, Response
import threading

app = Flask(__name__)
frame_data = None

def video_callback(msg):
    global frame_data
    frame_data = np.frombuffer(msg.data, dtype=np.uint8)
    frame_data = cv2.imdecode(frame_data, cv2.IMREAD_COLOR)

# Initialize ROS before starting the thread
def ros_listener():
    rospy.Subscriber('/video_frames', CompressedImage, video_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('video_stream_subscriber', anonymous=True)
        ros_thread = threading.Thread(target=ros_listener)
        ros_thread.daemon = True
        ros_thread.start()
    except rospy.ROSInterruptException:
        pass

def generate_frames():
    global frame_data
    while True:
        if frame_data is not None:
            _, buffer = cv2.imencode('.jpg', frame_data)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return "<html><body><h1>ROS Video Stream</h1><img src='/video_feed' style= 'display: block; margin-left:auto; margin-right: auto; width:50%'></body></html>"

# Run the server on a different port (5050) to avoid conflicts
app.run(host='0.0.0.0', port=8080, debug=False)

