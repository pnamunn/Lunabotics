import cv2
import numpy as np
import pyrealsense2 as rs
from flask import Flask, Response
import time

app = Flask(__name__)

def gen_frames1():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # Convert color frame to OpenCV format
            color_image = cv2.cvtColor(np.asanyarray(color_frame.get_data()), cv2.COLOR_BGR2RGB)
            color_image = color_image[:,:,::-1]            

            # Encode image as JPEG
            ret, color_buffer = cv2.imencode('.jpg', color_image)

            # Yield the resulting image as a byte stream
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + color_buffer.tobytes() + b'\r\n')
    finally:
        pipeline.stop()

def gen_frames2():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue

            # Convert depth frame to numpy array
            depth_image = np.asanyarray(depth_frame.get_data())

            # Convert numpy array to grayscale image
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
            )

            # Encode image as JPEG
            ret, buffer = cv2.imencode('.jpg', depth_colormap)

            # Yield the resulting image as a byte stream
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
    finally:
        pipeline.stop()

def gen_frames3():
    cap = cv2.VideoCapture(0)       #the number is to choose the webcam
    fpsLimit = 1.0/15 # denominator = fps cap
    startTime = time.time()    

    while True:
        nowTime = time.time()
        if (float(nowTime - startTime)) > fpsLimit:     #wait for enough time to pass
            ret, frame = cap.read()
            if not ret:
                break
            gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)      #convert to gray
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 15]      #number = encode quality
            ret, buffer = cv2.imencode('.jpg', frame, encode_param)           #encode the thing

            frame = buffer.tobytes()
            yield (b'--frame\r\n'                                       #idk how this line really works but it outputs the frame we just got
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # concat frame one by one and show result
            startTime = time.time()
            
        



@app.route('/depth')        #add the string to end of url to use
def depth():
    return Response(gen_frames2(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/color')
def color():
    return Response(gen_frames1(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/webcam')     
def webcam():
    return Response(gen_frames3(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='10.130.17.64', port=8000, threaded=True)     #on web browser connect to ip:port
