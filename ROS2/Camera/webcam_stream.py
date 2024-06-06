import cv2
import time
import threading
from flask import Response, Flask

global video_frame
video_frame = None

global thread_lock
thread_lock = threading.Lock()

GTREAMER_PIPELINE = 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=3280,height=2464,format=(string)NV12,framerate=21/1 ! nvvidconv flip-method=0 ! video/x-raw,width=960,height=616,format=(string)BGRx ! videoconvert ! video/x-raw,format(string)BGR ! appsink wait-on-eos=false max-buffers=1 drop=True'

app = Flask(__name__)

def captureFrame():
	global video_frame, frame_lock
	video_capture = cv2.VideoCapture(0)
	
	while True and video_capture.isOpened():
		return_key,frame = video_capture.read()
		if not return_key:
			break
		with thread_lock:
			video_frame = frame.copy()
		key = cv2.waitKey(30) & 0xff
		if key == 27:
			break
	video_capture.release()

def encodedFrame():
	global thread_lock
	while True:
		with thread_lock:
			global video_frame
			if video_frame is None:
				continue
			return_key, encoded_image = cv2.imencode(".jpg", video_frame)
			if not return_key:
				continue
		yield(b'--frmae\r\n'b'Content-Type:image/jpeg\r\n\r\n'+bytearray(encoded_image)+b'\r\n')

@app.route("/")
def streamFrames():
	return Response(encodedFrame(), mimetype="multipart/x-mixed-replace; boundary=frmae")

if __name__ == '__main__':
	process_thread = threading.Thread(target=captureFrame)
	process_thread.daemon = True
	process_thread.start()

app.run("10.42.0.1", port="8000")
