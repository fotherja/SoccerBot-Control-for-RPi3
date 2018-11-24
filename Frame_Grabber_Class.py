# This code runs a frame grabber in a thread.
#
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import time

class PiVideoStream:
    def __init__(self, resolution=(640, 480), framerate=25):
        # initialize the camera and stream
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
		
        time.sleep(0.1)
        self.stream = self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True)

        # initialize the frame and the variable used to indicate if the thread should be stopped
        self.frame = None
        self.framenum = 0
        self.stopped = False
        self.frame_timestamp = time.time()

    def start(self):
        # start the thread to read frames from the video stream
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        for f in self.stream:
            # grab the frame from the stream and clear the stream in preparation for the next frame
            frame_timestamp = time.time()
            self.frame = f.array
            self.framenum += 1
            self.rawCapture.truncate(0)

            # if the thread indicator variable is set, stop the thread and resource camera resources
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return self
	
    def Frame_Number(self):
        return self.framenum

    def read(self):
        # return the frame most recently read
        return self.frame, self.framenum, self.frame_timestamp

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
