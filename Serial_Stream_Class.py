# Code to output over the serial port using threading. 2 bytes, the Yaw_Error and the X_Error
#
import serial
from threading import Thread
import queue


class SerialStream:
    def __init__(self, baud=115200):
        # initialise the serial port
        self.ser = serial.Serial(
                       #port='/dev/ttyUSB0',
                       port='/dev/serial0',
                       baudrate = baud,
                       parity=serial.PARITY_NONE,
                       stopbits=serial.STOPBITS_ONE,
                       bytesize=serial.EIGHTBITS,
                       timeout=1
                   )

        self.q = queue.Queue()        
        self.stopped = False

    def start(self):
        # start the thread to output over the serial port
        Thread(target=self.Serial_Stream, args=()).start()
        return self
        
    def Serial_Stream(self):
        while(True):
            self.x_ser, self.y_ser = self.q.get()

            self.ser.write(bytes([255]))
            self.ser.write(bytes([self.x_ser]))
            self.ser.write(bytes([self.y_ser]))
            self.ser.flush()

            if self.stopped:
                self.ser.close()
                return self
        return self

    def write(self, xy=(127, 127)):
        # Add a new data pair to the queue 
        self.q.put_nowait(xy)

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

