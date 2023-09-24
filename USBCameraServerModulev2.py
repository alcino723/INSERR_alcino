import socket
import struct
import numpy as np
import cv2 as cv # pip install opencv-python
from ModuleBase import Module
from ModuleBase import ModuleManager
from pubsub import pub
import torch # pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
from ultralytics import YOLO # pip install ultralytics
import time

class USBCameraHandler(Module):
    def __init__(self):
        super().__init__()

        self.conn = None
        self.addr = None
        self.connected = False
        self.PORT = 8080

    # waits for client connection
    def wait_for_client(self):
        print("waiting for USB Client to connect")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.socket.bind(("", self.PORT))
        self.socket.listen()
        self.conn, self.addr = self.socket.accept()
        self.connected = True
        print(f"USB Connected from PORT: {self.PORT}")
    
     # receives camera frame and publish it out
    def run(self):
        while True:
            if self.connected:
                try:
                    # Receive the frame size from the client
                    frame_size_data = self.conn.recv(struct.calcsize('<L'))
                    if frame_size_data:
                        frame_size = struct.unpack('<L', frame_size_data)[0]

                        # Receive the frame data from the client
                        frame_data = b''
                        while len(frame_data) < frame_size:
                            data = self.conn.recv(frame_size - len(frame_data))
                            if not data:
                                break
                            frame_data += data

                        # Decode the MJPEG data and convert it to a BGR image
                        frame = cv.imdecode(np.frombuffer(frame_data, dtype=np.uint8), cv.IMREAD_COLOR)

                        # Publish Decode Frame
                        pub.sendMessage("usbcam", message = {"data": frame})

                    else:
                        # If self.conn.recv is empty string (Client Crashes)
                        self.connected = False
                        self.conn.close()
                        self.socket.close()
                        print(f"USB Disconnected from PORT:{self.PORT}")
                        cv.destroyAllWindows()

                except socket.error:
                    self.connected = False
                    self.conn.close()
                    self.socket.close()
                    print(f"USB Disconnected from PORT:{self.PORT}")
                    
            else:
                self.wait_for_client()

class ObjectDetection(Module):
    def __init__ (self):
        super().__init__()
        # Initialize AI model
        self.model = YOLO('yolov8s.pt')
        # Detection Model Parameters
        self.thres_hold = 0.6 
        self.classes = 0
        self.start_time = 0
        self.end_time = 0
        self.fps = 0
        pub.subscribe(self.message_listener, "usbcam")

    def message_listener(self, message):
        # Check if a CUDA-capable GPU is available, if no CUDA-capable GPU skip object detection
        self.start_time = time.time()
        if torch.cuda.is_available():
            # Detect objects in the image using Yolov8 on the GPU
            results = self.model(message["data"], conf=self.thres_hold, device=0, half=True, classes=self.classes, verbose=False)[0]
            
            # Draw bounding boxes around detected objects on the BGR image
            for box in results.boxes.xyxy:
                # Get the bounding box coordinates
                x1, y1, x2, y2 = box

                # Draw bounding boxes around detected objects on the BGR image
                cv.rectangle(message["data"], (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 5)

        # Calculate FPS
        self.end_time = time.time()
        tt = self.end_time - self.start_time
        self.fps = 0.9*self.fps + 0.1*(1/tt)
        cv.putText(message["data"], "FPS:" + str(int(self.fps)),(30,60),cv.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 3)

        # Publish frame
        pub.sendMessage("image", message = {"data": message["data"]})

class USBCameraDisplay(Module):
    def __init__ (self):
        super().__init__()
        
        pub.subscribe(self.message_listener, "image")
    
    def message_listener(self, message):
        cv.imshow('frame', message["data"])
        k = cv.waitKey(1)

if __name__ == "__main__":
    USBCameraHandler = USBCameraHandler()
    ObjectDetection = ObjectDetection()
    USBCameraDisplay = USBCameraDisplay()

    USBCameraHandler.start(120)
    ObjectDetection.start(120)
    USBCameraDisplay.start(60)
