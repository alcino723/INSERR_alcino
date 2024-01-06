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
import pyaudio
import USBCameraServerModulev2 as CSMv2
import USBCameraServerModule as CSM
import AudioServerModule as ASM

def start_cam_server_NOAI():
    # Start Camera Server with no object detection
    USBCameraHandler = CSM.USBCameraHandler()
    USBCameraDisplay = CSM.USBCameraDisplay()

    USBCameraHandler.start(120)
    USBCameraDisplay.start(60)

def start_cam_server_AI():

    # Start Camera Server with Object detection
    USBCameraHandler = CSMv2.USBCameraHandler()
    ObjectDetection = CSMv2.ObjectDetection()
    USBCameraDisplay = CSMv2.USBCameraDisplay()
    
    USBCameraHandler.start(120)
    ObjectDetection.start(120)
    USBCameraDisplay.start(60)

if __name__ == "__main__":
    start_cam_server_AI()

    # Start Audio Server
    AudioServerRecv = ASM.AudioServerRecv(ASM.p)
    AudioServerTransmit = ASM.AudioServerTransmit(ASM.p)

    AudioServerRecv.start(120)
    AudioServerTransmit.start(120)
    

    

    

    

