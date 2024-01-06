import cv2 as cv
import struct
import socket
import time 
from ModuleBase import Module
from pubsub import pub
import pyaudio
import USBCameraClientModule as CCM
import AudioClientModule as ACM

if __name__ == "__main__":
    # Start Camera Client
    USBCamera = CCM.USBCamera(0, 'MJPG', 1920, 1080, 30)
    USBCamera.start(60)

    # Start Audio Client
    AudioClientRecv = ACM.AudioClientRecv(p)
    AudioClientTransmit = ACM.AudioClientTransmit(p)

    AudioClientRecv.start(120)
    AudioClientTransmit.start(120)
