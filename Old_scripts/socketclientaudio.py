import cv2
import socket
import struct
import time
import pyaudio
import numpy as np
import threading

# Server Parameters
IP = '192.168.1.27'
V_PORT = 5050
AR_PORT = 9999
AS_PORT= 9998

# Audio parameters
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100

def connect_to_server(port):
    # Create a socket and connect to the server
    client_socket = socket.socket()
    while True:
        try:
            client_socket.connect((IP, port))
            print(f'Connection established Port:{port}')
            time.sleep(1)
            break
        except ConnectionRefusedError:
            print(f'No connection established to PORT:{port}. Retrying in 1 seconds...')
            time.sleep(1)
    return client_socket

def camera(cam_num, cam_format, v_width, v_height, v_fps):
    # Start the camera
    cam = cv2.VideoCapture(cam_num, cv2.CAP_V4L2)
    # Set the video format
    cam.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    fourcc = cv2.VideoWriter_fourcc(*cam_format)
    cam.set(cv2.CAP_PROP_FOURCC, fourcc)
    cam.set(cv2.CAP_PROP_HW_ACCELERATION, cv2.VIDEO_ACCELERATION_ANY)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, v_width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, v_height)
    cam.set(cv2.CAP_PROP_FPS, v_fps)
    return cam

def recv_audio(PORT, stream):
    try:
        client_socket_audio_recv = connect_to_server(PORT)
        while True:
            try:
                # Receive audio data from the server.
                audio_data = client_socket_audio_recv.recv(CHUNK * CHANNELS * 2)

                # Play the received audio data.
                stream.write(audio_data)
                
            except (BrokenPipeError, ConnectionResetError):
                print(f'Connection lost to PORT:{PORT}. Reconnecting ...')
                client_socket_audio_recv.close()
                stream.stop_stream()
                client_socket_audio_recv = connect_to_server(PORT)
                stream.start_stream()
                
    finally:
        stream.close()
        p.terminate()
        client_socket_audio_recv.close()
        
def send_audio(PORT, stream):
    try:
        client_socket_audio_send = connect_to_server(PORT)
        while True:
            try:
                 #Record Audio data from microphone
                audio_data = stream.read(CHUNK)
                
                #Send the audio data to the client
                client_socket_audio_send.sendall(audio_data)
            except (BrokenPipeError, ConnectionResetError):
                print(f'Connection lost to PORT:{PORT}. Reconnecting ...')
                client_socket_audio_send.close()
                stream.stop_stream()
                client_socket_audio_send = connect_to_server(PORT)
                stream.start_stream()
                
    finally:
        stream.close()
        p.terminate()
        client_socket_audio_send.close()    

# Start camera
cam0 = camera(0,'MJPG', 1920, 1080, 30)

# Create a PyAudio object
p = pyaudio.PyAudio()

# Open an audio stream
out_stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                output=True)
                    
# Open an input audio stream
in_stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK)

# Connect to the server for video data
client_socket = connect_to_server(V_PORT)

# Start the audio recv thread
recv_audio_thread = threading.Thread(target=recv_audio, args=(AR_PORT, out_stream))
recv_audio_thread.start()

# Start the audio send thread
send_audio_thread = threading.Thread(target=send_audio, args=(AS_PORT, in_stream))
send_audio_thread.start()

try:
    while True:
        # Read the frame
        ret, image = cam0.read()
        # Convert the frame to a byte array
        frame_data = image.tobytes()

        try:
            # Send the frame data through the socket
            client_socket.sendall(struct.pack('<L', len(frame_data)) + frame_data)

        except (BrokenPipeError, ConnectionResetError):
            print(f'Connection lost to PORT:{V_PORT}. Reconnecting ...')
            client_socket.close()
            client_socket = connect_to_server(V_PORT)

finally:  # Release everything if job is finished
    cam0.release()
    client_socket.close()
    recv_audio_thread.join()
    send_audio_thread.join()

