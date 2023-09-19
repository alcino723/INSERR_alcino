import socket
import struct
import threading
import queue
import numpy as np
import time
import cv2 # pip install opencv-python
import torch # pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
from ultralytics import YOLO # pip install ultralytics
import pyaudio

IP = '192.168.1.27'
V_PORT = 5050
AS_PORT = 9999
AR_PORT = 9998

# Detection Model Parameters
thres_hold = 0.5 
classes = 0

# Audio parameters
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100

#Initialize fps counter
fps = 0 

def video(client_socket, frame_queue):
    while True:
        try:
            # Receive the frame size from the client
            frame_size_data = client_socket.recv(struct.calcsize('<L'))
            if not frame_size_data:
                break
            frame_size = struct.unpack('<L', frame_size_data)[0]

            # Receive the frame data from the client
            frame_data = b''
            while len(frame_data) < frame_size:
                data = client_socket.recv(frame_size - len(frame_data))
                if not data:
                    break
                frame_data += data

            # Decode the MJPEG data and convert it to a BGR image
            frame = cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), cv2.IMREAD_COLOR)

            # Add the frame to the queue
            frame_queue.put(frame)

        except Exception as e:
            print(f'Error: {e}')
            break

def audio_send(client_socket, in_stream):
    while True:
        try:
            # Record audio data from the microphone.
            audio_data = in_stream.read(CHUNK)

            # Send the audio data to the client.
            client_socket.sendall(audio_data)

        except Exception:
            break

def audio_recv(client_socket, out_stream):
    while True:
        try:
            # Receive audio data from the server.
            audio_data = client_socket.recv(CHUNK * CHANNELS * 2)

            # Play the received audio data.
            out_stream.write(audio_data)

        except Exception:
            break

if __name__ == '__main__':
     # Load the pre-trained Yolov8 model
    model = YOLO('yolov8m.pt')
    
    # Create a socket and bind it to a video port
    server_socket_video = socket.socket()
    server_socket_video.bind((IP, V_PORT))
    server_socket_video.listen(1)

    # Create a socket and bind it to a send_audio port
    server_socket_audio_send = socket.socket()
    server_socket_audio_send.bind((IP, AS_PORT))
    server_socket_audio_send.listen(1)

    # Create a socket and bind it to a send_audio port
    server_socket_audio_recv = socket.socket()
    server_socket_audio_recv.bind((IP, AR_PORT))
    server_socket_audio_recv.listen(1)

    # Create a PyAudio object
    p = pyaudio.PyAudio()

    # Open a stream to record audio
    in_stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    output=False,
                    frames_per_buffer=CHUNK)
    
    # Open an output audio stream
    out_stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=False,
                    output=True)
    try:
        while True:
            # Wait for a video client to connect
            client_socket_video, client_address_video = server_socket_video.accept()

            # Wait for a audio send client to connect
            client_socket_audio_send, client_address_audio_send = server_socket_audio_send.accept()

            # Wait for a audio receive client to connect
            client_socket_audio_recv, client_address_audio_recv = server_socket_audio_recv.accept()
            try:
                # Create a queue to hold received frames
                frame_queue = queue.Queue()

                # Start a thread to receive and decode MJPEG frames from the client
                video_thread = threading.Thread(target=video, args=(client_socket_video, frame_queue))
                video_thread.start()

                # Start a thread to handle sending audio 
                send_audio_thread = threading.Thread(target=audio_send, args=(client_socket_audio_send, in_stream))
                send_audio_thread.start()

                # Start a thread to handle receiving audio
                recv_audio_thread = threading.Thread(target=audio_recv, args=(client_socket_audio_recv, out_stream))
                recv_audio_thread.start()

                while True:
                    try:
                        # Get the next frame from the queue
                        frame = frame_queue.get()
                        start_time = time.time()

                        # Check if a CUDA-capable GPU is available, if no CUDA-capable GPU skip object detection
                        if torch.cuda.is_available():

                            # Detect objects in the image using Yolov8 on the GPU
                            results = model(frame, conf=thres_hold, device=0, half=True, classes=classes, verbose=False)[0]  
                            
                            # Draw bounding boxes around detected objects on the BGR image
                            for box in results.boxes.xyxy:
                                # Get the bounding box coordinates
                                x1, y1, x2, y2 = box

                                # Draw bounding boxes around detected objects on the BGR image
                                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 5)

                        # Calculate FPS
                        end_time = time.time()
                        tt = end_time - start_time
                        fps = 0.9*fps + 0.1*(1/tt)
                        cv2.putText(frame, "FPS:" + str(int(fps)),(30,60),cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 3)

                        # Show the frame
                        cv2.imshow('Frame', frame)

                        k = cv2.waitKey(1)
                        if k == 27:
                            break
                        
                    except Exception as e:
                        print(f'Error: {e}')
                        break

                # Stop the threads and wait for it to finish
                video_thread.join()
                send_audio_thread.join()
                recv_audio_thread.join()
                
            finally:
                # Close the client socket and release any other resources when exiting.
                client_socket_video.close()
                client_socket_audio_send.close()
                client_socket_audio_recv.close()

    #Release everything if job is finished      
    finally:
        server_socket_video.close()
        server_socket_audio_send.close()
        server_socket_audio_recv.close()
        in_stream.stop_stream()
        in_stream.close()
        out_stream.stop_stream()
        out_stream.close()
        p.terminate()
        cv2.destroyAllWindows()
