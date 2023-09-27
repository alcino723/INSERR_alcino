import socket
import struct
import time
from ModuleBase import Module
from ModuleBase import ModuleManager
from pubsub import pub
import pyaudio

# Create a PyAudio object
p = pyaudio.PyAudio()

class AudioClientRecv(Module):
    def __init__(self, p):
        super().__init__()

        # Initialize Audio parameters
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 44100
        self.audio_data = None

        # Open an audio stream to play sound
        self.stream = p.open(format=self.FORMAT,
                             channels=self.CHANNELS,
                             rate=self.RATE,
                             input=False,
                             output=True)

        # Connect to the server
        self.HOST = "192.168.1.27" # Alcino PC
        # self.HOST = "169.254.196.165"  # Isaac's Laptop
        # self.HOST = '169.254.104.53' # Silver Laptop 
        self.PORT = 10001
        self.connect_to_server()

    def connect_to_server(self):
        while True:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.HOST, self.PORT))

                print(f"Receiving audio from audio server: {self.HOST}, PORT: {self.PORT}")
                break
            except ConnectionRefusedError:
                print('No connection for receving audio. Retrying in 1 seconds...')
                time.sleep(1)

    def run(self):
        try:
            # Receive audio data from the server.
            self.audio_data = self.socket.recv(self.CHUNK * self.CHANNELS * 2)

            # Play the received audio data.
            self.stream.write(self.audio_data)

        except (BrokenPipeError, ConnectionResetError):
            print('Receive audio connection lost. Reconnecting...')
            self.socket.close()
            self.stream.stop_stream()
            self.connect_to_server()
            self.stream.start_stream()

class AudioClientTransmit(Module):
    def __init__(self, p):
        super().__init__()

        # Initialize Audio parameters
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 44100
        self.audio_data = None

        # Create a PyAudio object
        self.p = pyaudio.PyAudio()

        # Open an audio stream to record sound
        self.stream = p.open(format=self.FORMAT,
                             channels=self.CHANNELS,
                             rate=self.RATE,
                             input=True,
                             output=False,
                             frames_per_buffer=self.CHUNK)

        # Connect to the server
        self.HOST = "192.168.1.27" # Alcino PC
        # self.HOST = "169.254.196.165"  # Isaac's Laptop
        # self.HOST = '169.254.104.53' # Silver Laptop 
        self.PORT = 10000
        self.connect_to_server()

    def connect_to_server(self):
        while True:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.HOST, self.PORT))

                print(f"Transmitting audio to audio server: {self.HOST}, PORT: {self.PORT}")
                break
            except ConnectionRefusedError:
                print('No connection for audio transmission. Retrying in 1 seconds...')
                time.sleep(1)

    def run(self):
        
        try:
            # Record audio data from the microphone.
            self.audio_data = self.stream.read(self.CHUNK)

            # Send the audio data to the server.
            self.socket.sendall(self.audio_data)

        except (BrokenPipeError, ConnectionResetError):
            print('Audio tranmsmission connection lost. Reconnecting...')
            self.socket.close()
            self.stream.stop_stream()
            self.connect_to_server()
            self.stream.start_stream()

if __name__ == "__main__":
    AudioClientRecv = AudioClientRecv(p)
    AudioClientTransmit = AudioClientTransmit(p)

    AudioClientRecv.start(120)
    AudioClientTransmit.start(120)
