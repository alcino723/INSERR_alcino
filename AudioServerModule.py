import socket
import struct
import numpy as np
import time
from ModuleBase import Module
from ModuleBase import ModuleManager
from pubsub import pub
import pyaudio

# Create a PyAudio object
p = pyaudio.PyAudio()

class AudioServerRecv(Module):
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
        
        self.conn = None
        self.addr = None
        self.connected = False
        self.PORT = 10000

    # waits for client connection
    def wait_for_client(self):
        print("Waiting to receive audio")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.socket.bind(("", self.PORT))
        self.socket.listen()
        self.conn, self.addr = self.socket.accept()
        self.connected = True
        print(f"Receiving audio from PORT: {self.PORT}")
    
     # receives audio data from client and play it
    def run(self):
        if self.connected:
            # Receive audio data from the client.
            self.audio_data = self.conn.recv(self.CHUNK * self.CHANNELS * 2)
            if self.audio_data:
                # Play the sound
                self.stream.write(self.audio_data)

            else:
                # If self.conn.recv is empty string (Client Crashes)
                self.connected = False
                self.conn.close()
                self.socket.close()
                self.stream.stop_stream()
                print(f"Audio client disconnected from PORT:{self.PORT}")

        else:
            self.wait_for_client()
            self.stream.start_stream()

class AudioServerTransmit(Module):
    def __init__(self, p):
        super().__init__()

        # Initialize Audio parameters
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 44100
        self.audio_data = None

        # Open an audio stream to record sound
        self.stream = p.open(format=self.FORMAT,
                             channels=self.CHANNELS,
                             rate=self.RATE,
                             input=True,
                             output=False,
                             frames_per_buffer=self.CHUNK)
        
        self.conn = None
        self.addr = None
        self.connected = False
        self.PORT = 10001

    # waits for client connection
    def wait_for_client(self):
        print("waiting to transmit audio")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.socket.bind(("", self.PORT))
        self.socket.listen()
        self.conn, self.addr = self.socket.accept()
        self.connected = True
        print(f"Transmitting audio to PORT: {self.PORT}")
    
     # Record audio and send it to client
    def run(self):
        if self.connected:
            try:
                # Record audio data from the microphone.
                self.audio_data = self.stream.read(self.CHUNK)

                # Send the audio data to the server.
                self.conn.sendall(self.audio_data)
            except socket.error:
                self.connected = False
                self.conn.close()
                self.socket.close()
                self.stream.stop_stream()
                print(f"Audio client Disconnected from PORT:{self.PORT}")

        else:
            self.wait_for_client()
            self.stream.start_stream()
            time.sleep(0.5) # This delay is import !!! To prevent constant ALSA lib pcm.c:8545:(snd_pcm_recover) underrun occurred. WHY this is needed? IDK, is just works.

if __name__ == "__main__":
    AudioServerRecv = AudioServerRecv(p)
    AudioServerTransmit = AudioServerTransmit(p)

    AudioServerRecv.start(120)
    AudioServerTransmit.start(120)
    