#!/usr/bin/env python3
'''
record audio
'''

import pyaudio
import time
import audioop

from utils import ErrorLogger


class Recorder:

    def __init__(self, pyaudio_config: dict, min_rms=100):
        '''
        Constructor
        Args:
            pyaudio_config: pyaudio configuration in format
                dictionary = {
            		'format': audio_format,
            		'channels': channels_number,
            		'rate': rate,
            		'frames_per_buffer': frames_per_buffer
                    'device': input_device_number
            	}

            min_rms: minimal sound rms value
        '''

        try:
            self.set_min_rms(min_rms=min_rms)
            #set up pyaudio configuration and start audio stream
            self.format = pyaudio_config['format']
            self.channels = pyaudio_config['channels']
            self.rate = pyaudio_config['rate']
            self.chunk_size = pyaudio_config['frames_per_buffer']
            self.input_device = pyaudio_config['input_device']

            self.audio = pyaudio.PyAudio()
            self.stream = self.audio.open(format=self.format, channels=self.channels, rate=self.rate, input=True, input_device_index=self.input_device, frames_per_buffer=self.chunk_size)
            self.stream.start_stream()
        except Exception as e:
            ErrorLogger(__file__, e)

    def set_min_rms(self, min_rms):
        '''
        Set minimal sound rms value
        Args:
            min_rms: minimal sound rms value
        '''
        self.min_rms = min_rms

    def record_audio(self, last_chunk):
        print('rec')
        '''Records speech until silence
        Args:
            last_chunk: last chunk before record starting
            stream: audio stream
            min_rms: minimal rms value that is not silence
        Returns:
            buf: contains recorded audio data, bytes type
        '''
        try:
            buf = last_chunk
            silence_start = time.time()
            while (time.time() - silence_start) < 1.2:
                chunk = self.stream.read(self.chunk_size, exception_on_overflow=False)
                if audioop.rms(chunk, 2) >= self.min_rms / 3 :
                    buf += chunk
                if audioop.rms(chunk, 2) >= self.min_rms:
                    silence_start = time.time()
            return buf
        except Exception as e:
            ErrorLogger(__file__, e)
            return None

    def listen_audio(self, timeout=1000000) -> bytes or None:
        '''
        Listen sound until silence
        '''
        try:
            start = time.time()
            while time.time() - start < timeout:
                chunk = self.stream.read(self.chunk_size, exception_on_overflow=False)
                #if sound detected record raw data until silence
                if audioop.rms(chunk, 2) >=  self.min_rms:
                    buf = self.record_audio(last_chunk=chunk)
                    return buf
        except Exception as e:
            ErrorLogger(__file__, e)
