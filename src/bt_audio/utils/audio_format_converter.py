#!/usr/bin/env python3
'''
Contains functions for converting audio data
'''
import sys
sys.path.insert(1,'/usr/local/lib/python3.5/dist-packages')

import io
import pyaudio
import wave
import logging
import base64

import logging
import os

from utils import ErrorLogger

def audio2str(raw_audio: bytes or None) -> str or None: #depricated!
    if raw_audio is None:
        return None
    return base64.b64encode(raw_audio).decode('utf-8')


def str2audio(string: str) -> bytes: #depricated
    return base64.b64decode(string.encode('utf-8'))


def raw_audio2wav(raw_audio: bytes, pyaudio_config: dict) -> bytes or None:
    '''
    pyaudio_config has the next format:
    "pyaudio":{
        "format": pyaudio.<format>,
        "channels": int,
        "rate": int,
        "frames_per_buffer": int,
        "device": int
    },
    '''
    try:
        samp_size = pyaudio.PyAudio().get_sample_size(pyaudio_config['format'])
        f = io.BytesIO()
        wave_writer = wave.Wave_write(f)
        wave_writer.setnchannels(pyaudio_config['channels'])
        wave_writer.setsampwidth(samp_size)
        wave_writer.setframerate(pyaudio_config['rate'])
        wave_writer.writeframes(raw_audio)

        f.seek(0)
        return f.read()
    except Exception as e:
        ErrorLogger(__file__, e)
        return None
