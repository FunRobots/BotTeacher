'''
playing audio files
'''

import io
import pyaudio
import wave
import http

from utils import ErrorLogger

class Player:
    '''
    Simple audio player
    [Now supports just wave audio data]
    '''

    def play_audio(self, audio_data_src: http.client.HTTPResponse or io.BytesIO, format: str='wav') -> None:
        '''
        Plays audio_data with format=format
        Args:
            audio_data: binary audio source
            format: audio format
        '''
        try:
            if format == 'wav':
                #define stream chunk
                chunk = 1024

                #open a wav format music
                f = wave.Wave_read(audio_data_src)
                #instantiate PyAudio
                p = pyaudio.PyAudio()
                #open stream
                stream = p.open(format = p.get_format_from_width(f.getsampwidth()),
                                channels = f.getnchannels(),
                                rate = f.getframerate(),
                                output = True)
                #read data
                data = f.readframes(chunk)

                #play stream
                while data:
                    stream.write(data)
                    data = f.readframes(chunk)

                #stop stream
                stream.stop_stream()
                stream.close()

                #close PyAudio
                p.terminate()
        except Exception as e:
            ErrorLogger(__file__, e)
