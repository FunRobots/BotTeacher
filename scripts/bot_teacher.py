#!/usr/bin/env python3
import rospy

from bt_audio.recorder import Recorder
from bt_audio.recognizer import SpeechRecognizer
from bt_audio.synthesizer import Talker
from bt_audio.player import Player
import io
from bt_apiai_service.bot_client import APIAIBot
import json
import time
from bt_audio.utils import audio_format_converter
import os
from pathlib import Path
from bt_utils import ErrorLogger


class SchemeProcessor:

    def __init__(self, yandex_voice_key, apiai_bot_client_key, pyaudio_config, scheme_file):

        #bad speech recognition client phrase
        self.BAD_SPEECH_RECOGNITION = 'Повторите, пожалуйста, еще раз. Плохо слышно'
        self.STAT_FOLDER = 'stat'
        if os.path.exists(self.STAT_FOLDER) is False:
            os.mkdir(self.STAT_FOLDER)
        self.log = open('stat/' + time.ctime() + '.log', 'w')

        self.yandex_voice_key = yandex_voice_key
        self.apiai_bot_client_key = apiai_bot_client_key
        self.pyaudio_config = pyaudio_config

        self.scheme = self._read_scheme_file(scheme_file)
        self.current_intent = self.scheme.get('start')
        #read config
        yandex_voice_key = rospy.get_param('yandex_voice_key')
        apiai_bot_client_key = rospy.get_param('apiai_bot_client_key')
        pyaudio_config = json.loads(rospy.get_param('pyaudio'))

        #create objects
        self.rec = Recorder(pyaudio_config, min_rms=1000)
        self.speech_recognizer = SpeechRecognizer(yandex_voice_key=yandex_voice_key)
        self.talker = Talker(yandex_voice_key=yandex_voice_key)
        self.player = Player()
        self.bot = APIAIBot(client_key=apiai_bot_client_key)

        self.scheme_functions = {
            'goto_next': self._goto_next,
            'say_phrase': self._say_phrase,
            'wait_before_transition': self._wait_before_transition
        }

    def _recognize_speech(self) -> str:
        if rospy.has_param('min_rms'):
            rec.set_min_rms(min_rms = rospy.get_param('min_rms'))

        speech_audio = rec.listen_audio(timeout=91)
        if speech_audio is not None:
            speech_text = speech_recognizer.recognize_speech(audio_format_converter.raw_audio2wav(speech_audio, pyaudio_config))
            return speech_text
        return str()

    def _say_text(self, text: str) -> None:
        speech = talker.text_to_speech(text)
        wav_source = io.BytesIO(speech)
        wav_source.seek(0)
        player.play_audio(wav_source)

    def _read_scheme_file(self, scheme_file: str) -> dict:
        pass

    def _goto_next(self, intent_name: str) -> None:
        pass

    def _say_phrase(self, phrase_to_say: str) -> None:
        pass

    def _wait_before_transition(self, variants_dict: dict) -> None:
        pass

    def process(self):

        self.log.close()


def main():
    pass


if __name__ == '__main__':
    rospy.init_node('bot_teacher')
    main()
