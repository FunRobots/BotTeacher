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
from typing import List


class SchemeProcessor:

    def __init__(self, yandex_voice_key, apiai_bot_client_key, pyaudio_config, scheme_file):

        #bad speech recognition client phrase
        self.BAD_SPEECH_RECOGNITION = 'Повторите, пожалуйста, еще раз. Плохо слышно'

        self.scheme_functions = {
            'goto_next': self._goto_next,
            'say_phrase': self._say_phrase,
            'wait_before_transition': self._wait_before_transition
        }

        self.yandex_voice_key = yandex_voice_key
        self.apiai_bot_client_key = apiai_bot_client_key
        self.pyaudio_config = pyaudio_config

        self.scheme = self._read_scheme_file(scheme_file)
        self.current_intent_name = self.scheme.get('start')

        #create objects
        self.rec = Recorder(self.pyaudio_config, min_rms=1000)
        self.speech_recognizer = SpeechRecognizer(yandex_voice_key=self.yandex_voice_key)
        self.talker = Talker(yandex_voice_key=self.yandex_voice_key)
        self.player = Player()
        self.bot = APIAIBot(client_key=self.apiai_bot_client_key)

        self.bot_answer_text = str()
        self.pause = 0
        self.do_bot_request_by_consultant_speech = True
        #self.current_intent_changed = False

    def _recognize_speech(self) -> str:
        if rospy.has_param('min_rms'):
            self.rec.set_min_rms(min_rms = rospy.get_param('min_rms'))

        speech_audio = self.rec.listen_audio(timeout=91)
        if speech_audio is not None:
            speech_text = self.speech_recognizer.recognize_speech(audio_format_converter.raw_audio2wav(speech_audio, self.pyaudio_config))
            return speech_text
        return str()

    def _say_text(self, text: str) -> None:
        speech = self.talker.text_to_speech(text)
        wav_source = io.BytesIO(speech)
        wav_source.seek(0)
        self.player.play_audio(wav_source)

    def _read_scheme_file(self, scheme_file: str) -> dict:
        scheme_file = Path(__file__).resolve().parents[1].as_posix() + '/run/' + rospy.get_param('scheme_file')
        return json.load(open(scheme_file,'r'))

    def _goto_next(self, intent_name: str) -> None:
        self.current_intent_name = intent_name
        #self.current_intent_changed = True
        return None

    def _say_phrase(self, phrase_to_say: str) -> List[str] or None:
        self.do_bot_request_by_consultant_speech = False

        bot_answer = self.bot.request(phrase_to_say)
        if isinstance(bot_answer, dict):
            return [bot_answer.get('text')]
        else:
            return None

    def _wait_before_transition(self, variants_dict: dict) -> List[str] or None:
        self.do_bot_request_by_consultant_speech = False

        wait_keys = [int(i) for i in variants_dict.keys()]
        wait_keys.sort()

        print('wait_keys', wait_keys)
        print('pause', self.pause)

        case_key_index = 0
        while case_key_index < len(wait_keys) and self.pause > wait_keys[case_key_index]:
            print('|')
            case_key_index += 1

        case = variants_dict[str(wait_keys[case_key_index])]
        print('case', case)
        self.current_intent_name = case.get('goto_next')

        phrase_to_say = case.get('say_phrase')
        if phrase_to_say is not None:
            bot_answer = self.bot.request(phrase_to_say)
            if isinstance(bot_answer, dict):
                return bot_answer.get('text')
            else:
                return None

        return case.get('return')

    def process(self):

        self.STAT_FOLDER = 'stat'
        if os.path.exists(self.STAT_FOLDER) is False:
            os.mkdir(self.STAT_FOLDER)
        self.log = open('stat/' + time.ctime() + '.log', 'w')

        while self.current_intent_name is not None:
            print('intent:', self.current_intent_name)
            #listen to Consultant and notify time
            speech_text = str()
            start = time.time()
            while time.time() - start < 100:
                print('listen...\n')
                speech_text = self._recognize_speech()
                if speech_text is not None and len(speech_text) > 0:
                    break

            self.pause = time.time() - start
            print('Consultant > ', speech_text)
            bot_answer = self.bot.request(speech_text)
            if isinstance(bot_answer, dict) and bot_answer.get('intent_name') == self.current_intent_name:
                current_intent = self.scheme['intents'].get(self.current_intent_name)
                if isinstance(current_intent, dict):
                    variants_dict = current_intent.get('wait_before_transition')
                else:
                    variants_dict = None
                    current_intent = dict()

                if variants_dict is not None:
                    self.log.write('\n\n [{0}] \n Intent: {1} \n Consultant > {2}'.format(time.ctime(), self.current_intent_name, speech_text))
                    bot_answer_text = self._wait_before_transition(variants_dict=variants_dict)
                    print('Client > ', bot_answer_text)
                    self._say_text(bot_answer_text)
                    self.log.write('\n Client > {0}'.format(bot_answer_text))
                else:
                    print('Client > ', bot_answer.get('text'))
                    self._say_text(bot_answer.get('text'))
                    self.log.write('[{0}] \n Intent: {1} \n Consultant > {2} \n Client > {3}'.format(time.ctime(), self.current_intent_name, speech_text, bot_answer['text']))
                    self.current_intent_name = current_intent.get('goto_next')

            else:
                print(self.BAD_SPEECH_RECOGNITION)
                self._say_text(self.BAD_SPEECH_RECOGNITION)
                continue

        self.log.close()

        print('Повторить?')
        self._say_text('Повторить?')
        speech_text = self._recognize_speech()
        if speech_text is not None and speech_text.strip().startswith('да'):
            self.process()


def main():
    #read config
    yandex_voice_key = rospy.get_param('yandex_voice_key')
    apiai_bot_client_key = rospy.get_param('apiai_bot_client_key')
    pyaudio_config = json.loads(rospy.get_param('pyaudio'))
    scheme_file = rospy.get_param('scheme_file')

    scheme_processor = SchemeProcessor(yandex_voice_key=yandex_voice_key, apiai_bot_client_key=apiai_bot_client_key, pyaudio_config=pyaudio_config, scheme_file=scheme_file)
    scheme_processor.process()


if __name__ == '__main__':
    rospy.init_node('bot_teacher')
    main()
