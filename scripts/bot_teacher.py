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

#read config
yandex_voice_key = rospy.get_param('yandex_voice_key')
apiai_bot_client_key = rospy.get_param('apiai_bot_client_key')
pyaudio_config = json.loads(rospy.get_param('pyaudio'))

#create objects
rec = Recorder(pyaudio_config, min_rms=1000)
speech_recognizer = SpeechRecognizer(yandex_voice_key=yandex_voice_key)
talker = Talker(yandex_voice_key=yandex_voice_key)
player = Player()
bot = APIAIBot(client_key=apiai_bot_client_key)

def recognize_speech():
    if rospy.has_param('min_rms'):
        rec.set_min_rms(min_rms = rospy.get_param('min_rms'))

    speech_audio = rec.listen_audio(timeout=91)
    if speech_audio is not None:
        speech_text = speech_recognizer.recognize_speech(audio_format_converter.raw_audio2wav(speech_audio, pyaudio_config))
        return speech_text
    return str()

def say_text(text: str):
    speech = talker.text_to_speech(text)
    wav_source = io.BytesIO(speech)
    wav_source.seek(0)
    player.play_audio(wav_source)

#bad speech recognition client phrase
BAD_SPEECH_RECOGNITION = 'Повторите, пожалуйста, еще раз. Плохо слышно'
STAT_FOLDER = 'stat'
#----------------------------------------------------------------
log = None

def main():
    global log
    global STAT_FOLDER

    if os.path.exists(STAT_FOLDER) is False:
        os.mkdir(STAT_FOLDER)
    log = open('stat/' + time.ctime() + '.log', 'w')

    try:
        scheme_file = Path(__file__).resolve().parents[1].as_posix() + '/run/' + rospy.get_param('scheme_file')
        scheme = json.load(open(scheme_file,'r'))

        current_intent = scheme.get('start')
        while current_intent is not None:
            speech_text = str()
            start = time.time()
            while time.time() - start < 90:
                print('listen...\n')
                speech_text = recognize_speech()
                if speech_text is not None and len(speech_text) > 0:
                    break

            pause = time.time() - start
            print('Consultant > ', speech_text)

            wait_cases_dict = scheme['intents'][current_intent].get('wait_before_transition')
            if wait_cases_dict is not None:

                wait_keys = [int(i) for i in wait_cases_dict.keys()]
                wait_keys.sort(reverse=True)

                print('wait_keys', wait_keys)

                case_key_index = 0
                while case_key_index < len(wait_keys) and pause < wait_keys[case_key_index]:
                    case_key_index += 1

                case = wait_cases_dict[str(wait_keys[case_key_index])]
                print('case', case)

                phrase_to_say = case.get('say_phrase')
                if phrase_to_say is not None:
                    bot_answer = bot.request(phrase_to_say)
                    print('Client > ', bot_answer.get('text'))
                    say_text(bot_answer.get('text'))
                    log.write('[{0}] \n Intent: {1} \n Consultant > {2} \n Client > {3}'.format(time.ctime(), bot_answer['intent_name'], speech_text, bot_answer['text']))
                else:
                    bot_answer = bot.request(speech_text)
                    if isinstance(bot_answer, dict) and bot_answer.get('intent_name') == current_intent:
                        print('Client > ', bot_answer.get('text'))
                        say_text(bot_answer.get('text'))
                        log.write('[{0}] \n Intent: {1} \n Consultant > {2} \n Client > {3}'.format(time.ctime(), bot_answer['intent_name'], speech_text, bot_answer['text']))
                        current_intent = case.get('goto_next')
                    else:
                        print(BAD_SPEECH_RECOGNITION)
                        say_text(BAD_SPEECH_RECOGNITION)
                        continue

            else:
                print(current_intent)
                bot_answer = bot.request(speech_text)
                if isinstance(bot_answer, dict) and bot_answer.get('intent_name') == current_intent:
                    print('Client > ', bot_answer.get('text'))
                    say_text(bot_answer.get('text'))
                    log.write('[{0}] \n Intent: {1} \n Consultant > {2} \n Client > {3}'.format(time.ctime(), bot_answer['intent_name'], speech_text, bot_answer['text']))
                    current_intent = scheme['intents'][current_intent].get('goto_next')
                else:
                    print(BAD_SPEECH_RECOGNITION)
                    say_text(BAD_SPEECH_RECOGNITION)
                    continue

        log.close()
        print('Repeat?')
        speech_text = recognize_speech()
        if speech_text is not None and speech_text.strip().startswith('да'):
            main()

    except Exception as e:
        ErrorLogger(__file__, e)


if __name__ == '__main__':
    rospy.init_node('bot_teacher')
    main()
