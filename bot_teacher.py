from recorder import Recorder
from recognizer import SpeechRecognizer
from bot_client import APIAIBot
import json
import time
import audio_format_converter

from utils import ErrorLogger

#read config
config = json.load(open('config.conf','r'))
yandex_voice_key = config['yandex_voice_key']
apiai_bot_client_key = config['apiai_bot_client_key']
pyaudio_config = config['pyaudio']

#create objects
rec = Recorder(pyaudio_config, min_rms=1000)
speech_recognizer = SpeechRecognizer(yandex_voice_key=yandex_voice_key)
bot = APIAIBot(client_key=apiai_bot_client_key)

def recognize_speech():
    speech_audio = rec.listen_audio()
    speech_text = speech_recognizer.recognize_speech(audio_format_converter.raw_audio2wav(speech_audio, pyaudio_config))
    return speech_text

#bad speech recognition client phrase
BAD_SPEECH_RECOGNITION = 'Повторите, пожалуйста, еще раз. Плохо слышно'

#----------------------------------------------------------------
log = None

def welcome():
    global log
    print('listen...\n')
    speech_text = recognize_speech()
    bot_answer = bot.request(speech_text)
    if bot_answer['intent_name'] == '1.Welcome':
        print('Consultant > ', speech_text)
        print('Client > ', bot_answer['text'])

        log.write('[{0}] \n Intent: {1} \n Consultant > {2} \n Client > {3}'.format(time.ctime(), bot_answer['intent_name'], speech_text, bot_answer['text']))

        if bot_answer['text'] == 'Добрый день' or bot_answer['text'] == 'Доброе утро' or bot_answer['text'] == 'Добрый вечер' or bot_answer['text'] == 'Здравствуйте':
            approach()
        elif bot_answer['text'] == 'Здравствуйте! Я ищу утюг, который бы автоматически отключался, если его не выключили':
            needs_detection()
    else:
        print(BAD_SPEECH_RECOGNITION)
        welcome()


def approach():
    global log
    #listen to consultor and notify the time
    speech_text = str()
    start = time.time()
    while time.time() - start < 90:
        print('listen...\n')
        speech_text = recognize_speech()
        if speech_text is not None and len(speech_text) > 0:
            break
    
    pause = time.time() - start

    if pause < 30:
        #approach-decision-leave
        bot_answer = bot.request('goodbye')
        print('Consultant > ', speech_text)
        print('Client > ', bot_answer['text'])
        log.write('[{0}] \n Intent: {1} \n Consultant > {2} \n Client > {3}'.format(time.ctime(), bot_answer['intent_name'], speech_text, bot_answer['text']))
    elif pause < 90:
        #approach
        bot_answer = bot.request(speech_text)
        if bot_answer['intent_name'] == '2-Approach':
            print('Consultant > ', speech_text)
            #approach-decision-continue
            bot_answer = bot.request('continue')
            print('Client > ', bot_answer['text'])
            log.write('[{0}] \n Intent: {1} \n Consultant > {2} \n Client > {3}'.format(time.ctime(), bot_answer['intent_name'], speech_text, bot_answer['text']))
            #product presentation
            #NEED FOR PRODUCT PRESENTATION SCENARIO

        else:
            print(BAD_SPEECH_RECOGNITION)
            approach()

    else:
        print('Client gone!')


def needs_detection():
    global log
    print('listen...\n')
    speech_text = recognize_speech()
    bot_answer = bot.request(speech_text)
    if bot_answer['intent_name'] == '3-Needs-detection':
        print('Consultant > ', speech_text)
        print('Client > ', bot_answer['text'])
        log.write('[{0}] \n Intent: {1} \n Consultant > {2} \n Client > {3}'.format(time.ctime(), bot_answer['intent_name'], speech_text, bot_answer['text']))
        #product presentation
        #NEED FOR PRODUCT PRESENTATION SCENARIO

    else:
        print(BAD_SPEECH_RECOGNITION)
        needs_detection()
        
#----------------------------------------------------------------

def main():
    global log    
    log = open('stat/' + time.ctime() + '.log', 'w')
    
    print('listen...\n')
    #welcome
    try:
       welcome()
       log.close()
       print('Repeat?')
       speech_text = recognize_speech()
       if speech_text is not None and speech_text.strip().startswith('да'):
           main()
           
    except Exception as e:
        ErrorLogger(__file__, e)

if __name__ == '__main__':
    main()
