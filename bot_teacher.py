from recorder import Recorder
from recognizer import SpeechRecognizer
from bot_client import APIAIBot
import json
import time

#read config
config = json.load('config.conf')
yandex_voice_key = config['yandex_voice_key']
apiai_bot_client_key = config['apiai_bot_client_key']
pyaudio_config = config['pyaudio']

#create objects
rec = Recorder(pyaudio_config)
speech_recognizer = SpeechRecognizer(yandex_voice_key=yandex_voice_key)
bot = APIAIBot(client_key=apiai_bot_client_key)

def speech_recognize():
    speech_audio = rec.listen_audio()
    speech_text = speech_recognizer.recognize_speech(speech_audio)
    return speech_text

def main():

    log = open(time.ctime() + '.log', w)

    #welcome
    speech_text = recognize_speech()
    bot_answer = bot.request(speech_text)
    if bot_answer['intent_name'] == '1. Welcome':
        print('Consultant > ', speech_text)
        print('Client > ', bot_answer['text'])

        log.write('[{0}] \n Intent: {1} \n Consultant > {2} \n Client > {3}'.format(time.ctime(), bot_answer['intent_name'], speech_text, bot_answer['text']))

        if bot_answer['text'] == 'Добрый день' or bot_answer['text'] == 'Доброе утро' or bot_answer['text'] == 'Добрый вечер' or bot_answer['text'] == 'Здравствуйте':

            #listen to consultor and notify the time
            speech_text = str()
            start = time.time()
            while len(speech_text) == 0 and time.time() - start < 90:
                speech_text = recognize_speech()
                time.sleep(1)

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

            else:
                print('Client gone!')


        elif bot_answer['text'] == 'Здравствуйте! Я ищу утюг, который бы автоматически отключался, если его не выключили':
            print('Consultant > ', speech_text)
            print('Client > ', bot_answer['text'])
            log.write('[{0}] \n Intent: {1} \n Consultant > {2} \n Client > {3}'.format(time.ctime(), bot_answer['intent_name'], speech_text, bot_answer['text']))
            #needs-detection
            speech_text = speech_recognize()
            bot_answer = bot.request(speech_text)
            if bot_answer['intent_name'] == '3-Needs-detection':
                print('Consultant > ', speech_text)
                print('Client > ', bot_answer['text'])
                log.write('[{0}] \n Intent: {1} \n Consultant > {2} \n Client > {3}'.format(time.ctime(), bot_answer['intent_name'], speech_text, bot_answer['text']))
                #product presentation
                #Consultant must make product presentation

    log.close()
    print('Repeat?')
    speech_text = recognize_speech()
    if speech_text.strip().startswith('да'):
        main()

if __name__ == '__main__':
    main()
