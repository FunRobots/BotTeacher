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

def request():
    speech_audio = rec.listen_audio()
    speech_text = speech_recognizer.recognize_speech(speech_audio)
    bot_answer = bot.request(speech_text)
    request bot_answer

def main():

    #welcome
    bot_answer = request()
    if bot_answer['intent_name'] == '1. Welcome':
        if bot_answer['text'] == 'Добрый день' or bot_answer['text'] == 'Доброе утро' or bot_answer['text'] == 'Добрый вечер' or bot_answer['text'] == 'Здравствуйте':
            #approach
            bot_answer = request()
            if bot_answer['intent_name'] == '2-Approach':
                #listen to consultor and notify the time
                pass

        elif bot_answer['text'] == 'Здравствуйте! Я ищу утюг, который бы автоматически отключался, если его не выключили':
            #needs-detection
            bot_answer = request()
            if bot_answer['intent_name'] == '3-Needs-detection':
                #product presentation
                pass

if __name__ == '__main__':
    main()
