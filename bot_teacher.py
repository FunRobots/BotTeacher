from recorder import Recorder
from recognizer import SpeechRecognizer

from bot_client import APIAIBot
import json

import time


if __name__ == '__main__':
    #read config
    config = json.load('config.conf')
    yandex_voice_key = config['yandex_voice_key']
    apiai_bot_client_key = config['apiai_bot_client_key']

    #create objects
    speech_recognizer = SpeechRecognizer(yandex_voice_key=yandex_voice_key)
    bot = APIAIBot(client_key=apiai_bot_client_key)

    
