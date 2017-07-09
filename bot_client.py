#!/usr/bin/env python3

import apiai
import random
import json
from typing import Dict

import logging
import os
LOG_FOLDER = 'logs'
if os.path.exists(LOG_FOLDER) is False:
    os.mkdir(LOG_FOLDER)

logging.basicConfig(filename=LOG_FOLDER + '/' + __name__ + '.log', format='[%(asctime)s] %(message)s\n\n',
                    level=logging.DEBUG)


class APIAIBot:
    '''
    Simple bot class based on api.ai
    Allows to communicate with api.ai bot
    '''

    def __init__(self, client_key: str):
        '''
        Constructor
        Args:
            client_key: api.ai client key
        '''

        self.client_key = client_key
        self._make_session_id()
        self._connect()

    def _make_session_id(self) -> None:
        '''
        Generate random session id
        '''
        symbols = '0123456789abcdef'
        self.session_id = ''
        for i in range(36):
            self.session_id += symbols[random.randint(0, len(symbols) - 1)]

    def _connect(self):
        '''
        Connect to api.ai
        '''
        self._ai = apiai.ApiAI(self.client_key)

    def request(self, msg: str) -> Dict or None:
        '''
        Make request and returns response from api.ai bot
        Args:
            msg: message to send to api.ai bot
        Returns:
            dictionary = {
                'text': text, 'action':{'name': name, 'parameters':parameters
                              }
            }: if data recieved
            None: if failed
        '''

        try:
            req = self._ai.text_request()
            req.lang = 'ru'
            req.session_id = self.session_id
            req.query = msg
            response = json.loads(req.getresponse().read().decode('utf-8'))

            return self._parse_intent(response)
        except Exception as e:
            logging.error(str(e))
            return None

    def _parse_intent(self, intent: Dict) -> Dict:
        '''
        Parse data recieved from api.ai bot
        Args:
            intent: data from api.ai bot
        Returns:
            dictionary = {
                'text': text, 'action':{'name': name, 'parameters':parameters
                              }
            }
        '''

        try:
            answer = dict()
            #extract bot answer
            answer['text'] = intent['result']['fulfillment']['speech']
            answer['action'] = dict()
            answer['action']['name'] = str()
            answer['action']['parameters'] = str()
            #if command is complete (all required fields are filled), extract the command information
            if 'actionIncomplete' in intent['result'] and intent['result']['actionIncomplete'] is False:
                answer['action']['name'] = intent['result']['action']
                answer['action']['parameters'] = intent['result']['parameters']

            return answer
        except Exception as e:
            logging.error(str(e))
            return None
