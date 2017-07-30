from pathlib import Path
top = Path(__file__).resolve().parents[0].as_posix()

import logging
import os
LOG_FOLDER = top + '/logs'
if os.path.exists(LOG_FOLDER) is False:
    os.mkdir(LOG_FOLDER)


class ErrorLogger:
    '''
    append log file with specified filename
    '''

    def __init__(sefl, log_file_name: str, e: Exception):
        '''
        Constructor
        open and append log file
        Args:
            log_file_name: name of log file
            e: object of Exception class
        '''

        try:
            path_parts = log_file_name.split('/')
            log_file = path_parts[-1:][0]
            print(LOG_FOLDER + '/' + log_file + '.log')
            logger = logging.getLogger()
            logger.setLevel(logging.DEBUG)
            handler = logging.FileHandler(LOG_FOLDER + '/' + log_file + '.log')
            handler.setLevel(logging.ERROR)
            formatter = logging.Formatter('[%(asctime)s] %(message)s\n\n')
            handler.setFormatter(formatter)
            logger.addHandler(handler)
            logger.error(e, exc_info=True)
        except Exception as e:
            print(str(e))
            pass
