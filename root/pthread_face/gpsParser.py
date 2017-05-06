#https://gist.github.com/wolfg1969/4653340

import threading
import os 
import time
from gps import *
from time import * 
from timeit import default_timer as timer

class GpsPoller(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.session = gps(mode=WATCH_ENABLE)
        self.gps_get = None 
        self.running = True

    def get_gps_value(self):
        return self.gps_get

    def run(self):
        try:
            while self.running:
                self.gps_get = self.session.next() 
        except StopIteration:
            pass

