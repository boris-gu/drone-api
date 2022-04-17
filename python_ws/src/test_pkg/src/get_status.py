#!/usr/bin/env python3

from drone_api import *


def toFixed(numObj):
    return f'{numObj:.2f}'


telem = Telemerty_api()
while not Drone_api.is_shutdown():
    print('\n\n\n\n\n', telem.state)
    Drone_api.sleep(0.5)
