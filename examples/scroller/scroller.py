#! /usr/bin/env python
# Scroll a message
# Copyright (C) Paul Brook <paul@nowt.org>
# Released under the terms of the GNU General Public License version 3

import boardmatrix
import font
import math
import time
import sys
import random

board_x = 6
board_y = 1

def random_color():
    r = 0
    g = 0
    b = 0
    while r == 0 and g == 0 and b == 0:
        r = random.choice((0, 0xff))
        g = random.choice((0, 0xff))
        b = random.choice((0, 0xff))
    return (r, g, b)

class Scroller(object):
    def __init__(self, msg):
        self.m = boardmatrix.BoardMatrix(None, (board_x, board_y))
        self.width = 8 * board_x
        self.double_buffer = True
        self.bitmap = [0] * self.width
        self.color = random_color()
        self.message = msg
    def frame(self):
        while len(self.bitmap) < self.width:
            if self.message == '':
                break;
            c = self.message[0]
            self.message = self.message[1:]
            n = ord(c) - 32
            if n >= 0 and n < len(font.font_data):
                self.bitmap.extend(font.font_data[n])
            self.bitmap.append(0)
        if len(self.bitmap) == 0:
            raise StopIteration
        for x in range(0, min(len(self.bitmap), self.width)):
            mask = self.bitmap[x]
            for y in range(0, 8):
                if mask & (0x01 << y):
                    color = self.color
                else:
                    color = (0,0,0)
                self.m.set_pixel((x, y), color)
        self.bitmap.pop(0)

msg = ' '.join(sys.argv[1:])
if msg == '':
    msg = "Leeds Hackspace"
s = Scroller(msg)
tick = time.time()
interval = 1.0/20
try:
    while True:
        s.frame()
        now = time.time()
        delta = now - tick
        #print(1.0/delta)
        if delta < interval:
            time.sleep(interval - delta)
        tick += interval

except KeyboardInterrupt:
    pass
except StopIteration:
    pass
finally:
    s.m.clear()
