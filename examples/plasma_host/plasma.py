#! /usr/bin/env python3

from boardmatrix import BoardMatrix
from numpy import *
from math import pi
import time

board_x = 6
board_y = 1

def color_from_val(val):
    if val < 85:
        r = val * 3;
        g = 255 - r;
        b = 0;
    elif val < 170:
        b = (val - 85) * 3;
        r = 255 - b;
        g = 0;
    else:
        g = (val - 170) * 3;
        b = 255 - g;
        r = 0;
    return (r, g, b)

def pixel_value(x, y):
  xf = ((x + 0.0) * (pi / 8.0)) * (8/6.0);
  yf = ((y + 0.0) * (pi / 8.0)) * (8/6.0);

  u = cos(xf) + 1.0;
  v = cos(yf) + 1.0;
  return int((u + v) * 255.9/4.0)
    

class plasma(object):
    def __init__(self):
        self.m = BoardMatrix(None, (board_x, board_y))
        self.m.set_brightness((0x30, 0x45, 0x40))
        #self.m.set_brightness((0x08, 0x04, 0x04))

    def frame(self, n):
        for x in range(0, 8*board_x):
            for y in range(0, 8):
                val = pixel_value(x, y)
                val = (val + n) & 0xff
                color = color_from_val(val)
                self.m.set_pixel((x, y), color)

p = plasma()
tick = time.time()
try:
    while True:
        for n in range(0, 255):
            p.frame(n)
            now = time.time()
            if now < tick:
                time.sleep(tick - now)
            else:
                print("Frameskip")
            tick += 5.0/256
except KeyboardInterrupt:
    p.m.clear()
