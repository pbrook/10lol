#! /usr/bin/env python3

from boardmatrix import BoardMatrix
import math
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

scale = (math.pi / 8.0) * (8/6.0)
def pixel_value(x, y):
  xf = (x + 0.0) * scale
  yf = (y + 0.0) * scale

  u = math.cos(xf) + 1.0;
  v = math.cos(yf) + 1.0;
  return int((u + v) * 255.9/4.0)

class plasma(object):
    def __init__(self):
        self.m = BoardMatrix(None, (board_x, board_y))
        self.m.set_brightness((0x30, 0x45, 0x40))
        #self.m.set_brightness((0x08, 0x04, 0x04))
        self.init_pixels()

    def init_pixels(self):
        pass

    def frame(self, n):
        for x in range(0, 8*board_x):
            for y in range(0, 8):
                val = pixel_value(x, y)
                val = (val + n) & 0xff
                color = color_from_val(val)
                self.m.set_pixel((x, y), color)

p = plasma()
tick = time.time()
n = 0.0
minframe = 1.0/20
try:
    while True:
        if n >= 256:
            n -= 256
        p.frame(int(n))
        now = time.time()
        delta = now - tick
        #print(1.0/delta)
        n += delta * 256 / 5.0
        if delta < minframe:
            time.sleep(minframe - delta)
        tick = now
except KeyboardInterrupt:
    p.m.clear()
