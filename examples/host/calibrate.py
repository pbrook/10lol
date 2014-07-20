#! /usr/bin/env python

from boardmatrix import BoardMatrix
import curses
import time

m = BoardMatrix(None, (1,1))

def set_all(board, rgb):
    m.select_board(board)
    for i in range(0, 64):
        m.do_cmd(i, rgb[0], rgb[1], rgb[2])

def blink_board(board):
    set_all(board, (0xff, 0x00, 0x00))
    time.sleep(1)
    set_all(board, (0x00, 0xff, 0x00))
    time.sleep(1)
    set_all(board, (0x00, 0x00, 0xff))
    time.sleep(1)
    set_all(board, (0x00, 0x00, 0x00))
    time.sleep(0.1)

def set_brightness():
    m.set_brightness((0x30, 0x45, 0x40))
    #m.set_brightness((0x08, 0x04, 0x04))

def main(stdscr):
    set_all(0xff, (0, 0, 0))
    time.sleep(0.1)
    set_brightness()
    blink_board(0xff)

    n = None
    while True:
        key = stdscr.getkey()
        if key == 'q':
            raise StopIteration
        if (key >= '0') and (key <= '9'):
            if n is None:
                n = 0
            n = n * 10 + int(key)
        elif (key == '\r' or key == '\n') and n is not None:
            stdscr.addstr("Setting address %d\n" % n)
            stdscr.refresh()
            m.bus_reset()
            m.do_cmd(0xd0, 0xf5, (n >> 4) | 0xf0, n | 0xf0)
            time.sleep(0.1)
            blink_board(n)
            m.bus_reset()
            m.select_board(0xff)
            stdscr.addstr("Done\n")
            stdscr.refresh()
            n = None
        else:
            if key == 'r':
                set_all(0xff, (0xff, 0x00, 0x00))
            elif key == 'g':
                set_all(0xff, (0x00, 0xff, 0x00))
            elif key == 'b':
                set_all(0xff, (0x00, 0x00, 0xff))
            elif key == 'w':
                set_all(0xff, (0xff, 0xff, 0xff))
            else:
                set_all(0xff, (0x00, 0x00, 0x00))
            n = 0
            got_data = False

try:
    curses.wrapper(main)
except StopIteration:
    pass
