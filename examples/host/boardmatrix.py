#! /usr/bin/env python3

import serial

class BoardMatrix(object):
    def __init__(self, port, xy):
        self.ser = serial.Serial(port, 115200)
        self.current_board = None
        self.board_x = xy[0]
        self.board_y = xy[1]

    def do_cmd(self, cmd, d0, d1, d2):
        n = self.ser.write(bytes((cmd, d0, d1, d2)))
        if n != 4:
            raise Exception("only worte %d" % n)

    def bus_reset(self):
        self.do_cmd(0xff, 0xff, 0xff, 0xff)
        self.do_cmd(0xe0, 0xf0, 0xf1, 0xf2)
        self.current_board = None

    def select_board(self, board):
        self.bus_reset()
        self.do_cmd(0xe1, board, 0, 0)
        self.current_board = board

    def set_brightness(self, rgb):
        self.select_board(0xff)
        self.do_cmd(0xc0, rgb[0], rgb[1], rgb[2])

    def clear(self, rgb=(0,0,0)):
        self.select_board(0xff)
        for i in range(0, 64):
            self.do_cmd(i, 0, 0, 0)

    def set_pixel(self, xy, rgb):
        board = xy[0] >> 3
        pos = (xy[0] & 7) + (xy[1] * 8)
        if board != self.current_board:
            self.select_board(board)
        self.do_cmd(pos, rgb[0], rgb[1], rgb[2])
