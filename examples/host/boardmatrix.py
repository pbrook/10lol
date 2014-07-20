#! /usr/bin/env python3

import os

port_list = ['/dev/ttyACM0', '/dev/spidev1.0']

def SerialWriter(port):
    import serial
    return serial.Serial(port, 115200)

class SPIWriter(object):
    def __init__(self, port):
        import spidev
        if port[:11] == '/dev/spidev':
            port = port[11:]
        bus = None
        for sep in '.,:-':
            if sep in port:
                p = port.split(sep)
                bus = int(p[0])
                dev = int(p[1])
                break
        if bus is None:
            if port == '':
                bus = 1
            else:
                bus = int(port)
            dev = 0
        spi = spidev.SpiDev(bus, dev)
        spi.max_speed_hz = 2000000
        spi.mode = 3
        spi.lsbfirst = False
        spi.cshigh = False
        spi.bits_per_word = 8
        self.spi = spi
    def write(self, b):
        self.spi.writebytes(b)

def find_port(l):
    for p in l:
        if os.access(p, os.R_OK | os.W_OK):
            return p
    raise Exception("Unable to find suitable port")

class BoardMatrix(object):
    def __init__(self, port, xy):
        if port is None:
            port = find_port(port_list)
        if 'spi' in port:
            self.ser = SPIWriter(port)
        else:
            self.ser = SerialWriter(port)
        self.current_board = None
        self.board_x = xy[0]
        self.board_y = xy[1]

    def do_cmd(self, cmd, d0, d1, d2):
        self.ser.write(bytearray((cmd, d0, d1, d2)))

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
