#! /usr/bin/env python3
import sys

def fn(c):
    return ord(c.lower()) - ord('a')

sys.stdout.write("/* Generated by gen-board-map.py */\n")

l = sys.stdin.readline().strip()
common = "AK".index(l[0])
fn = l[1:].index


while True:
    l1 = sys.stdin.readline()
    if l1 == "":
        break;
    l2 = sys.stdin.readline()
    l3 = sys.stdin.readline()
    sys.stdin.readline()

    x = map(fn, l1.split())
    y = zip(*[iter(map(fn, l2.split()))]*2)
    z = map(fn, l3.split())
    for (r, (g, k), b) in zip(x, y, z):
        if common == 0:
            # Common anode
            sys.stdout.write("PIXEL(%d,%d), " % (k, r))
            sys.stdout.write("PIXEL(%d,%d), " % (k, g))
            sys.stdout.write("PIXEL(%d,%d),\n" % (k, b))
        else:
            # Common cathode
            sys.stdout.write("PIXEL(%d,%d), " % (r, k))
            sys.stdout.write("PIXEL(%d,%d), " % (g, k))
            sys.stdout.write("PIXEL(%d,%d),\n" % (b, k))
    sys.stdout.write("\n")
