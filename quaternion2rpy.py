#!/usr/bin/python2

'''
This program reads quaternion values from stdin or pipe in the form
x: 0.0
y: 0.0
z: 0.0
w: 0.0
and prints out the euler angles to stdout in the form
roll: 0.0
pitch: 0.0
yaw: 0.0
'''

import tf
import sys
import argparse

def printEuler(x, y, z, w):
    quaternion = (x, y, z, w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    print("roll: " + str(euler[0]))
    print("pitch: " + str(euler[1]))
    print("yaw: " + str(euler[2]))

has_x = False
has_y = False
has_z = False
has_w = False
def reset():
    global has_x
    global has_y
    global has_z
    global has_w
    has_x = False
    has_y = False
    has_z = False
    has_w = False

try:
    buff = ''
    while True:
        line = sys.stdin.readline()
        x_index = line.find("x: ")
        y_index = line.find("y: ")
        z_index = line.find("z: ")
        w_index = line.find("w: ")

        if x_index != -1:
            if has_x:
                print("Duplicated x entries!")
                sys.exit(0)
            else:
                x = float(line[x_index+3:])
                has_x = True
        elif y_index != -1:
            if has_y:
                print("Duplicated y entries!")
                sys.exit(0)
            else:
                y = float(line[y_index+3:])
                has_y = True
        elif z_index != -1:
            if has_z:
                print("Duplicated z entries!")
                sys.exit(0)
            else:
                z = float(line[z_index+3:])
                has_z = True
        elif w_index != -1:
            if has_w:
                print("Duplicated w entries!")
                sys.exit(0)
            else:
                w = float(line[w_index+3:])
                has_w = True
        else:
            print("Unknown line encountered: " + str(line))

        if has_x and has_y and has_z and has_w:
            printEuler(x, y, z, w)
            reset()

except KeyboardInterrupt:
    sys.stdout.flush()
    sys.exit(0)
