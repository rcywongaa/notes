#!/usr/bin/python3

import argparse
import cv2
import xmlrpc.client
import time

parser = argparse.ArgumentParser()
parser.add_argument('filename', metavar='filename', type=str, help='Name of jpg file')
args = parser.parse_args()

with open(args.filename, "rb") as handle:
    serialized = xmlrpc.client.Binary(handle.read())
    with xmlrpc.client.ServerProxy("http://localhost:8000/RPC2") as proxy:
        print(str(proxy.system.listMethods()))
        print(str(proxy.getObjectSize(serialized)))
        print(str(proxy.isPaperPresent(serialized)))

        while True:
            print(str(proxy.getX()))
            time.sleep(1)
