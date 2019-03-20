#!/usr/bin/python3

import cv2
import xmlrpc.server
import xmlrpc.client
import pickle
import pdb
import numpy as np
import object_size
import paper_detection

PORT = 8000

def get_img_bgr(serialized_file):
    file_bytes = np.fromstring(serialized_file.data, dtype=np.uint8)
    return cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)

def xmlrpc_get_object_size(serialized_file):
    img_bgr = get_img_bgr(serialized_file)
    return object_size.get_size(img_bgr)

def xmlrpc_is_paper_present(serialized_file):
    img_bgr = get_img_bgr(serialized_file)
    return paper_detection.is_paper(img_bgr)

class PersistentService:
    def __init__(self):
        self.x = 0

    def getX(self):
        self.x += 1
        return self.x

with xmlrpc.server.SimpleXMLRPCServer(('localhost', PORT)) as server:
    print("serving at port", PORT)
    server.register_introspection_functions()
    server.register_function(xmlrpc_get_object_size, "getObjectSize")
    server.register_function(xmlrpc_is_paper_present, "isPaperPresent")
    server.register_instance(PersistentService())
    server.register_multicall_functions()
    try:
        server.serve_forever()
    finally:
        server.server_close()
