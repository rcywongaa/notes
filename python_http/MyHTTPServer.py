#!/usr/bin/python3

import http.server
import socketserver

PORT = 8000

class MyHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        print("GET received!")
        self.wfile.write(bytes("RESPONSE", "utf-8"))

httpd = socketserver.TCPServer(("", PORT), MyHandler)
print("serving at port", PORT)
httpd.serve_forever()
