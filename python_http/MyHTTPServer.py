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
        self.wfile.write(bytes("GET RESPONSE", "utf-8"))
    def do_POST(self):
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        content_len = int(self.headers.get('Content-Length'))
        post_body = self.rfile.read(content_len)
        print("POST received!")
        self.wfile.write(bytes("POST RESPONSE", "utf-8"))

httpd = socketserver.TCPServer(("", PORT), MyHandler)
print("serving at port", PORT)

try:
    httpd.serve_forever()
finally:
    httpd.server_close()
