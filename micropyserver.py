import re
import socket
import sys
import utime
import select
import errno

class MicroPyServer(object):

    def __init__(self, host="0.0.0.0", port=80):
        self._host = host
        self._port = port
        self._routes = []
        self._connect = None
        self._sock = None
        self.log("Server initialized")

    def start(self):
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind((self._host, self._port))
            self._sock.setblocking(0)  # Set the socket to non-blocking
            self._sock.listen(5)
            self.log("Server started")
        except Exception as e:
            self.log(f"Error starting server: {e}", error=True)
            sys.exit(1)

    def loop(self):
        if self._sock is None:
            return

        read_sockets, _, _ = select.select([self._sock], [], [], 0)
        for sock in read_sockets:
            print(sock)
            if sock is self._sock:
                print(True)
                client, address = sock.accept()
                print(client)
                self.handle_client(client, address)
                
    def handle_client(self, client, address):
        client.setblocking(0)  # Set the client socket to non-blocking
        try:
            ready_to_read, _, _ = select.select([client], [], [], 5)
            if ready_to_read:
                request = self.get_request(client)
                print("receive: " + request)
                if request:
                    self.log(f"Request from {address}: {request}")
                    route = self.find_route(request)
                    if route:
                        self.log(f"Route found: {route['path']}")
                        route["handler"](client, request)
                    else:
                        self._route_not_found(request)
                else:
                    self.log("Empty request received", warning=True)
        except Exception as e:
            self.log(f"Error handling client {address}: {e}", error=True)
        finally:
            client.close()
                
    def find_route(self, request):
        """ Find route """
        lines = request.split("\r\n")
        method = re.search("^([A-Z]+)", lines[0]).group(1)
        path = re.search("^[A-Z]+\\s+(/[-a-zA-Z0-9_.]*)", lines[0]).group(1)
        for route in self._routes:
            if method != route["method"]:
                continue
            if path == route["path"]:
                return route
            else:
                match = re.search("^" + route["path"] + "$", path)
                if match:
                    print(method, path, route["path"])
                    return route

    def stop(self):
        if self._sock:
            self._sock.close()
            self._sock = None
            self.log("Server stopped")

    def add_route(self, path, handler, method="GET"):
        self._routes.append({"path": path, "handler": handler, "method": method})
        self.log(f"Route added: {method} {path}")

    def get_request(self, client, buffer_length=4096):
        try:
            request = client.recv(buffer_length)
            print('receive: ' + str(request, "utf8"))
            if request:
                return str(request, "utf8")
            else:
                return ""
        except Exception as e:
            self.log(f"Error receiving request: {e}", error=True)
            return ""

    def _route_not_found(self, request):
        self.send("HTTP/1.0 404 Not Found\r\n")
        self.send("Content-Type: text/plain\r\n\r\n")
        self.send("Not found")
        self.log("404 Not Found sent")

    def log(self, message, error=False, warning=False):
        """ Simplified logging function for MicroPython """
        log_type = "ERROR" if error else "WARNING" if warning else "INFO"
        timestamp = utime.localtime(utime.time())
        formatted_time = "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(*timestamp)
        print("[{}] [{}] {}".format(formatted_time, log_type, message))

    def send(self, client, data):
        if client:
            try:
                client.sendall(data.encode())
                self.log(f"Response sent: {len(data)} bytes")
            except Exception as e:
                self.log(f"Error sending response: {e}", error=True)
