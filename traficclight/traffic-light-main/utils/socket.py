import socket
import json
import logging


class SocketServer:

    def __init__(self, socketcfg):
        # self.host = socketcfg['host']
        self.host = None
        self.host = self.get_host()
        self.port = socketcfg['port']
        self.socket = None
        self.conn = None
        self.addr = None
        self.logger = logging.getLogger('app_logger')

    def start(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((self.host, self.port))
        self.socket.listen(1)
        self.logger.info("waiting for the connection of client...")
        self.client, self.addr = self.socket.accept()
        self.logger.info("connect success!")
        self.logger.info(f"client addr is {self.addr}")

    def stop(self):
        if self.socket:
            if self.client:
                self.client.close()
                self.addr = None
                self.client = None
            self.socket.close()
            self.socket = None

    def send_json(self, json_data):
        json_str = json.dumps(json_data)
        self.client.sendall(json_str.encode('utf-8'))

    def receive_json(self):
        json_str = self.client.recv(1024)
        if not json_str:
            return None
        json_data = json.loads(json_str.decode('utf-8'))
        return json_data

    def get_host(self):
        if self.host != None:
            return self.host
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        while True:
            try:
                s.connect(('10.255.255.255', 1))
            except Exception:
                pass
            else:
                local_ip = s.getsockname()[0]
                s.close()
                break
        return local_ip
