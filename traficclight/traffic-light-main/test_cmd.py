import socket
import json
import time
import re
import threading

def receive():
    while True:
        json_str = client.recv(1024)
        json_str = json_str.decode('utf-8')
        decoder = json.JSONDecoder()
        try:
            json_data, end_index = decoder.raw_decode(json_str[0:])
        except Exception as e:
            print('decode failed. no json object.')
        print(json_data)

if __name__ == '__main__':
    with open('config.json', 'r', encoding='utf-8') as file:
        config = json.load(file)
    port = config['socket']['port']
    broadcast_port = config['socket']['broadcast_port']
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', broadcast_port))
    ip, addr = sock.recvfrom(1024)
    print(f"ip = {ip}, port = {port}, broadcast_port = {broadcast_port}")
    sock.close()
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect((ip, port))

    receive_threading = threading.Thread(target=receive,daemon=True)
    receive_threading.start()
    while True:
        print("Input start or stop:")
        cmd = input()
        json_data = {"action" : f"{cmd}"}
        json_str = json.dumps(json_data)
        client.sendall(json_str.encode('utf-8'))

    # json_data = {"action" : "start"}
    # json_str = json.dumps(json_data)
    # client.sendall(json_str.encode('utf-8'))
    # json_str = client.recv(1024)
    # json_str = json_str.decode('utf-8')
    # decoder = json.JSONDecoder()
    # try:
    #     json_data, end_index = decoder.raw_decode(json_str[0:])
    # except Exception as e:
    #     print('decode failed. no json object.')
    # print(json_data)

    # json_data = {"action" : "start"}
    # json_str = json.dumps(json_data)
    # client.sendall(json_str.encode('utf-8'))

    # time.sleep(10)
    # json_str = client.recv(1024)
    # json_str = json_str.decode('utf-8')
    # decoder = json.JSONDecoder()
    # try:
    #     json_data, end_index = decoder.raw_decode(json_str[0:])
    # except Exception as e:
    #     print('decode failed. no json object.')
    # print(json_data)