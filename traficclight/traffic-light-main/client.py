import socket
import json
import time
import re

# IP = '192.168.20.120'
# PORT = 12345

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

    # 发送开启检测指令
    json_data = {"action" : "start"}
    json_str = json.dumps(json_data)
    client.sendall(json_str.encode('utf-8'))
    step = 0
    while True:
        json_str = client.recv(1024)
        json_str = json_str.decode('utf-8')
        decoder = json.JSONDecoder()
        # print('origin data: ', json_str)
        # json_data = json.loads(json_str)
        try:
            json_data, end_index = decoder.raw_decode(json_str[0:])
        except Exception as e:
            print('decode failed. no json object.')
            continue
        print(json_data)
        if 'status' in json_data:
            if json_data['status'] == 'error':
                json_data = {"action" : "start"}
                json_str = json.dumps(json_data)
                time.sleep(2)
                client.sendall(json_str.encode('utf-8'))
        step += 1
        
        if step == 5:
            json_data = {"action" : "stop"}
            json_str = json.dumps(json_data)
            client.sendall(json_str.encode('utf-8'))
            print('stop')
            time.sleep(10)
            json_data = {"action" : "start"}
            json_str = json.dumps(json_data)
            client.sendall(json_str.encode('utf-8'))
            step = 0