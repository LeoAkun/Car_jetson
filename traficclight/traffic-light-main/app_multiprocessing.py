import threading
import time
from utils.socket import SocketServer
from utils.camera import GstreamerCamera
from utils.model.model import Model
# from utils.model.trafficlight import TrafficLightModel
import json
import atexit
import multiprocessing
import numpy as np
import sys
import logging
import atexit
import os
import socket
import queue


def detect(capture_event, detect_event, lock, config, shape, freq,
           shared_frame_array, socket_queue, error_queue):
    """
    模型推理进程
    """
    logger = logging.getLogger('app_logger')
    try:
        logger.info('try to init model...')
        with lock:
            model = Model(config['model'])
            # model = TrafficLightModel(config['model'])
    except Exception as e:
        logger.error(f'Model init error: {e}')
        error_queue.put(("detect", str(e)))
        sys.exit(-1)
    logger.info("detect init success, start detect processing.")
    while True:
        capture_event.wait()    # 保证摄像头读取到了图像
        detect_event.wait()     # 保证推理开启
        with lock:
            frame = np.frombuffer(shared_frame_array.get_obj(), dtype=np.uint8).reshape(shape)
        start_time = time.time()
        try:
            result = model(frame=frame)
            logger.info(f'detect result: {result}')
        except Exception as e:
            logger.error(f'Model inference error: {e}')
            error_queue.put(("detect", str(e)))
            sys.exit(-1)
        else:
            finish_time = time.time()
            task_time = finish_time - start_time    # 单次任务所用时间
            logger.debug(f"tast_time = {task_time}")
            socket_queue.put(result)                # 结果存放入socket消息队列
            delay_time = 1 / freq - task_time       # 根据发送频率计算延时
            if delay_time > 0:
                time.sleep(delay_time)


def capture(capture_event, lock, config, shape,
            shared_frame_array, error_queue):
    """
    捕获图像进程
    """
    logger = logging.getLogger('app_logger')
    try:
        logger.info('try to init camera...')
        with lock:
            camera = GstreamerCamera(config['camera'])
    except Exception as e:
        # 暂停检测进程
        capture_event.clear()
        # camera.close()
        logger.error(f'Camera init error: {e}')
        error_queue.put(("capture", str(e)))
        sys.exit(-1)
    logger.info("capture init success, start capture processing.")
    while True:
        try:
            frame = camera.read()
        except Exception as e:
            # 暂停检测进程
            capture_event.clear()
            camera.close()
            logger.error(f'Camera read error: {e}')
            error_queue.put(("capture", str(e)))
            sys.exit(-1)
        else:
            with lock:
                np.frombuffer(shared_frame_array.get_obj(), dtype=np.uint8).reshape(shape)[:] = frame
            # 摄像头读取图像就绪
            capture_event.set()
            if config['camera']['mode'] == 'video':
                time.sleep(1 / config['camera']['framerate'])


def dosocket(config, socket_queue, error_queue, detect_event):
    """
    socket相关进程
    """
    logger = logging.getLogger('app_logger')
    # 设置线程的socket_queue,和error_queue
    threading_socket_queue = queue.Queue()
    threading_error_queue = queue.Queue()
    threading_cmd_queue = queue.Queue()
    # 初始化socket服务器
    server = SocketServer(config['socket'])
    # 获取本地ip
    local_ip = server.get_host()
    send_threading = threading.Thread(target=send, daemon=True, args=(
        logger, server, threading_socket_queue, threading_error_queue
    ))
    receive_threading = threading.Thread(target=receive, daemon=True, args=(
        logger, server, threading_socket_queue, threading_error_queue, threading_cmd_queue
    ))
    broadcast_treading = threading.Thread(target=broadcast, daemon=True, args=(
        logger, local_ip, config['socket']['broadcast_port'], threading_error_queue
    ))
    # 广播线程启动
    broadcast_treading.start()
    # 等待客户端连接
    try:
        server.start()
    except Exception as e:
        error_queue.put(('dosocket', str(e)))
        sys.exit(-1)
    # 开启主进程的子线程
    send_threading.start()
    receive_threading.start()

    # 单独与其他进程通信
    while True:
        # 发送或接收线程错误
        if not threading_error_queue.empty():
            error_temp = threading_error_queue.get()
            if error_temp[0] == 'send' or error_temp[0] == 'receive':
                    # 等待客户端连接
                    try:
                        # 挂起检测进程
                        detect_event.clear()
                        server.stop()
                        server.start()
                    except Exception as e:
                        logger.error(f"restart socket failed: {str(e)}")
                        error_queue.put(('dosocket', str(e)))
                        sys.exit(-1)
                    else:
                        send_threading.join()
                        receive_threading.join()
                        send_threading = threading.Thread(target=send, daemon=True, args=(
                            logger, server, threading_socket_queue, threading_error_queue
                        ))
                        receive_threading = threading.Thread(target=receive, daemon=True, args=(
                            logger, server, threading_socket_queue, threading_error_queue, threading_cmd_queue
                        ))
                        send_threading.start()
                        receive_threading.start()
            elif error_temp[0] == 'broadcast':
                error_queue.put(('dosocket', str(e)))
        if not socket_queue.empty():
            message_temp = socket_queue.get()
            threading_socket_queue.put(message_temp)
        if not threading_cmd_queue.empty():
            cmd_temp = threading_cmd_queue.get()
            if cmd_temp == 'start':
                detect_event.set()
            elif cmd_temp == 'stop':
                detect_event.clear()
        time.sleep(0.01)

def broadcast(logger, message:str, broadcast_port, threading_error_queue):
    """
    通过socket广播本机ip地址
    """
    # 创建UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    broadcast_address = '<broadcast>'
    logger.info(f'broadcast local ip: {message}, broadcast port: {broadcast_port}')
    try:
        while True:
            sock.sendto(message.encode('utf-8'), (broadcast_address, broadcast_port))
            time.sleep(1)
    except Exception as e:
        logger.error("broadcast error.")
        threading_error_queue.put(("broadcast", str(e)))
    finally:
        sock.close()

def send(logger, server, threading_socket_queue, threading_error_queue):
    """
    发送socket消息的子线程
    """
    while True:
        message = threading_socket_queue.get()
        if message is None:
            return
        try:
            server.send_json(message)
        except Exception as e:
            logger.error(f'Socket send message error: {e}')
            threading_error_queue.put(("send", str(e)))
            return

def receive(logger, server, threading_socket_queue, threading_error_queue, threading_cmd_queue):
    """
    接收socket消息的子线程
    """
    while True:
        try:
            receive_data = server.receive_json()
            if receive_data is None:
                threading_socket_queue.put(None)
                raise RuntimeError('None data received')
        except Exception as e:
            logger.error(f'Socket receive message error: {e}')
            threading_error_queue.put(("receive", str(e)))
            return
        else:
            logger.info(f'receive data is: {receive_data}')
            if receive_data['action'] == 'start':
                response_data = {"status" : 'success', "message" : 'start detecting'}
                # 开始检测进程
                threading_cmd_queue.put('start')
                threading_socket_queue.put(response_data)     # 将成功消息添加到socket消息队列
            elif receive_data['action'] == 'stop':
                response_data = {"status" : 'success', "message" : 'stop detecting'}
                # 暂停检测进程
                threading_cmd_queue.put('stop')
                threading_socket_queue.put(response_data)     # 将成功消息添加到socket消息队列


def app_cleanup():
    if detect_processing.is_alive():
        detect_processing.terminate()
    if capture_processing.is_alive():
        capture_processing.terminate()
    if dosocket_processing.is_alive():
        dosocket_processing.terminate()
    detect_processing.join()
    capture_processing.join()
    dosocket_processing.join()
    logger.info(f"app will restart after {restart_time} seconds")
    listener.stop()
    log_queue.close()
    log_queue.join_thread()
    error_queue.close()
    error_queue.join_thread()
    socket_queue.close()
    socket_queue.join_thread()
    time.sleep(restart_time)
    os.execv(sys.executable, [python] + sys.argv)
    

if __name__ == '__main__':
    atexit.register(app_cleanup)
    # 获取配置信息
    with open('config.json', 'r', encoding='utf-8') as file:
        config = json.load(file)
    height = config['camera']['height']
    width = config['camera']['width']
    shape = (height, width, 3)
    size = height * width * 3
    freq = config['app']['freq']
    python = config['app']['python']
    restart_time = config['app']['restart_time']

    # 定义进程间共享变量，存放图像数据
    shared_frame_array = multiprocessing.Array('B', size)
    # 定义互斥锁
    lock = multiprocessing.Lock()
    # 定义事件
    detect_event = multiprocessing.Event()
    capture_event = multiprocessing.Event()
    # 定义消息队列
    error_queue = multiprocessing.Queue(-1)
    socket_queue = multiprocessing.Queue(-1)
    log_queue = multiprocessing.Queue(-1)

    # 初始化logger
    queue_handler = logging.handlers.QueueHandler(log_queue)
    formatter = logging.Formatter('[%(asctime)s] - [%(name)s] - [%(levelname)s] - %(message)s')
    file_handler = logging.FileHandler('app.log')
    file_handler.setFormatter(formatter)
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger = logging.getLogger('app_logger')
    logger.setLevel(logging.INFO)
    logger.addHandler(queue_handler)
    listener = logging.handlers.QueueListener(log_queue, *[console_handler, file_handler])
    # 开启logger监听
    listener.start()

    # 打印app配置信息
    logger.debug(f'App config:\n{json.dumps(config, indent=4)}')

    # 初始化子进程
    detect_processing = multiprocessing.Process(target=detect,
                                                args=(capture_event, detect_event,
                                                      lock, config, shape, freq,
                                                      shared_frame_array,
                                                      socket_queue, error_queue))
    capture_processing = multiprocessing.Process(target=capture,
                                                args=(capture_event,
                                                      lock, config, shape, 
                                                      shared_frame_array,
                                                      error_queue))
    dosocket_processing = multiprocessing.Process(target=dosocket,
                                                args=(config, socket_queue, 
                                                      error_queue, detect_event))

    # 设置为守护进程，在主进程退出后子进程自动退出
    detect_processing.daemon = True
    capture_processing.daemon = True
    dosocket_processing.daemon = True
    # 开启进程
    dosocket_processing.start()
    time.sleep(0.5)
    detect_processing.start()
    time.sleep(0.5)
    capture_processing.start()


    # 统一进行异常处理
    while True:
        error = error_queue.get()
        if error[0] == 'dosocket':
            logger.error('failed in dosocket processing, suspend detect processing.')
            # 检测进程置为未启动
            detect_event.clear()
            if dosocket_processing.is_alive():
                dosocket_processing.terminate()
            dosocket_processing.join()
            # 尝试重新启动dosocket进程
            dosocket_processing = multiprocessing.Process(target=dosocket,
                                                          args=(config, socket_queue, 
                                                                error_queue, detect_event),
                                                          daemon=True)
            logger.info(f'try to restart dosocket processing after {restart_time} seconds...')
            time.sleep(restart_time)
            dosocket_processing.start()
            # sys.exit(-1)
        elif error[0] == 'capture':
            logger.error('failed in capture processing, suspend detect processing.')
            detect_event.clear()
            capture_processing.join()
            # 将异常消息加入socket_queue中
            response_data = {"status" : 'error', "message" : 'capture processing error.'}
            socket_queue.put(response_data)
            # 尝试重新启动摄像头进程
            capture_processing = multiprocessing.Process(target=capture,
                                                         args=(capture_event,
                                                               lock, config, shape,
                                                               shared_frame_array,
                                                               error_queue),
                                                          daemon=True)
            logger.info(f'try to restart capture processing after {restart_time} seconds...')
            time.sleep(restart_time)
            capture_processing.start()
        elif error[0] == 'detect':
            logger.error('failed in detect processing.')
            # 检测进程置为未启动
            detect_event.clear()
            # 等待子进程退出
            detect_processing.join()
            # 将异常消息加入socket_queue中
            response_data = {"status" : 'error', "message" : 'detect processing error.'}
            socket_queue.put(response_data)
            # 尝试重新启动检测进程
            detect_processing = multiprocessing.Process(target=detect,
                                                        args=(capture_event, 
                                                              detect_event, 
                                                              lock, config, shape, freq,
                                                              shared_frame_array,
                                                              socket_queue, error_queue),
                                                        daemon=True)
            logger.info(f'try to restart detect processing after {restart_time} seconds...')
            time.sleep(restart_time)
            detect_processing.start()
        else:
            logger.error('unknown error!')
            response_data = {"status" : 'error', "message" : 'unknown error.'}
            socket_queue.put(response_data)
            sys.exit(-1)