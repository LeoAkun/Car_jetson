import cv2
import pyudev
import os

def get_video_device_by_vid_pid(target_vid, target_pid):
    """
    根据VID和PID查找摄像头对应的/dev/videoX设备节点
    :param target_vid: 摄像头厂商ID（16进制字符串，如"1bcf"）
    :param target_pid: 摄像头产品ID（16进制字符串，如"0b09"）
    :return: 视频设备路径（如"/dev/video0"），未找到返回None
    """
    # 初始化pyudev上下文
    context = pyudev.Context()
    # 遍历所有视频设备
    for device in context.list_devices(subsystem='video4linux'):
        # 获取设备的VID和PID（16进制，小写）
        vid = device.get('ID_VENDOR_ID', '').lower()
        pid = device.get('ID_MODEL_ID', '').lower()
        
        # 匹配目标VID/PID
        if vid == target_vid.lower() and pid == target_pid.lower():
            # 返回视频设备路径（如/dev/video0）
            return device.device_node
    return None

def read_camera(video_device, width=640, height=480, fps=30):
    """
    读取指定视频设备的摄像头画面
    :param video_device: 视频设备路径（如/dev/video0）
    :param width: 画面宽度
    :param height: 画面高度
    :param fps: 帧率
    """
    if not os.path.exists(video_device):
        print(f"错误：设备 {video_device} 不存在！")
        return

    # 打开摄像头
    cap = cv2.VideoCapture(video_device)
    if not cap.isOpened():
        print(f"错误：无法打开摄像头 {video_device}！")
        return

    # 设置摄像头参数
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    print(f"成功打开摄像头：{video_device}")
    print("按 'q' 键退出画面预览")

    # 循环读取画面
    while True:
        ret, frame = cap.read()
        if not ret:
            print("警告：无法读取摄像头画面，可能已断开连接")
            break

        # 显示画面
        cv2.imshow('Camera Preview', frame)

        # 按q键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # ========== 配置参数（替换为你的摄像头VID/PID） ==========
    CAMERA_VID = "1bcf"   # 摄像头厂商ID（从lsusb获取）
    CAMERA_PID = "0001"   # 摄像头产品ID（从lsusb获取）
    # ========== 可选：调整画面参数 ==========
    CAMERA_WIDTH = 1920   # 画面宽度
    CAMERA_HEIGHT = 1080   # 画面高度
    CAMERA_FPS = 60       # 帧率

    # 1. 根据VID/PID查找摄像头设备
    video_dev = get_video_device_by_vid_pid(CAMERA_VID, CAMERA_PID)
    if not video_dev:
        print(f"错误：未找到VID={CAMERA_VID} PID={CAMERA_PID}的摄像头设备！")
        print("请检查：1.摄像头是否已连接 2.VID/PID是否正确 3.摄像头是否被其他程序占用")
    else:
        # 2. 读取摄像头画面
        read_camera(video_dev, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS)
