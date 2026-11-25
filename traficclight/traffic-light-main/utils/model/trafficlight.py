from utils.model.base import ModelBase
import numpy as np
import cv2
import torchvision.transforms as transforms
import json
from ultralytics import YOLOv10
import torch


class TrafficLightModel(ModelBase):

    def  __init__(self, modelcfg):
        super().__init__(modelcfg)
        with open('class_config.json', 'r', encoding='utf-8') as file:
            self.classcfg = json.load(file)
        self.logger.debug(f'class config:\n{json.dumps(self.classcfg, indent=4)}')
        # x方向剪裁比例，左右各15%
        self.width_crop_rate = 0.15
        # 红绿灯状态保持
        self.light_status = ''
        # 绿灯计数器
        self.green_count = 0
        # 红灯计数器
        self.red_count = 0
        # 面积过滤阈值
        self.S_filter_thread = 0.08
        # 加载模型
        self.load()

    def load(self):
        self.model = YOLOv10(self.path, task='detect')
        # 提前载入模型到内存
        self.model(torch.zeros((1, 3, 640, 640)))

    def preprocess(self, frame):
        # image = cv2.resize(frame, self.imgsz)   # to (640, 640, 3)
        # image = image.transpose(2, 0, 1)        # to (3, 640, 640)
        # image = np.expand_dims(image, axis=0)   # to (1, 3, 640, 640)

        # 图像剪裁之前的宽度、高度
        self.img_width = frame.shape[1]
        self.img_height = frame.shape[0]
        # 裁剪宽度为图像宽度的 15%
        crop_width = int(self.img_width * self.width_crop_rate)
        # 裁剪左右两边
        cropped_image = frame[:, crop_width:self.img_width-crop_width]

        return cropped_image

    def inference(self, image):
        return self.model(image, save=False, conf=self.conf, imgsz=self.imgsz)

    def postprocess(self, output):
        if (not output) or (output[0].boxes.shape[0]) == 0:
            return {'datas':[]}
        classes = {'0':'red','1':'green','2':'yellow'}

        # 计算剪裁后图像的宽度
        cropped_width = self.img_width * (1 - 2 * self.width_crop_rate)

        # 这个box就是剪裁后的
        # xyxy ,经过剪裁的xyxy, 注意y没变, x变了
        boxes = output[0].boxes.xyxy.tolist()
        confs = output[0].boxes.conf.cpu().tolist()
        cls_ids = output[0].boxes.cls.tolist()

        # 过滤低置信度目标
        valid_indices = [i for i, conf in enumerate(confs) if conf >= 0.5]
        if not valid_indices:
            return {'datas':[]}
        # 处理有效目标
        x_centers = []
        cropped_output = []  # 保存剪裁之后的数据
        origin_output = [] # 保存剪裁之前的数据
        for idx in valid_indices:
            # 归一化处理, 注意这里的boxes[idx][0]是绝对坐标，坐标系为模型的输入的图片，即剪裁后的图片
            x_min = boxes[idx][0] / cropped_width # 剪裁后的归一化坐标x，坐标系为剪裁后的图片
            x_max = boxes[idx][2] / cropped_width
            y_min = boxes[idx][1]
            y_max = boxes[idx][3]

            # 计算中心点
            x_center = round((x_min + x_max) / 2)
            x_centers.append(x_center)

            # 构建输出
            cropped_output.append([
                classes[str(int(cls_ids[idx]))],
                x_min * cropped_width,
                y_min,
                x_max * cropped_width,
                y_max,
                confs[idx]
            ])
            origin_output.append([
                classes[str(int(cls_ids[idx]))],
                x_min * cropped_width + self.img_width*self.width_crop_rate, # 还原x_min像素至剪裁前
                y_min,
                x_max * cropped_width + self.img_width*self.width_crop_rate, # 还原x_max像素
                y_max,
                confs[idx]
            ])

        # 计算目标到中心的距离
        distances = [abs(x - 0.5) for x in x_centers]

        # 找到最接近中心的两个目标
        sorted_indices = sorted(range(len(distances)), key=lambda x: distances[x])
        if len(sorted_indices) >= 2: #如果有两个以上的目标
            idx1, idx2 = sorted_indices[0], sorted_indices[1]
            # 选择y坐标更大的目标
            selected_idx = idx1 if (cropped_output[idx1][2] + cropped_output[idx1][4])/2 > (cropped_output[idx2][2] + cropped_output[idx2][4])/2 else idx2 # 选择更靠下的
            # 排序输出
            origin_output[0], origin_output[selected_idx] = origin_output[selected_idx], origin_output[0]
            origin_output[1:] = sorted(origin_output[1:], key=lambda x: x[5], reverse=True) # 索引1后面的目标按置信度降序排序

        # 过滤掉面积过大的
        box_width = (origin_output[0][3] - origin_output[0][1]) / self.img_width
        box_height =(origin_output[0][4] - origin_output[0][2]) / self.img_height
        S = box_width * box_height
        if S > self.S_filter_thread: # 面积过大
            return {'datas':[]}

        # 三帧确认
        # 判断第一个元素是不是绿灯
        ack_count = 3
        # 如果结果非空
        if len(origin_output):
            if origin_output[0][0] == 'green':
                # 检测到绿灯就将红灯计数值归零
                self.red_count = 0
                if self.light_status == 'red' or self.light_status == '':
                    self.green_count += 1
                    # 如果当前是红灯，连续检测到的绿灯数量小于ack_count时，依旧视为红灯
                    if self.green_count >= ack_count:
                        self.light_status = 'green'
                        self.green_count = 0
            # 检测到红灯就将绿灯计数值归零
            elif origin_output[0][0] == 'red':
                self.green_count = 0
                if self.light_status == 'green' or self.light_status == '':
                    self.red_count += 1
                    if self.red_count >= ack_count:
                        self.light_status = 'red'
                        self.red_count = 0
        # 未确认状态返回空
        if self.light_status == '':
            return {'datas':[]}
        # 格式化输出，基于输入模型之前的原始图片坐标系的绝对坐标
        return {
            'datas': [{
                'label': self.light_status,
                'confidence': origin_output[0][5],
                'location': [origin_output[0][1], origin_output[0][2], origin_output[0][3], origin_output[0][4]]
            }].extend([{
                'label': item[0],
                'confidence': item[5],
                'location': [item[1], item[2], item[3], item[4]]
            } for item in origin_output[1:]])
        }