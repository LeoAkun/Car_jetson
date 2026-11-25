from utils.model.base import ModelBase
import numpy as np
import cv2
import torchvision.transforms as transforms
import json
from ultralytics import YOLOv10
import torch


class Model(ModelBase):

    def  __init__(self, modelcfg):
        super().__init__(modelcfg)
        with open('class_config.json', 'r', encoding='utf-8') as file:
            self.classcfg = json.load(file)
        self.logger.debug(f'class config:\n{json.dumps(self.classcfg, indent=4)}')
        # x方向剪裁左右比例
        self.width_cut_rate = 0.20
        # y方向上方裁剪比例
        self.height_up_cut_rate = 0.10
        # y方向下方裁剪比例
        self.height_down_cut_rate = 0.30
        # 红绿灯状态保持
        self.light_status = ''
        # 绿灯计数器
        self.green_count = 0
        # 红灯计数器
        self.red_count = 0
        self.load()

        # 面积过滤阈值
        self.S_filter_thread = 0.1

    def load(self):
        self.model = YOLOv10(self.path, task='detect')
        # 提前载入模型到内存
        self.model(torch.zeros((1, 3, 640, 640)))

    def preprocess(self, frame):
        # image = cv2.resize(frame, self.imgsz)   # to (640, 640, 3)
        # image = image.transpose(2, 0, 1)        # to (3, 640, 640)
        # image = np.expand_dims(image, axis=0)   # to (1, 3, 640, 640)

        # 图像剪裁之前的宽度、高度
        self.before_cut_width = frame.shape[1]
        self.before_cut_height = frame.shape[0]
        # 裁剪宽度
        crop_width = int(self.before_cut_width * self.width_cut_rate)
        # 裁剪高度
        crop_height_up = int(self.before_cut_height * self.height_up_cut_rate)
        crop_height_down = int(self.before_cut_height * self.height_down_cut_rate)
        # 裁剪左右两边
        cropped_image = frame[crop_height_up:self.before_cut_height-crop_height_down, crop_width:self.before_cut_width-crop_width]

        return cropped_image

    def inference(self, image):
        return self.model(image, save=False, conf=self.conf, imgsz=self.imgsz)
    
    def postprocess(self, output):
        if (not output) or (output[0].boxes.shape[0]) == 0:
            return {'datas':[]}
            
        classes = {'0':'red','1':'green','2':'yellow'}

        # 计算剪裁后图像的宽度
        after_cut_width = self.before_cut_width * (1 - 2 * self.width_cut_rate)
        # 计算裁剪后图像的高度
        after_cut_height = self.before_cut_height * (1 - self.height_up_cut_rate - self.height_down_cut_rate)
       
        for i in output:       

            # 这个box就是剪裁后的
            # xyxy, 经过剪裁的xyxy
            boxes = i.boxes.xyxy.tolist()  
            confs = i.boxes.conf.cpu().tolist()
            cls_ids = i.boxes.cls.tolist()
            
            # 过滤低置信度目标
            valid_indices = [j for j, conf in enumerate(confs) if conf >= 0.5]
            if not valid_indices:
                continue
                
            # 处理有效目标
            output_after = []  # 保存剪裁之后的数据
            x_centers = []
            output_before = [] # 保存剪裁之前的数据
            for idx in valid_indices:
                # 归一化处理, 注意这里的boxes[idx][0]是绝对坐标，坐标系为模型的输入的图片，即剪裁后的图片
                x_min = boxes[idx][0] / after_cut_width # 剪裁后的归一化坐标x，坐标系为剪裁后的图片
                x_max = boxes[idx][2] / after_cut_width
                y_min = boxes[idx][1] / after_cut_height
                y_max = boxes[idx][3] / after_cut_height

                # 计算中心点
                x_center = round(abs(x_min + x_max) / 2, 6)
                x_centers.append(x_center)

                # 构建输出
                output_after.append([
                    classes[str(int(cls_ids[idx]))],
                    x_min * after_cut_width,
                    y_min * after_cut_height,
                    x_max * after_cut_width,
                    y_max * after_cut_height,
                    confs[idx]
                ])
                output_before.append([
                    classes[str(int(cls_ids[idx]))],
                    x_min * after_cut_width + self.before_cut_width*self.width_cut_rate, # 还原x_min像素至剪裁前
                    y_min * after_cut_height + self.before_cut_height * self.height_up_cut_rate, # 还原y_min像素至剪裁前
                    x_max * after_cut_width + self.before_cut_width*self.width_cut_rate, # 还原x_max像素
                    y_max * after_cut_height + self.before_cut_height * self.height_up_cut_rate, # 还原y_max像素至剪裁前
                    confs[idx]
                ])

            # 计算目标到中心的距离
            distances = [abs(x - 0.5) for x in x_centers]

            # 找到最接近中心的两个目标
            sorted_indices = sorted(range(len(distances)), key=lambda x: distances[x])
            if len(sorted_indices) >= 2: #如果有两个以上的目标
                idx1, idx2 = sorted_indices[0], sorted_indices[1]
                # 选择y坐标更大的目标
                j = idx1 if (output_after[idx1][2] + output_after[idx1][4])/2 > (output_after[idx2][2] + output_after[idx2][4])/2 else idx2 # 选择更靠下的
                # 排序输出
                output_before[0], output_before[j] = output_before[j], output_before[0]
                output_before[1:] = sorted(output_before[1:], key=lambda x: x[5], reverse=True) # 索引1后面的目标按置信度降序排序
            else: # 如果只有一个目标
                j = sorted_indices[0]
                output_before[0] = output_before[j]

            # 过滤掉面积过大的
            box_width = (output_before[0][3] - output_before[0][1]) / self.before_cut_width
            box_height =(output_before[0][4] - output_before[0][2]) / self.before_cut_height
            S = box_width * box_height
            if S > self.S_filter_thread: # 面积过大
                # print(f"[DEBG] S = {S} SO BIG!!!! \n")
                return {'datas':[]}

            # # 三帧确认
            # # 判断第一个元素是不是绿灯
            # ack_count = 3
            # # 如果结果非空
            # if len(output_before):
            #     if output_before[0][0] == 'green':
            #         # 检测到绿灯就将红灯计数值归零
            #         self.red_count = 0
            #         if self.light_status == 'red':
            #             self.green_count += 1
            #             # 如果当前是红灯，连续检测到的绿灯数量小于ack_count时，依旧视为红灯
            #             if self.green_count < ack_count:
            #                 output_before[0][0] = 'red'
            #             else:
            #                 self.light_status = 'green'
            #                 self.green_count = 0
            #         # 无状态时，直接确认
            #         elif self.light_status == '':
            #             self.light_status = 'green'
            #     # 检测到红灯就将绿灯计数值归零
            #     elif output_before[0][0] == 'red':
            #         self.green_count = 0
            #         if self.light_status == 'green':
            #             self.red_count += 1
            #             if self.red_count < ack_count:
            #                 output_before[0][0] = 'green'
            #             else:
            #                 self.light_status = 'red'
            #                 self.red_count = 0
            #         elif self.light_status == '':
            #             self.light_status = 'red'
            ack_count = 3
            if len(output_before):
                if output_before[0][0] == 'green':
                    # 检测到绿灯就将红灯计数值归零
                    self.red_count = 0
                    if self.light_status == 'red' or self.light_status == '':
                        self.green_count += 1
                        # 如果当前是红灯，连续检测到的绿灯数量小于ack_count时，依旧视为红灯
                        if self.green_count >= ack_count:
                            self.light_status = 'green'
                            self.green_count = 0
                # 检测到红灯就将绿灯计数值归零
                elif output_before[0][0] == 'red':
                    self.green_count = 0
                    if self.light_status == 'green' or self.light_status == '':
                        self.red_count += 1
                        if self.red_count >= ack_count:
                            self.light_status = 'red'
                            self.red_count = 0
            if self.light_status == '':
                return {'datas':[]}
            elif self.light_status == 'green':
                output_before[0][0] = 'green'
            elif self.light_status == 'red':
                output_before[0][0] = 'red'
            # 格式化输出，基于输入模型之前的原始图片坐标系的绝对坐标
            return {
                'datas': [{
                    'label': item[0],
                    'confidence': item[5],
                    'location': [item[1], item[2], item[3], item[4]]
                } for item in output_before]
            }