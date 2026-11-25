import tensorrt as trt
import numpy as np
import torch
import cv2
from abc import ABC, abstractmethod
import logging
import os


class ModelBase(ABC):
    
    def __init__(self, modelcfg):
        self.path = modelcfg["mpath"]
        self.input_name = modelcfg["input"]
        self.output_name = modelcfg["output"]
        self.model = None
        self.conf = modelcfg['conf']
        self.imgsz = modelcfg['imgsz']
        self.save = modelcfg['save']
        self.logger = logging.getLogger('app_logger')
    
    def __call__(self, *args, **kwargs):
        return self.predict(kwargs['frame'])

    @abstractmethod
    def load(self):
        # logger = trt.Logger(trt.Logger.INFO)
        # with open(path, "rb") as f, trt.Runtime(logger) as runtime:
        #     engine=runtime.deserialize_cuda_engine(f.read())
        # self.model = TRTModule(engine, input_names=[input_name], output_names=[output_name])
        # self.imgsz = (modelcfg["imgsz"], modelcfg["imgsz"])
        pass

    @abstractmethod
    def preprocess(self, frame):
        pass

    @abstractmethod
    def inference(self, image):
        # img_input = image.astype(np.float16)
        # img_input = torch.from_numpy(img_input)
        # img_input = img_input.to(0)             # to GPU0
        # output = self.model(img_input)
        # return output
        pass

    @abstractmethod
    def postprocess(self, output):
        pass

    # def tosave(self, image, result):
    #     if image is None:
    #         self.logger.warning("Your saving image is empty!")
    #         return
    #     height, width, channels = image.shape
    #     datas = result['datas']
    #     for data in datas:
    #         color = (0, 255, 0)
    #         if data['label'] == 'green':
    #             color = (0, 255, 0)
    #         elif data['label'] == 'red':
    #             color = (0, 0, 255)
    #         else:
    #             color = (0, 255, 255)
    #         start_point = (int(data['location'][0]), int(data['location'][1]))
    #         end_point = (int(data['location'][2]), int(data['location'][3]))
    #         thickness = 2
    #         cv2.rectangle(image, start_point, end_point, color, thickness)
    #         if datas[0] != None:
    #             text = f"{datas[0]['label']} {datas[0]['confidence']:.2f}"
    #             cv2.putText(image, text, (int(datas[0]['location'][0]), int(datas[0]['location'][1] - 10)), cv2.FONT_HERSHEY_SIMPLEX, 5, color, 10)

    #     resized_image = cv2.resize(image, (width // 4, height // 4))
    #     if not os.path.exists('./save'):
    #         os.makedirs('./save')
    #     cv2.imwrite('./save/output.jpg', resized_image)
    def tosave(self, image, output):
        if image is None:
            return
        height, width, channels = image.shape
        boxes = output[0].boxes.xyxy.tolist()
        confs = output[0].boxes.conf.cpu().tolist()
        cls_ids = output[0].boxes.cls.tolist()

        # 过滤低置信度目标
        valid_indices = [i for i, conf in enumerate(confs) if conf >= 0.5]
        if not valid_indices:
            return {'datas':[]}
        for idx in valid_indices:
            if int(cls_ids[idx]) == 0:
                color = (0, 0, 255)
            elif int(cls_ids[idx]) == 1:
                color = (0, 255, 0)
            else:
                color = (255, 0, 0)
            start_point = (int(boxes[idx][0]), int(boxes[idx][1]))
            end_point = (int(boxes[idx][2]), int(boxes[idx][3]))
            thickness = 2
            cv2.rectangle(image, start_point, end_point, color, thickness)
        resized_image = cv2.resize(image, (width // 4, height // 4))
        if not os.path.exists('./save'):
            os.makedirs('./save')
        cv2.imwrite('./save/output.jpg', resized_image)

    def predict(self, frame):
        image = self.preprocess(frame)
        output = self.inference(image)
        if self.save:
            self.tosave(image, output)
        result = self.postprocess(output)
        # if self.save:
        #     self.tosave(frame, result)
        return result
