from utils.model.base import ModelBase
import numpy as np
import cv2
import torchvision.transforms as transforms
import logging


class ModelSample(ModelBase):

    def  __init__(self, modelcfg):
        super().__init__(modelcfg)

    def preprocess(self, frame):
        image = cv2.resize(frame, self.imgsz)   # to (640, 640, 3)
        image = image.transpose(2, 0, 1)        # to (3, 640, 640)
        image = np.expand_dims(image, axis=0)   # to (1, 3, 640, 640)
        return image
    
    def postprocess(self, output):
        result = []
        return result