from utils.camera import GstreamerCamera
import json
import cv2
import argparse
import time
import os

parser = argparse.ArgumentParser(description='')
parser.add_argument('path', help='保存路径')
args = parser.parse_args()
if __name__ == "__main__":
    with open('config.json', 'r', encoding='utf-8') as file:
        config = json.load(file)
    pathdir = args.path
    if not os.path.exists(pathdir):
        os.makedirs(pathdir)
    camera = GstreamerCamera(config['camera'])
    step=0
    while True:
        frame = camera.read()
        cv2.imwrite(f'{pathdir}/image{step}.jpg', frame)
        step += 1
        time.sleep(0.2)


