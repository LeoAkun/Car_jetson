from ultralytics import YOLO
# Load a model
model = YOLO("runs/detect/trafficLight/traffic_light.pt")  # 载入预训练模型
# Export the model
success = model.export(format="engine",imgsz=640, half=True)  # 导出静态ONNX模型，需设置batch参数注意opset为12/13
