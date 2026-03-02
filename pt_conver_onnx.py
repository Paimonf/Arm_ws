# 最简单的Python方式
from ultralytics import YOLO
model = YOLO('best.pt')
model.export(format='onnx')  # 不需要任何图片！