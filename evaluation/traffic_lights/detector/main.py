from json import loads
from time import time
from cv2 import imread, imwrite
from evaluation.traffic_lights.detector.yolo import YOLO
from evaluation.traffic_lights.detector.postprocessing import draw_boxes
from evaluation.traffic_lights.detector.predict import predict_with_model_from_file
from os.path import dirname, join, abspath


def init_function():
    this_path = abspath(dirname(__file__))
    config_path = join(this_path, "config.json")
    with open(config_path) as config_buffer:
        config = loads(config_buffer.read())
    yolo = YOLO(config)
    yolo.model.load_weights(join(this_path, "checkpoints", "traffic-light-detection.h5"))
    return yolo, config


def main(image_path, config, model, bbox_path):
    boxes = predict_with_model_from_file(config, model, image_path)
    pplt_image = draw_boxes(imread(image_path), boxes, config['model']['classes'])
    imwrite(join(bbox_path, str(time()) + ".png"), pplt_image)
    if len(boxes) == 0:
        return "off"
    else:
        for box in boxes:
            label = config['model']['classes'][box.get_label()]
            if label == "go":
                return "green"
            elif label == "stop":
                return "red"
