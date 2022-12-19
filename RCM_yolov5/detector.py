import argparse
import os
import platform
import sys
from pathlib import Path

import numpy as np
import torch

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.common import DetectMultiBackend
from utils.augmentations import letterbox
from utils.general import (LOGGER, Profile, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_boxes, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, smart_inference_mode


class Detector:
    def __init__(self,
            weights=ROOT / 'trained_models/version1/best_50.pt',  # model path or triton URL
            data=ROOT / 'trained_models/version1/data.yaml',  # dataset.yaml path
            imgsz=(640, 640),  # inference size (height, width)
            conf_thres=0.25,  # confidence threshold
            iou_thres=0.45,  # NMS IOU threshold
            max_det=1000,  # maximum detections per image
            device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
            view_img=False,  # show results
            classes=None,  # filter by class: --class 0, or --class 0 2 3
            agnostic_nms=False,  # class-agnostic NMS
            augment=False,  # augmented inference
            line_thickness=3,  # bounding box thickness (pixels)
            half=False,  # use FP16 half-precision inference
            dnn=False,  # use OpenCV DNN for ONNX inference
    ):

        # Load model
        self.device = select_device(device)
        self.model = DetectMultiBackend(weights, device=self.device, dnn=dnn, data=data, fp16=half)
        self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
        self.imgsz = check_img_size(imgsz, s=self.stride)  # check image size

        # Dataloader
        self.bs = 1  # batch_size
        
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.classes = classes
        self.view_img = view_img
        self.agnostic_nms = agnostic_nms
        self.max_det = max_det
        self.augment = augment
        self.line_thickness = line_thickness
    
    @smart_inference_mode()
    def run(self, img):
        im0 = img.copy()
        # Run inference
        self.model.warmup(imgsz=(1 if self.pt or self.model.triton else self.bs, 3, *self.imgsz))  # warmup
        dt = (Profile(), Profile(), Profile())
        
        im = letterbox(im0, self.imgsz, stride=self.stride)[0]  # padded resize
        im = im.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        im = np.ascontiguousarray(im)  # contiguous
        
        with dt[0]:
            im = torch.from_numpy(im).to(self.model.device)
            im = im.half() if self.model.fp16 else im.float()  # uint8 to fp16/32
            im /= 255  # 0 - 255 to 0.0 - 1.0
            if len(im.shape) == 3:
                im = im[None]  # expand for batch dim
        
        # Inference
        with dt[1]:
            pred = self.model(im, augment=self.augment)

        # NMS
        with dt[2]:
            pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)

        # Second-stage classifier (optional)
        # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

        # Process predictions
        det = pred[0]

        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()
        
        # Print results
        # t = tuple(x.t * 1E3 for x in dt)  # speeds per image
        # LOGGER.info(f'Detect: {len(det)}, Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *self.imgsz)}' % t)
        
        if self.view_img:
            annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))
            
            for *xyxy, conf, cls in reversed(det):
                # Add bbox to image
                c = int(cls)  # integer class
                label = f'{self.names[c]} {conf:.2f}'
                annotator.box_label(xyxy, label, color=colors(c, True))
            
            # Stream results
            im0 = annotator.result()
            # cv2.imshow("img0", im0)
            # cv2.waitKey(0)  # 1 millisecond
        
            return det, im0
        
        return det
    

if __name__ == "__main__":
    detector = Detector(view_img=True, classes=0)
    img = cv2.imread(ROOT / "img0.jpg")
    det, im0 = detector.run(img)
    print(det)
    cv2.imshow("img0", im0)
    cv2.waitKey(0)  # 1 millisecond
