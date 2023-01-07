import os
import cv2
import time
import IPython
import scipy
import torch
import torchvision
import torch.backends.cudnn as cudnn
import numpy as np

from math import cos, sin
from hsemotion_onnx.facial_emotions import HSEmotionRecognizer as HSEmotionRecognizerONNX
from hsemotion.facial_emotions import HSEmotionRecognizer

from .MOS.models.MOS import MOS
from .MOS.config import cfg_mos_m, cfg_mos_s
from .MOS.utils.functions.prior_box import PriorBox
from .MOS.utils.nms.py_cpu_nms import py_cpu_nms
from .MOS.utils.box_utils import decode, decode_landm

class Pipeline:
    mos_cfg = {
        "trained_model": f"{os.getcwd()}/MOS_HSEmotion/MOS/test_weights/MOS-M.pth",
        "network": 'cfg_mos_m',
        "origin_size": True,
        "long_side": 840,
        "cpu": False,
        "confidence_threshold": 0.55,
        "top_k": 5000,
        "nms_threshold": 0.4,
        "keep_top_k": 750,
        "vis_thres": 0.55,
        "face2body_thres": 0.85
        }
    
    hsemotion_cfg = {
        "model_name": "enet_b2_8",
        "device": "cuda",
        "onnx": False,
        "idx_to_7class": {0: 'Anger', 1: 'Disgust', 2: 'Fear', 3: 'Happiness', 4: 'Neutral', 5: 'Sadness', 6: 'Surprise'},
        "idx_to_8class": {0: 'Anger', 1: 'Contempt', 2: 'Disgust', 3: 'Fear', 4: 'Happiness', 5: 'Neutral', 6: 'Sadness', 7: 'Surprise'},
    }

    attention_cfg = {
        "yaw": [-45, 45],
        "pitch": [-45, 45],
        "roll": [-45, 45]
    }
    def __init__(self):
        self.MOS_net = MOS(cfg=cfg_mos_m if self.mos_cfg['network'] == "cfg_mos_m" else cfg_mos_s, 
                           phase='test')
        self.MOS_net = load_model(self.MOS_net, 
                                  self.mos_cfg["trained_model"], 
                                  self.mos_cfg["cpu"])
        self.MOS_net.eval()
        print('Finished loading model!')
        # cudnn.benchmark = True
        self.MOS_net.device = torch.device("cpu" if self.mos_cfg["cpu"] else "cuda")
        self.MOS_net = self.MOS_net.to(self.MOS_net.device)

        if self.hsemotion_cfg["onnx"]:
            self.HSEmotion_net = HSEmotionRecognizerONNX(model_name=self.hsemotion_cfg["model_name"])
        else:
            self.HSEmotion_net = HSEmotionRecognizer(model_name="{}_best".format(self.hsemotion_cfg["model_name"]), 
                                                     device=self.hsemotion_cfg["device"])
    
    def infer(self, frame, target_box=None):
        if target_box is None: return None

        tic_mos_pre = time.time()
        device = self.MOS_net.device
        cfg = cfg_mos_m if self.mos_cfg['network'] == "cfg_mos_m" else cfg_mos_s
        img_raw = frame
        img = np.float32(img_raw)

        # testing scale
        target_size = self.mos_cfg["long_side"]
        max_size = self.mos_cfg["long_side"]
        im_shape = img.shape
        im_size_min = np.min(im_shape[0:2])
        im_size_max = np.max(im_shape[0:2])
        resize = float(target_size) / float(im_size_min)
        # prevent bigger axis from being more than max_size:
        if np.round(resize * im_size_max) > max_size:
            resize = float(max_size) / float(im_size_max)
        if self.mos_cfg["origin_size"]:
            resize = 1

        if resize != 1:
            img = cv2.resize(img, None, None, fx=resize, fy=resize, interpolation=cv2.INTER_LINEAR)

        img_rgb = img_raw.copy()
        im_height, im_width, _ = img.shape
        #print(im_height, im_width)

        scale = torch.Tensor([img.shape[1], img.shape[0], img.shape[1], img.shape[0]])
        img -= (104, 117, 123)
        img = img.transpose(2, 0, 1)
        img = torch.from_numpy(img).unsqueeze(0)
        img = img.to(device)
        scale = scale.to(device)
        toc_mos_pre = time.time()
        # print(f"MOS pre FPS = {1/(toc_mos_pre - tic_mos_pre)}")
        
        tic_mos_infer = time.time()
        loc, conf, landms, head_cls_y, head_cls_p, head_cls_r = self.MOS_net(img)
        toc_mos_infer = time.time()
        # print(f"MOS infer FPS = {1/(toc_mos_infer - tic_mos_infer)}")

        tic_mos_post = time.time()
        head_cls_y = head_cls_y.squeeze(0)
        head_cls_p = head_cls_p.squeeze(0)
        head_cls_r = head_cls_r.squeeze(0)
        idx_tensor = [idx for idx in range(66)]
        idx_tensor = torch.FloatTensor(idx_tensor).to(device)

        head_cls_y = torch.sum(head_cls_y * idx_tensor, 1).to(device) * 3 - 99
        head_cls_p = torch.sum(head_cls_p * idx_tensor, 1).to(device) * 3 - 99
        head_cls_r = torch.sum(head_cls_r * idx_tensor, 1).to(device) * 3 - 99

        priorbox = PriorBox(cfg, image_size=(im_height, im_width))
        priors = priorbox.forward()
        priors = priors.to(device)
        prior_data = priors.data
        boxes = decode(loc.data.squeeze(0), prior_data, cfg['variance'])
        boxes = boxes * scale / resize
        boxes = boxes.cpu().numpy()
        scores = conf.squeeze(0).data.cpu().numpy()[:, 1]
        landms = decode_landm(landms.data.squeeze(0), prior_data, cfg['variance'])

        head_cls_y = head_cls_y.data.cpu().numpy()
        head_cls_p = head_cls_p.data.cpu().numpy()
        head_cls_r = head_cls_r.data.cpu().numpy()

        scale1 = torch.Tensor([img.shape[3], img.shape[2], img.shape[3], img.shape[2],
                               img.shape[3], img.shape[2], img.shape[3], img.shape[2],
                               img.shape[3], img.shape[2]])
        scale1 = scale1.to(device)
        landms = landms * scale1 / resize
        landms = landms.cpu().numpy()

        # ignore low scores
        inds = np.where(scores > self.mos_cfg["confidence_threshold"])[0]
        boxes = boxes[inds]
        landms = landms[inds]
        scores = scores[inds]
        head_cls_y = head_cls_y[inds]
        head_cls_p = head_cls_p[inds]
        head_cls_r = head_cls_r[inds]

        # keep top-K before NMS
        order = scores.argsort()[::-1][:self.mos_cfg["top_k"]]
        boxes = boxes[order]
        landms = landms[order]
        scores = scores[order]
        head_cls_y = head_cls_y[order]
        head_cls_p = head_cls_p[order]
        head_cls_r = head_cls_r[order]
        idx_tensor = [idx for idx in range(66)]
        idx_tensor = np.array(idx_tensor)

        # do NMS
        dets = np.hstack((boxes, scores[:, np.newaxis])).astype(np.float32, copy=False)
        keep = py_cpu_nms(dets, self.mos_cfg["nms_threshold"])
        dets = dets[keep, :]
        landms = landms[keep]
        yaw_predicted = head_cls_y[keep]
        pitch_predicted = head_cls_p[keep]
        roll_predicted = head_cls_r[keep]

        dets = dets[:self.mos_cfg["keep_top_k"], :]
        landms = landms[:self.mos_cfg["keep_top_k"], :]
        yaw_predicted = yaw_predicted[:self.mos_cfg["keep_top_k"]]
        pitch_predicted = pitch_predicted[:self.mos_cfg["keep_top_k"]]
        roll_predicted = roll_predicted[:self.mos_cfg["keep_top_k"]]
        
        dets = np.concatenate((dets, landms, np.stack((yaw_predicted, pitch_predicted, roll_predicted), axis=1)), axis=1)
        toc_mos_post = time.time()
        # print(f"MOS post FPS = {1/(toc_mos_post - tic_mos_post)}")
        
        tic_mos_match = time.time()
        match_det = match(dets, target_box, self.mos_cfg["face2body_thres"])
        toc_mos_match = time.time()
        # print(f"MOS match FPS = {1/(toc_mos_match - tic_mos_match)}")

        if match_det is None: return {"attention": False, "emotion": None}

        tx1, ty1, tx2, ty2 = map(int, target_box)
        x1, y1, x2, y2 = map(int, match_det[:4])
        target_face_region = frame[y1:y2, x1:x2]
        tic_emo = time.time()
        emotion, scores = self.HSEmotion_net.predict_emotions(target_face_region, logits=True)
        toc_emo = time.time()
        # print(f"Emo FPS = {1/(toc_emo - tic_emo)}")

        tic_att = time.time()
        target_yaw, target_pitch, target_roll = match_det[-3:]
        if (self.attention_cfg["yaw"][0] <= target_yaw <= self.attention_cfg["yaw"][1]) and \
           (self.attention_cfg["pitch"][0] <= target_pitch <= self.attention_cfg["pitch"][1]) and \
           (self.attention_cfg["roll"][0] <= target_yaw <= self.attention_cfg["roll"][1]):
            attention = True
        else:
            attention = False
        toc_att = time.time()
        # print(f"Att FPS = {1/(toc_att - tic_att)}")

        tic_vis = time.time()
        out_frame = frame.copy()
        cv2.rectangle(out_frame, (x1, y1), (x2, y2), (0, 0, 255), 1)
        cv2.rectangle(out_frame, (tx1, ty1), (tx2, ty2), (255, 255, 255), 1)
        labels = [
            f"yaw: {int(target_yaw)}, pitch: {int(target_pitch)}, roll: {int(target_roll)}",
            f"emotion: {emotion}, attention: {attention}"
        ]
        background_width = 0
        background_height = 0
        for label in labels:
            (label_width, label_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            background_width = max(background_width, label_width)
            background_height += label_height + 2
        cv2.rectangle(out_frame, (x1, y1 - background_height), (x1 + background_width, y1), (255, 255, 255), -1)
        # Draw labels
        for i, label in enumerate(labels):
            position = (x1, y1 - i * (label_height + 3))
            cv2.putText(out_frame, label, position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        draw_axis(out_frame, int(target_yaw), int(target_pitch), int(target_roll), tdx=match_det[9],
                      tdy=match_det[10], size=30)
        cv2.imwrite("face.jpg", out_frame)
        toc_vis = time.time()
        # print(f"Vis FPS = {1/(toc_vis - tic_vis)}")
        return {"attention": attention, "emotion": emotion, "vis": out_frame}


def match(face_bboxs, ref_bbox, overlap_thres):
    if len(face_bboxs) == 0: return None
    fbs = torch.Tensor(face_bboxs[:, :4])
    b = torch.Tensor(ref_bbox).unsqueeze(0)
    inters, unions, b_area, fbs_area = box_inter_union(b, fbs)
    overlap_ratios = inters / fbs_area
    overlap_ratios = np.array(overlap_ratios).squeeze()
    mask = overlap_ratios > overlap_thres
    valid_face_bboxs = face_bboxs[mask]
    if len(valid_face_bboxs) == 0: return None
    elif len(valid_face_bboxs) == 1: return valid_face_bboxs.squeeze()
    else:
        valid_face_centers_x = (valid_face_bboxs[:, 0] + valid_face_bboxs[:, 2]) / 2
        valid_face_centers_y = (valid_face_bboxs[:, 1] + valid_face_bboxs[:, 3]) / 2
        valid_face_centers = np.stack([valid_face_centers_x, valid_face_centers_y], axis=1)
        ref_center = np.array([[(ref_bbox[0] + ref_bbox[2]) / 2, (ref_bbox[1] + ref_bbox[3]) / 2]])
        dist = scipy.spatial.distance.cdist(valid_face_centers, ref_center)
        return valid_face_bboxs[np.argmin(dist)]

def box_inter_union(boxes1, boxes2):
    area1 = torchvision.ops.boxes.box_area(boxes1)
    area2 = torchvision.ops.boxes.box_area(boxes2)

    lt = torch.max(boxes1[:, None, :2], boxes2[:, :2])  # [N,M,2]
    rb = torch.min(boxes1[:, None, 2:], boxes2[:, 2:])  # [N,M,2]

    wh = (rb - lt).clamp(min=0)  # [N,M,2]
    inter = wh[:, :, 0] * wh[:, :, 1]  # [N,M]
    union = (area1[:, None] + area2 - inter)
    return inter, union, area1, area2

def check_keys(model, pretrained_state_dict):
    ckpt_keys = set(pretrained_state_dict.keys())
    model_keys = set(model.state_dict().keys())
    used_pretrained_keys = model_keys & ckpt_keys
    unused_pretrained_keys = ckpt_keys - model_keys
    missing_keys = model_keys - ckpt_keys
    print('Missing keys:{}'.format(len(missing_keys)))
    print('Unused checkpoint keys:{}'.format(len(unused_pretrained_keys)))
    print('Used keys:{}'.format(len(used_pretrained_keys)))
    assert len(used_pretrained_keys) > 0, 'load NONE from pretrained checkpoint'
    return True

def remove_prefix(state_dict, prefix):
    ''' Old style model is stored with all names of parameters sharing common prefix 'module.' '''
    print('remove prefix \'{}\''.format(prefix))
    f = lambda x: x.split(prefix, 1)[-1] if x.startswith(prefix) else x
    return {f(key): value for key, value in state_dict.items()}


def load_model(model, pretrained_path, load_to_cpu):
    print('Loading pretrained model from {}'.format(pretrained_path))
    if load_to_cpu:
        pretrained_dict = torch.load(pretrained_path, map_location=lambda storage, loc: storage)
    else:
        device = torch.cuda.current_device()
        pretrained_dict = torch.load(pretrained_path, map_location=lambda storage, loc: storage.cuda(device))
    if "state_dict" in pretrained_dict.keys():
        pretrained_dict = remove_prefix(pretrained_dict['state_dict'], 'module.')
    else:
        pretrained_dict = remove_prefix(pretrained_dict, 'module.')
    check_keys(model, pretrained_dict)
    model.load_state_dict(pretrained_dict, strict=False)
    return model

def draw_axis(img, yaw, pitch, roll, tdx=None, tdy=None, size = 100):

    pitch = pitch * np.pi / 180
    yaw = -(yaw * np.pi / 180)
    roll = roll * np.pi / 180

    if tdx != None and tdy != None:
        tdx = tdx
        tdy = tdy
    else:
        height, width = img.shape[:2]
        tdx = width / 2
        tdy = height / 2

    # X-Axis pointing to right. drawn in red
    x1 = size * (cos(yaw) * cos(roll)) + tdx
    y1 = size * (cos(pitch) * sin(roll) + cos(roll) * sin(pitch) * sin(yaw)) + tdy

    # Y-Axis | drawn in green
    #        v
    x2 = size * (-cos(yaw) * sin(roll)) + tdx
    y2 = size * (cos(pitch) * cos(roll) - sin(pitch) * sin(yaw) * sin(roll)) + tdy

    # Z-Axis (out of the screen) drawn in blue
    x3 = size * (sin(yaw)) + tdx
    y3 = size * (-cos(yaw) * sin(pitch)) + tdy

    cv2.line(img, (int(tdx), int(tdy)), (int(x1),int(y1)),(0,0,255),3)
    cv2.line(img, (int(tdx), int(tdy)), (int(x2),int(y2)),(0,255,0),3)
    cv2.line(img, (int(tdx), int(tdy)), (int(x3),int(y3)),(255,0,0),4)

    return img   


    
