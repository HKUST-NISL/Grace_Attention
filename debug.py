import time
import cv2
from MOS_HSEmotion import grace_emotion_attention
grace_emotion_attention_modules_pipepline = grace_emotion_attention.Pipeline()
for _ in range(1000):
    input = cv2.imread("test.jpg")
    target_box = [123.46533, 34.51011, 170.13535, 120.95128]
    # target_box = [123.46533, 34.51011, 394.13535, 120.95128]
    tic = time.time()
    res = grace_emotion_attention_modules_pipepline.infer(input, target_box)
    toc = time.time()
    print(f"Overall FPS = {1/(toc-tic)}")
    print(f"===========================")