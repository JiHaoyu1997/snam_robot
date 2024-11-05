import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image


class HSVSpace:
    
    def __init__(self, h_u, h_l, s_u, s_l, v_u, v_l) -> None:
        self._h_upper = h_u
        self._h_lower = h_l
        self._s_upper = s_u
        self._s_lower = s_l
        self._v_upper = v_u
        self._v_lower = v_l
    
    def _generate_lower_mask(self):
        return np.array([self._h_lower, self._s_lower, self._v_lower])
    
    def _generate_upper_mask(self):
        return np.array([self._h_upper, self._s_upper, self._v_upper])
    
    def apply_mask(self, hsv_image):
        _mask   = cv2.inRange(hsv_image, self._generate_lower_mask(), self._generate_upper_mask())
        _kernel = np.ones((9,9), np.uint8)
        _mask   = cv2.morphologyEx(_mask,cv2.MORPH_CLOSE, _kernel)
        return _mask

HSV_RANGES = {
    'pink':     HSVSpace(h_u=180, h_l=140, s_u=255, s_l=100, v_u=255, v_l=150),
    'yellow':   HSVSpace(h_u=30,  h_l=20,  s_u=255, s_l=100, v_u=255, v_l=150),
    'red':      HSVSpace(h_u=10,  h_l=0,   s_u=255, s_l=100, v_u=255, v_l=150),  # 或者 h_u=180, h_l=170
    'green':    HSVSpace(h_u=80,  h_l=50,  s_u=255, s_l=100, v_u=255, v_l=50),
    'blue':     HSVSpace(h_u=140, h_l=100, s_u=255, s_l=100, v_u=255, v_l=50),
    'white':    HSVSpace(h_u=180, h_l=0,   s_u=80,  s_l=0,   v_u=255, v_l=180),
    'purple':   HSVSpace(h_u=160, h_l=130, s_u=255, s_l=100, v_u=255, v_l=50),
    'orange':   HSVSpace(h_u=25,  h_l=10,  s_u=255, s_l=100, v_u=255, v_l=150),
}

def convert_raw_img_to_hsv_img(data: Image, cv_bridge: CvBridge):
    try:
        cv_img_raw = cv_bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)        

    # the top 1/3 part of image_raw for acc function
    acc_img = cv_img_raw[0 : int(cv_img_raw.shape[0]/3), :]

    # the bottom 3/4 part of image_raw for tracking function
    cv_img_raw2 = cv_img_raw[int(cv_img_raw.shape[0]/4) : cv_img_raw.shape[0], :]

    # Image Operation
    cv_img = adjust_gamma(cv_img=cv_img_raw2, gamma=0.5)

    # convert BGR image to HSV image
    acc_hsv_img = from_cv_to_hsv(acc_img)
    cv_hsv_img = from_cv_to_hsv(cv_img)

    return cv_img, cv_hsv_img, acc_hsv_img

def adjust_gamma(cv_img, gamma=1.0):
    invGamma = 1.0 / gamma
    lookup_table = np.array([ (i /255.0) ** invGamma * 255 for i in range(256)]).astype("uint8")
    return cv2.LUT(cv_img, lookup_table)

def from_cv_to_hsv(in_image):
    return cv2.cvtColor(in_image, cv2.COLOR_BGR2HSV)

def draw_test_mark_at_center(cv_pic):
    
    width_select    = int(cv_pic.shape[1]/2)
    height_select   = int(cv_pic.shape[0]/2)

    cv2.circle(cv_pic, (width_select ,height_select), 5, (0,0,255), 1)

    cv2.line(cv_pic,(width_select -10, height_select), (width_select  +10,height_select), (0,0,255), 1)
    cv2.line(cv_pic,(width_select , height_select-10), (width_select , height_select+10), (0,0,255), 1)
    
    return cv_pic  