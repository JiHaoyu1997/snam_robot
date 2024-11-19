import os
import numpy as np
from typing import Union

from hsv.hsv import HSVSpace


script_dir  = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
filepath    = os.path.join(script_dir,'robot_setting')

if os.path.exists(filepath):
    from robot_setting_example import LEFT_TURN_L,LEFT_TURN_R,RIGHT_TURN_L,RIGHT_TURN_R,THUR_L,THUR_R,LANE_L,LANE_R
    print('Loading customize robot settings')
else:
    # default values
    LEFT_TURN_R     = 200 
    LEFT_TURN_L     = 60
    RIGHT_TURN_R    = 280
    RIGHT_TURN_L    = 120
    THUR_L          = 100
    THUR_R          = 220
    LANE_L          = 80
    LANE_R          = 240

def search_buffer_line(cv_hsv_img, buffer_line_hsv: HSVSpace) -> list:
    buffer_line_mask_img = buffer_line_hsv.apply_mask(cv_hsv_img)
    _height_center = int(buffer_line_mask_img.shape[0]/2)
    _line_center   = 0

    h = _height_center + 10
    h_list = []
    for i in range(0, 40, 10):
        
        if i == 0:
            h_list.append(h)
            continue
        
        j = 1 
        h_list.append(h + j * i)
        j = -j
        h_list.append(h + j * i)

    for h in h_list:
        # search this part of the picture
        _line = np.nonzero(buffer_line_mask_img[h, 40: ])[0]
        _line = _line + 40
        if len(_line) > 8 and len(_line) < 50:
            # there are proper amount of points at this part
            _line_center = int(np.mean(_line))        
            return _line_center, _height_center + i
    
    return None, None
    
def search_line(hsv_image, hsv_space: HSVSpace, top_line = 100) -> Union[int, float]:
    mask = hsv_space.apply_mask(hsv_image)
    lower_bound = int(hsv_image.shape[0]/2)
    upper_bound = 2 * lower_bound
    width_center = int(hsv_image.shape[1]/2)

    # point = np.nonzero(mask[lower_bound : upper_bound, width_center - 50])
    # return len(point[0])

    for i in range(-50, top_line, 25):
        point = np.nonzero(mask[lower_bound : upper_bound, width_center + i])
        # print(len(point[0]))
        if len(point[0]) > 5:
            return len(point[0])
        else:
            continue # no valid result found
    return 0

def _search_lane_linecenter(_mask, 
                            _lower_bias: int,
                            _upper_bias: int,
                            _interval: int,
                            _height_center: int,
                            _width_range_left: int,
                            _width_range_right: int,
                            _isYellow: bool = True,
                        ) -> int:
    
    for i in range(_lower_bias, _upper_bias, _interval):
        point = np.nonzero(_mask[_height_center + i, _width_range_left : _width_range_right])[0] + _width_range_left
        segs = _break_segs(point)
        valid_segments = {key: seg for key, seg in segs.items() if len(seg) < 35}
        # print(f"valid_segments: {valid_segments}, {_isYellow}")
        res = None

        if len(valid_segments) == 0:
            continue

        if len(valid_segments) == 1:
            res = int(np.mean(next(iter(valid_segments.values()), [])))  # 获取第一个值或空列表
            print(_height_center + i, res, 1, _isYellow)
            return res

        averages = {key: int(np.mean(seg)) for key, seg in valid_segments.items()}
        sorted_segments = sorted(averages.items(), key=lambda x: -len(valid_segments[x[0]]))
        
        res = next(
            (avg for key, avg in sorted_segments if (_isYellow and 40 < avg < 160) or (not _isYellow and avg > 160)),
            None
        )
        print(_height_center + i, res, 2, _isYellow)
        return res if res is not None else 0
    return 0 

def search_lane_center(space1:HSVSpace, space2:HSVSpace, hsv_image, is_yellow_left:bool) -> int:
    hsv_image1 = hsv_image
    hsv_image2 = hsv_image
    mask1 = space1.apply_mask(hsv_image1)
    mask2 = space2.apply_mask(hsv_image2)
    _line_center1 = _search_lane_linecenter(mask1, -20, 50, 10, int(hsv_image.shape[0]/2), 0, int(hsv_image.shape[1]))

    if _line_center1 == 0 and not is_yellow_left:
        # failed to find the center yellow line
        _line_center1 = hsv_image.shape[1]
        
    # search the while line, but limited area
    if is_yellow_left:
        _line_center2 = _search_lane_linecenter(mask2, -20, 50, 10, int(hsv_image.shape[0]/2), _line_center1, int(hsv_image.shape[1]), False)
    else:
        _line_center2 = _search_lane_linecenter(mask2, -20, 50, 10, int(hsv_image.shape[0]/2), 0 ,_line_center1, False)

    if is_yellow_left :
        # miss detections 
        if _line_center1 == 0:
            _line_center1 = hsv_image.shape[1] / 4
        if _line_center2 == 0:
            _line_center2 = hsv_image.shape[1] * 3 / 4

    # print(_line_center1, _line_center2)
    _lane_center = int((_line_center1 + _line_center2)/2)
    return max(min(_lane_center,LANE_R),LANE_L)

def search_inter_guide_line(hsv_space:HSVSpace,hsv_image,action:int):
    mask = hsv_space.apply_mask(hsv_image)
    
    # handle the crossing (X) patterns on the guide lines
    height = int(hsv_image.shape[0])
    x_list = []

    if action == 1:
        # left turn
        for i in range(80,170,20):
            y = height - i
            line = np.nonzero(mask[y,:])[0]
            # print(y,line)
            seg = _break_segs(line)

            if len(seg) == 0:
                continue        
            
            x =  np.mean(seg[0])
            x_list.append(x)
        #     y_list.append(y)
        if len(x_list) > 0:
            return max(LEFT_TURN_L,min(int(np.mean(x_list)),LEFT_TURN_R))
        else:
            return None
        
    elif action == 2:
        # right turn
        for i in range(70,130,15):
            y = i
            line = np.nonzero(mask[y,:])[0]
            seg = _break_segs(line)
            if len(seg) == 1:
                return min(max(RIGHT_TURN_L,int(np.mean(seg[0]))),RIGHT_TURN_R)
        return None
    
    else: # thur
        width = hsv_image.shape[1]
        y_hl = []
        for x in range(5,width,50):
            y_keep_out_high = height
            y_keep_out_low = 0
            vline = np.nonzero(mask[:,x])[0]
            if len(vline) > 5 and len(vline) < 80:
                y_hl.append(np.mean(vline))
            elif len(vline) >= 80:
                return x
        if len(y_hl) >=5 :
            # there is a front line here
            y_keep_out_high = min(y_hl)
            y_keep_out_low = max(y_hl)
        for i in range(30,120,15):
            y = height - i
            if y < y_keep_out_high or y > y_keep_out_low:
                line = np.nonzero(mask[y,:])[0]
                seg = _break_segs(line)
                if len(seg) == 1:
                    return max(min(int(np.mean(seg[0])),THUR_R),THUR_L)
        return None

def search_inter_guide_line2(hsv_space: HSVSpace, hsv_image, action: int, recursion_depth=0):
    # handle the crossing (X) patterns on the guide lines
    # x_list = []
    mask = hsv_space.apply_mask(hsv_image)
    width  = int(hsv_image.shape[1])
    height = int(hsv_image.shape[0])

    if recursion_depth > 1:
        return None

    if action == 1:
        # left turn
        res = None
        for i in range(50, 130, 20):
            y = height - i

            if recursion_depth == 0:
                line1 = np.nonzero(mask[y, 40 : 280])[0]
                line2 = np.nonzero(mask[y - 20, 40 : 280])[0]
                line1 = line1 + 40
                line2 = line2 + 40

            else: 
                line1 = np.nonzero(mask[y, : ])[0]
                line2 = np.nonzero(mask[y-20, : ])[0]

            seg1 = _break_segs(line1)
            seg2 = _break_segs(line2)

            if len(seg2) == 0:
                # the further line missing
                if len(seg1) == 0:
                    res =  None
                else:
                    res = int(np.mean(seg1[0]))
            
            elif len(seg2) == 1:
                # one line in front
                if len(seg1) == 1:
                    res = int(np.mean(seg1[0]))
                if len(seg1) > 1:
                    res = int(np.mean(seg2[0]))
                
            elif len(seg2) == 2:
                if len(seg1) == 1:
                    mean0 = np.mean(seg2[0])
                    mean1 = np.mean(seg2[1])
                    if 40 < mean0 < 280 and 40 < mean1 < 280:
                        dist0 = abs(mean0 - width // 3)
                        dist1 = abs(mean1 - width // 3)
                        if dist0 < dist1:
                            res = int(np.mean(seg2[0]))
                        else:
                            res = int(np.mean(seg2[1]))
                    elif 40 < mean0 < 280:
                        res = int(np.mean(seg2[0]))
                    elif 40 < mean1 < 280:
                        res = int(np.mean(seg2[1]))                 

                elif len(seg1) == 2:
                    n1 = abs(np.mean(seg1[0]) - np.mean(seg1[1]))
                    n2 = abs(np.mean(seg2[0]) - np.mean(seg2[1]))
                    if n1 > n2:
                        # closer gap is bigger
                        res = int(np.mean(seg1[1])) # follow lines on the right
                    else:
                        res = int(np.mean(seg1[0]))
                else:
                    res = int(np.mean(seg2[0]))
            
            elif len(seg2) > 2:
                new_seg2 = { k: v for k, v in seg2.items()if 40 < np.mean(v) < 280 }

                if len(new_seg2) == 0:
                    res = None
                
                elif len(new_seg2) == 1:
                    res = int() 

                avg_positions = [np.mean(segment) for segment in seg2.values() if 40 < np.mean(segment) < 280]
                avg_pos = int(np.mean(avg_positions))
                # print(avg_pos, recursion_depth)
                if avg_pos < width // 2:
                    cropped_hsv_image = hsv_image[ : , : width // 2]
                    # print('Left Half', recursion_depth)
                    res = search_inter_guide_line2(hsv_space=hsv_space, hsv_image=cropped_hsv_image, action=action, recursion_depth=recursion_depth+1)
                else:
                    cropped_hsv_image = hsv_image[ : , width //2 :]
                    # print('Right Half', recursion_depth)
                    res = search_inter_guide_line2(hsv_space=hsv_space, hsv_image=cropped_hsv_image, action=action, recursion_depth=recursion_depth+1)
                    if res is not None:
                        res += width // 2

            # print(y, len(seg1), len(seg2), res, recursion_depth)
            
            if res is None:
                return res
            
            if res > 250:
                return None
            
            if recursion_depth == 0:
                # print(seg2)
                res = max(min(res, LEFT_TURN_R), LEFT_TURN_L)
            
            return res    
          
    elif action == 2:
        # right turn
        for i in range(90, 140, 10):
            y = i
            line = np.nonzero(mask[y, 80 : ])[0]
            line = line + 80
            seg = _break_segs(line)
            # print(seg)
            if len(seg) == 1:
                result = min(max(RIGHT_TURN_L,int(np.mean(seg[0]))),RIGHT_TURN_R)
                # print(y ,result)
                return result
            
            elif len(seg) == 2:
                mean0 = np.mean(seg[0])
                mean1 = np.mean(seg[1])
                dist0 = abs(mean0 - 240)
                dist1 = abs(mean1 - 240)
                if dist0 < dist1:
                    res = int(np.mean(seg[0]))
                else:
                    res = int(np.mean(seg[1]))
                result = min(max(RIGHT_TURN_L, res), RIGHT_TURN_R)
                    
        return int(hsv_image.shape[1] * 0.83)
    
    else: 
        # thur
        width = hsv_image.shape[1]
        y_hl = []
        for x in range(5,width,50):
            y_keep_out_high = height
            y_keep_out_low = 0
            vline = np.nonzero(mask[:,x])[0]
            if len(vline) > 5 and len(vline) < 80:
                y_hl.append(np.mean(vline))
            elif len(vline) >= 80:
                return x
        if len(y_hl) >=5 :
            # there is a front line here
            y_keep_out_high = min(y_hl)
            y_keep_out_low = max(y_hl)
        for i in range(30,120,15):
            y = height - i
            if y < y_keep_out_high or y > y_keep_out_low:
                line = np.nonzero(mask[y,:])[0]
                seg = _break_segs(line)
                if len(seg) == 1:
                    return max(min(int(np.mean(seg[0])),THUR_R),THUR_L)
        return None      

def _break_segs(numbers: list,max_gap=5):
    
    segements = {}
    
    segement_number = 0
    current_segement = []
    
    for index, i in enumerate(numbers):
        
        if not current_segement:
            current_segement.append(i)
        else:
            if i <= numbers[index-1] + max_gap:
                # IN 
                current_segement.append(i)
            else:
                if len(current_segement) > 3:
                    segements[segement_number] = current_segement
                    segement_number += 1
                current_segement = [i]
    
    if len(current_segement) > 5:
        segements[segement_number] = current_segement
        
    return segements

def search_front_car(acc_image_hsv,hsv_space: HSVSpace):
    
    mask = hsv_space.apply_mask(acc_image_hsv)
    height = mask.shape[0]
    for i in range(5,60,5):
        y = height - i
        line = mask[y,:]
        point = np.nonzero(line)[0]
        
        if len(point) > 10:
            
            # there is a line
            center = int(np.mean(point))
            vline = np.nonzero(mask[:,center])
            dis_equiv = len(vline[0])
            return dis_equiv
        else:
            continue
        
    return None
        
