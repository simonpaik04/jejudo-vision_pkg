import os
import re
import rospy
import rospkg
import config as cfg

def save_config_to_files(rqt_config):
    """
    현재 Dynamic Reconfigure(rqt_reconfigure) 설정을 config.py와 Vision.cfg 파일에 저장합니다.
    :param rqt_config: rqt 콜백으로부터 전달받은 현재 설정 객체
    """
    rospack = rospkg.RosPack()
    try:
        pkg_path = rospack.get_path('vision_pkg')
    except Exception as e:
        rospy.logerr(f"Could not find vision_pkg path: {e}")
        return
    
    # 1. config.py 파일 업데이트
    config_path = os.path.join(pkg_path, "src", "config.py")
    
    # Physical Camera Parameters & ROI
    save_map_py = {
        "CAMERA_HEIGHT_M":  round(rqt_config.camera_height, 3),
        "CAMERA_PITCH_DEG": round(rqt_config.camera_pitch, 3),
        "BEV_LENGTH_M"   :  round(rqt_config.bev_length, 3),
        "BEV_WIDTH_M"    :  round(rqt_config.bev_width, 3),
        "CANNY_LOW":          rqt_config.canny_low,
        "CANNY_HIGH":         rqt_config.canny_high,
        "N_WINDOWS":          rqt_config.n_windows,
        "SW_MARGIN":          rqt_config.sw_margin,
        "MINPIX":             rqt_config.minpix,
        "MINPIX_BOTTOM":      rqt_config.minpix_bottom,
        "MAX_GAP_WINDOWS":    rqt_config.max_gap_windows,
        "SW_BOTTOM_OFFSET":   rqt_config.sw_bottom_offset,
        "SW_CURVE_RATIO":     rqt_config.sw_curve_ratio,
        "SW_PRED_AMP":        rqt_config.sw_pred_amp,
        "DEFAULT_LANE_WIDTH_PX": rqt_config.lane_width_px,
        "MIN_LANE_WIDTH_PX":     rqt_config.min_lane_width_px,
        "MINLANEPIX":         rqt_config.minlanepix,
        "SMOOTH_ALPHA":       rqt_config.smooth_alpha,
        "POLY_ORDER":         rqt_config.poly_order,
        "PUBLISH_DEBUG_IMAGES": rqt_config.publish_debug_images
    }
    
    # RQT에서 입력된 차선 폭(픽셀)을 그대로 저장합니다 (ROI에 따라 가변적)
    save_map_py["DEFAULT_LANE_WIDTH_PX"] = rqt_config.lane_width_px

    _regex_replace_file(
        config_path, 
        save_map_py, 
        lambda k, v: (r"(^|\n)" + re.escape(k) + r"\s*=\s*[^\n]*", f"\\1{k} = {repr(v)}")
    )

    # 2. Vision.cfg (실제 RQT 슬라이더 기본값) 파일 업데이트
    cfg_path = os.path.join(pkg_path, "cfg", "Vision.cfg")
    
    save_map_cfg = {
        "camera_height":      round(rqt_config.camera_height, 3),
        "camera_pitch":       round(rqt_config.camera_pitch, 3),
        "camera_fov":         rqt_config.camera_fov,
        "bev_length":         round(rqt_config.bev_length, 3),
        "bev_width":          round(rqt_config.bev_width, 3),
        "canny_low":          rqt_config.canny_low,
        "canny_high":         rqt_config.canny_high,
        "n_windows":          rqt_config.n_windows,
        "sw_margin":          rqt_config.sw_margin,
        "minpix":             rqt_config.minpix,
        "minpix_bottom":      rqt_config.minpix_bottom,
        "max_gap_windows":    rqt_config.max_gap_windows,
        "sw_bottom_offset":   rqt_config.sw_bottom_offset,
        "sw_curve_ratio":     rqt_config.sw_curve_ratio,
        "minlanepix":         rqt_config.minlanepix,
        "lane_width_px":      rqt_config.lane_width_px,
        "smooth_alpha":       rqt_config.smooth_alpha,
        "poly_order":         rqt_config.poly_order,
        "publish_debug_images": rqt_config.publish_debug_images,
    }

    _regex_replace_file(
        cfg_path, 
        save_map_cfg, 
        lambda k, v: (r"(gen\.add\(\s*[\"\']" + re.escape(k) + r"[\"\']\s*,[^\n,]*,[^\n,]*,[^\n,]*,)\s*([^\n,)]*)([^\n]*)", r"\g<1> " + str(v) + r"\g<3>")
    )
    
    # cfg 파일에 실행 권한 부여
    os.system(f"chmod +x {cfg_path}")

def _regex_replace_file(filepath, values_map, pattern_replacement_func):
    try:
        with open(filepath, "r") as f:
            content = f.read()
            
        for key, val in values_map.items():
            pattern, replacement = pattern_replacement_func(key, val)
            content = re.sub(pattern, replacement, content)
            
        with open(filepath, "w") as f:
            f.write(content)
    except Exception as e:
        rospy.logerr(f"Failed to update file {filepath}: {e}")
