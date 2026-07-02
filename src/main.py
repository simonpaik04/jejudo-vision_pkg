#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from nav_msgs.msg import Path
from cv_bridge import CvBridge

import sys
import os
import rospkg
# Ensure we can import other files in the same directory
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from process import LaneProcessor
import config as cfg
import ros_utils
import config_manager

from dynamic_reconfigure.server import Server
from vision_pkg.cfg import VisionConfig

class LaneNode:
    def __init__(self):
        rospy.init_node("lane_node", anonymous=True)
        
        self.bridge = CvBridge()
        self.processor = LaneProcessor()

        # Dynamic Mode Loading (Sim vs Real)
        # real 모드: config.IMAGE_TOPIC 사용 (/image_raw 등). sim 모드만 토픽 고정.
        self.mode = rospy.get_param("/driving_mode", "real")  # Default to real if not found
        if self.mode == "sim":
            cfg.IMAGE_TOPIC = "/image_jpeg/compressed"
            cfg.K = None
            cfg.DIST_COEFF = np.zeros(5, dtype=np.float32)
        else:
            # real 모드: 캘리브레이션된 K, 왜곡계수 사용
            cfg.K = cfg.K_REAL
            cfg.DIST_COEFF = cfg.DIST_COEFF_REAL

        # Keep track of compressed vs raw image topics
        self.is_compressed = "compressed" in cfg.IMAGE_TOPIC

        # Dynamic Reconfigure Setup
        initial_config = {
            'camera_height': cfg.CAMERA_HEIGHT_M,
            'camera_pitch': cfg.CAMERA_PITCH_DEG,
            'camera_fov': cfg.CAMERA_FOV_DEG,
            'bev_length': cfg.BEV_LENGTH_M,
            'bev_width': cfg.BEV_WIDTH_M,
            'canny_low': cfg.CANNY_LOW,
            'canny_high': cfg.CANNY_HIGH,
            'n_windows': cfg.N_WINDOWS,
            'sw_margin': cfg.SW_MARGIN,
            'minpix': cfg.MINPIX,
            'minpix_bottom': cfg.MINPIX_BOTTOM,
            'max_gap_windows': cfg.MAX_GAP_WINDOWS,
            'sw_bottom_offset': cfg.SW_BOTTOM_OFFSET,
            'sw_curve_ratio': cfg.SW_CURVE_RATIO,
            'minlanepix': cfg.MINLANEPIX,
            'lane_width_px': cfg.DEFAULT_LANE_WIDTH_PX,
            'smooth_alpha': cfg.SMOOTH_ALPHA,
            'sw_pred_amp': cfg.SW_PRED_AMP,
            'poly_order': cfg.POLY_ORDER,
            'reset_interval': cfg.RESET_INTERVAL,
            'publish_debug_images': getattr(cfg, 'PUBLISH_DEBUG_IMAGES', True)
        }
        self.srv = Server(VisionConfig, self.dyn_callback)
        self.srv.update_configuration(initial_config)

        # Publishers
        self.pub_canny     = rospy.Publisher("/vision/canny", Image, queue_size=1)
        self.pub_bev       = rospy.Publisher("/vision/bev", Image, queue_size=1)
        self.pub_sw        = rospy.Publisher("/vision/sliding_window", Image, queue_size=1)
        self.pub_fit       = rospy.Publisher("/vision/lane_fitting", Image, queue_size=1)
        self.pub_final     = rospy.Publisher("/vision/final_result", Image, queue_size=1)
        
        self.pub_path       = rospy.Publisher("/vision/lane_path", Path, queue_size=1)
        self.pub_left_lane  = rospy.Publisher("/vision/left_lane", Path, queue_size=1)
        self.pub_right_lane = rospy.Publisher("/vision/right_lane", Path, queue_size=1)

        # Subscriber (cfg.IMAGE_TOPIC: sim=/image_jpeg/compressed, real=/usb_cam/image_raw)
        image_topic = cfg.IMAGE_TOPIC
        if self.is_compressed:
            rospy.Subscriber(image_topic, CompressedImage, self.callback, queue_size=1, buff_size=2**24)
        else:
            rospy.Subscriber(image_topic, Image, self.callback, queue_size=1, buff_size=2**24)
        
        rospy.loginfo(f"Lane Node Started. Subscribing to: {image_topic} (Compressed: {self.is_compressed})")

    def dyn_callback(self, config, level):
        # ... (lines 88-125 remain unchanged) ...
        # Physical Camera Parameters & ROI
        cfg.CAMERA_HEIGHT_M   = config.camera_height
        cfg.CAMERA_PITCH_DEG  = config.camera_pitch
        cfg.CAMERA_FOV_DEG    = config.camera_fov
        cfg.BEV_LENGTH_M      = config.bev_length
        cfg.BEV_WIDTH_M       = config.bev_width
        
        # Dynamically recalculate BEV pixel resolution based on physical size
        cfg.BEV_IMAGE_WIDTH   = int(cfg.BEV_WIDTH_M / cfg.M_PER_PIXEL)
        cfg.BEV_IMAGE_HEIGHT  = int(cfg.BEV_LENGTH_M / cfg.M_PER_PIXEL)

        # Canny
        cfg.CANNY_LOW         = config.canny_low
        cfg.CANNY_HIGH        = config.canny_high
        # Sliding Window
        cfg.N_WINDOWS         = config.n_windows
        cfg.SW_MARGIN         = config.sw_margin
        cfg.MINPIX            = config.minpix
        cfg.MINPIX_BOTTOM     = config.minpix_bottom
        cfg.MAX_GAP_WINDOWS   = config.max_gap_windows
        cfg.SW_BOTTOM_OFFSET  = config.sw_bottom_offset
        cfg.SW_CURVE_RATIO    = config.sw_curve_ratio
        cfg.MINLANEPIX        = config.minlanepix
        # Lane Geometry (Direct Pixel Values from RQT)
        cfg.DEFAULT_LANE_WIDTH_PX = config.lane_width_px
        cfg.SMOOTH_ALPHA          = config.smooth_alpha
        cfg.SW_PRED_AMP           = config.sw_pred_amp
        cfg.POLY_ORDER            = config.poly_order
        cfg.RESET_INTERVAL        = config.reset_interval
        cfg.PUBLISH_DEBUG_IMAGES  = config.publish_debug_images

        # Save to disk if requested
        if getattr(config, 'save_config', False):
            config_manager.save_config_to_files(config)
            config.save_config = False
            rospy.loginfo("Configuration saved to disk.")

        return config

    def callback(self, msg):
        try:
            if self.is_compressed:
                np_arr = np.frombuffer(msg.data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return

        # Process the frame through the vision pipeline
        results = self.processor.process(frame)
        
        # ALWAYS publish /lane_fitting for basic monitoring
        if results.get("vis") is not None:
            self.pub_fit.publish(self.bridge.cv2_to_imgmsg(results["vis"], "bgr8"))
        
        # Publish other Debug Images (Optional)
        if getattr(cfg, 'PUBLISH_DEBUG_IMAGES', True):
            if results.get("bev") is not None:
                self.pub_bev.publish(self.bridge.cv2_to_imgmsg(results["bev"], "mono8"))
            if results.get("final") is not None:
                self.pub_final.publish(self.bridge.cv2_to_imgmsg(results["final"], "bgr8"))
            if results.get("canny") is not None:
                self.pub_canny.publish(self.bridge.cv2_to_imgmsg(results["canny"], "mono8"))
            if results.get("sw_vis") is not None:
                self.pub_sw.publish(self.bridge.cv2_to_imgmsg(results["sw_vis"], "bgr8"))

        # Generate and Publish nav_msgs/Path
        pts_line = results.get("pts_line")
        pts_left = results.get("pts_left")
        pts_right = results.get("pts_right")

        if pts_line is not None and results.get("bev") is not None:
            h, w = results["bev"].shape[:2]
            self.pub_path.publish(ros_utils.create_path_msg(pts_line, (h, w)))
            self.pub_left_lane.publish(ros_utils.create_path_msg(pts_left, (h, w)))
            self.pub_right_lane.publish(ros_utils.create_path_msg(pts_right, (h, w)))
        else:
            # Publish empty paths
            for pub in [self.pub_path, self.pub_left_lane, self.pub_right_lane]:
                empty_path = Path()
                empty_path.header.stamp = rospy.Time.now()
                empty_path.header.frame_id = "stier"
                pub.publish(empty_path)
            rospy.logwarn_throttle(2.0, "No lane detected — publishing empty path")


    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = LaneNode()
    node.run()