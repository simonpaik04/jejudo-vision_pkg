import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import config as cfg

def create_path_msg(pts_line, bev_shape):
    """
    Convert image-space points (pts_line) into a real-world BEV coordinate system Path message.
    :param pts_line: numpy array of shape (N, 1, 2) representing the lane centerline in pixels
    :param bev_shape: tuple of (height, width) of the BEV image
    :return: nav_msgs/Path message
    """
    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = "stier"
    
    if pts_line is None or len(pts_line) == 0:
        return path_msg

    h, w = bev_shape[:2]
    pts = np.asarray(pts_line).reshape(-1, 2)
    
    for pt in pts:
        px = float(pt[0])
        py = float(pt[1])
        
        # In BEV, the vehicle is at the bottom center of the image.
        # x-axis (forward) = bottom of image (h) going up to top (0)
        # y-axis (left) = center of image (w/2) going left to right (w)
        local_x_px = float(h - py)
        local_y_px = float((w / 2.0) - px)
        
        local_x = local_x_px * cfg.M_PER_PIXEL
        local_y = local_y_px * cfg.M_PER_PIXEL
        
        pose = PoseStamped()
        pose.header = path_msg.header
        pose.pose.position.x = local_x
        pose.pose.position.y = local_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0 # No rotation
        path_msg.poses.append(pose)
        
    return path_msg
