import numpy as np
import cv2
import config as cfg

# ============================================================
# Preprocessing
# ============================================================

def to_gray(frame):
    return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

def canny_edge(img, low_threshold=50, high_threshold=100):
    return cv2.Canny(img, low_threshold, high_threshold)

def bilateralmsg(img):
    return cv2.bilateralFilter(img, d=7, sigmaColor=30, sigmaSpace=30)

def compute_ipm_matrix(w, h):
    """
    Computes the Inverse Perspective Mapping (IPM) Homography matrix using 
    physical Extrinsic and Intrinsic parameters.
    """
    # 1. Compute Intrinsic Matrix K
    if getattr(cfg, 'K', None) is not None:
        K = cfg.K.astype(np.float32)
    else:
        # Default FOV-based calculation (Sim)
        fov_rad = np.deg2rad(cfg.CAMERA_FOV_DEG)
        fx = (w / 2.0) / np.tan(fov_rad / 2.0)
        fy = fx
        K = np.array([
            [fx, 0.0, cfg.CENTER_X],
            [0.0, fy, cfg.CENTER_Y],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)

    pitch_rad = np.deg2rad(cfg.CAMERA_PITCH_DEG)
    
    # Camera to World Rotation (Pitch down is positive rotation about X)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(pitch_rad), -np.sin(pitch_rad)],
        [0, np.sin(pitch_rad),  np.cos(pitch_rad)]
    ], dtype=np.float32)

    # Translation to ground (Camera is at H aloft, so ground is Y = -H)
    C = np.array([[0], [-cfg.CAMERA_HEIGHT_M], [0]], dtype=np.float32)
    T = -Rx @ C

    RT = np.hstack((Rx, T))
    P = K @ RT

    bev_w = cfg.BEV_IMAGE_WIDTH
    bev_h = cfg.BEV_IMAGE_HEIGHT
    
    # We define 4 keypoints on the ground in real-world meters
    # The BEV image represents [0, BEV_LENGTH_M] in Z, and [-W/2, W/2] in X.
    # Pixel (u, v) in BEV maps to:
    # X_world = (u - bev_w/2) * M_PER_PIXEL
    # Z_world = (bev_h - v) * M_PER_PIXEL
    # We will pick the 4 corners of the BEV image.
    
    # dst_pts (BEV Pixel Coordinates): Top-Left, Top-Right, Bottom-Right, Bottom-Left
    dst_pts = np.array([
        [0,         0],
        [bev_w - 1, 0],
        [bev_w - 1, bev_h - 1],
        [0,         bev_h - 1]
    ], dtype=np.float32)
    
    # Find matching src_pts (Camera Image Coordinates) by projecting the physical 3D world corners
    src_pts = []
    for (u_bev, v_bev) in dst_pts:
        # Convert BEV pixel to World coordinate
        X_w = (u_bev - bev_w/2.0) * cfg.M_PER_PIXEL
        Z_w = (bev_h - 1- v_bev) * cfg.M_PER_PIXEL
        Y_w = 0.0 # Ground plane
        
        # Project 3D point to camera 2D image plane
        world_pt = np.array([[X_w], [Y_w], [Z_w], [1.0]], dtype=np.float32)
        uvw = P @ world_pt
        
        if uvw[2, 0] <= 0:
            return None, None  # 또는 raise
        else:
            u_cam = uvw[0, 0] / uvw[2, 0]
            v_cam = uvw[1, 0] / uvw[2, 0]
            
        src_pts.append([u_cam, v_cam])

    src_pts = np.array(src_pts, dtype=np.float32)

    M    = cv2.getPerspectiveTransform(src_pts, dst_pts)
    Minv = cv2.getPerspectiveTransform(dst_pts, src_pts)
    
    return M, Minv

def birds_eye_view(img, pad=0):
    """
    Applies the mathematical IPM transformation directly to the image.
    """
    h, w = img.shape[:2]
    
    # Compute mathematically accurate homography
    M, Minv = compute_ipm_matrix(w, h)
    
    warped = cv2.warpPerspective(
        img, M, (cfg.BEV_IMAGE_WIDTH, cfg.BEV_IMAGE_HEIGHT),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0
    )

    if pad and pad > 0:
        warped[:pad, :]  = 0
        warped[-pad:, :] = 0
        warped[:, :pad]  = 0
        warped[:, -pad:] = 0

    return warped, M, Minv


def compute_bev_black_rows(w_img, h_img):
    """
    BEV 하단에서 카메라 FOV 밖인 검은 행 수를 기하학적으로 계산.
    카메라 중심축(X_w=0) 기준으로, 아래에서 올라가며
    처음으로 카메라 프레임 안에 들어오는 BEV 행을 찾는다.
    """
    bev_h = cfg.BEV_IMAGE_HEIGHT

    # ── K 매트릭스 (compute_ipm_matrix 와 동일 로직) ──
    if getattr(cfg, 'K', None) is not None:
        K = cfg.K.astype(np.float32)
    else:
        fov_rad = np.deg2rad(cfg.CAMERA_FOV_DEG)
        fx = (w_img / 2.0) / np.tan(fov_rad / 2.0)
        fy = fx
        K = np.array([
            [fx, 0.0, cfg.CENTER_X],
            [0.0, fy, cfg.CENTER_Y],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)

    pitch_rad = np.deg2rad(cfg.CAMERA_PITCH_DEG)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(pitch_rad), -np.sin(pitch_rad)],
        [0, np.sin(pitch_rad),  np.cos(pitch_rad)]
    ], dtype=np.float32)
    C = np.array([[0], [-cfg.CAMERA_HEIGHT_M], [0]], dtype=np.float32)
    T = -Rx @ C
    RT = np.hstack((Rx, T))
    P = K @ RT

    # 아래에서 위로 스캔하며 첫 유효 행 탐색
    for v_bev in range(bev_h - 1, -1, -1):
        Z_w = (bev_h - 1 - v_bev) * cfg.M_PER_PIXEL
        world_pt = np.array([[0], [0], [Z_w], [1]], dtype=np.float32)
        uvw = P @ world_pt
        if uvw[2, 0] <= 0:
            continue
        v_cam = uvw[1, 0] / uvw[2, 0]
        u_cam = uvw[0, 0] / uvw[2, 0]
        if 0 <= u_cam < w_img and 0 <= v_cam < h_img:
            return bev_h - 1 - v_bev   # 검은 행 수
    return 0

# ============================================================
# Visualization / Postprocessing
# ============================================================

def draw_lane_on_bev(roi_img, left_fit, right_fit, ptsC=None, l_yr=None, r_yr=None,
                      left_color=(255,0,0), right_color=(0,0,255), line_thickness=3,
                      center_color=(0,255,0), center_thickness=2,
                      center_radius=3):
    
    if roi_img is None:
        return None

    if roi_img.ndim == 2:
        vis = cv2.cvtColor(roi_img, cv2.COLOR_GRAY2BGR)
    else:
        vis = roi_img.copy()

    h, w = vis.shape[:2]

    # Draw Lines
    def draw_fit(fit, color, yr):
        if fit is None: return
        f = np.asarray(fit).reshape(-1)
        
        y_min = 0
        y_max = h - 1
        if yr is not None:
            y_min = max(0, int(yr[0]))
            y_max = min(h - 1, int(yr[1]))
            
        ploty_fit = np.arange(y_max, max(-1, y_min - 1), -1, dtype=np.float32)
        if len(ploty_fit) < 2: return
        
        xs = np.polyval(f, ploty_fit)
        xs = np.round(xs).astype(np.int32)
        
        mask = (xs >= 0) & (xs < w)
        if np.sum(mask) < 2: return
        
        pts = np.stack([xs[mask], ploty_fit[mask].astype(np.int32)], axis=1).reshape(-1, 1, 2)
        cv2.polylines(vis, [pts], False, color, line_thickness)

    draw_fit(left_fit, left_color, l_yr)
    draw_fit(right_fit, right_color, r_yr)

    # Draw Centerline
    if ptsC is not None:
        pts = np.asarray(ptsC, dtype=np.int32)
        if pts.ndim == 2: pts = pts.reshape(-1, 1, 2)
        
        for p in pts:
            cv2.circle(vis, (p[0][0], p[0][1]), center_radius, center_color, -1)
        
        cv2.polylines(vis, [pts], False, center_color, center_thickness)

    return vis

def project_bev_to_camera(frame_bgr, bev_vis, minv):
    """
    Overlays the BEV visualization (windows, fits, etc.) back onto the original frame
    by projecting it through the inverse homography.
    """
    h, w = frame_bgr.shape[:2]
    
    # 1. Project BEV visualization back to camera perspective
    warped_back = cv2.warpPerspective(
        bev_vis, minv, (w, h),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0
    )
    
    # 2. Overlay onto original frame
    # Create mask where warped_back is not black
    mask = cv2.cvtColor(warped_back, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(mask, 1, 255, cv2.THRESH_BINARY)
    
    out = frame_bgr.copy()
    # Apply warped_back to out where mask is active
    out[mask > 0] = warped_back[mask > 0]
    
    return out
