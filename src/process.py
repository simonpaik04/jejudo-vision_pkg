import cv2
import time
import numpy as np
import utils
import config as cfg
from sliding_window2 import slidingmsg2


class LaneProcessor:
    def __init__(self):
        pass

    # ----------------------------------------------------------
    # 스케일 유틸: 다운스케일된 fit/yr/pts를 풀 해상도로 복원
    # ----------------------------------------------------------
    @staticmethod
    def _scale_poly(fit, inv):
        """fit 계수를 inv 배율로 스케일백 (y,x 모두 inv배 확대)."""
        if fit is None:
            return None
        f = np.asarray(fit, dtype=np.float64).reshape(-1)
        n = f.size - 1
        factors = np.power(inv, 1.0 - np.arange(n, -1, -1, dtype=np.float64))
        return f * factors

    @staticmethod
    def _scale_yr(yr, inv):
        if yr is None:
            return None
        return (int(yr[0] * inv), int(yr[1] * inv))

    def process(self, frame):
        t0 = time.time()

        # 0. Undistort (실차 렌즈 왜곡 보정) — cfg.ENABLE_UNDISTORT로 ON/OFF
        if (getattr(cfg, 'ENABLE_UNDISTORT', True)
                and getattr(cfg, 'DIST_COEFF', None) is not None
                and np.any(cfg.DIST_COEFF != 0)
                and getattr(cfg, 'K', None) is not None):
            frame = cv2.undistort(frame, cfg.K, cfg.DIST_COEFF)

        # 1. Preprocessing
        gray_img     = utils.to_gray(frame)
        gaussian_img = utils.bilateralmsg(gray_img)
        canny_img    = utils.canny_edge(gaussian_img, cfg.CANNY_LOW, cfg.CANNY_HIGH)

        # True BEV Projection (IPM)
        bev_img, M, minv = utils.birds_eye_view(canny_img)

        # Auto-compute BEV black rows (첫 프레임 1회)
        if not hasattr(self, '_offset_done'):
            cfg.SW_BOTTOM_OFFSET = utils.compute_bev_black_rows(
                frame.shape[1], frame.shape[0])
            import rospy
            rospy.loginfo("[vision] BEV black rows = %d px → SW_BOTTOM_OFFSET auto",
                          cfg.SW_BOTTOM_OFFSET)
            self._offset_done = True

        # BEV 하단 검은 영역 크롭 (윈도우 낭비 제거)
        h_bev, w_bev = bev_img.shape[:2]
        bev_offset = cfg.SW_BOTTOM_OFFSET
        if bev_offset > 0:
            bev_crop = bev_img[:h_bev - bev_offset, :]
        else:
            bev_crop = bev_img

        # SW 시야 제한: BEV 유효영역 중 하단 비율만 SW에 입력
        vision_ratio = getattr(cfg, 'SW_VISION_RATIO', 1.0)
        h_valid = bev_crop.shape[0]
        if 0 < vision_ratio < 1.0:
            h_use = int(h_valid * vision_ratio)
            bev_sw = bev_crop[h_valid - h_use:, :]   # 하단(가까운 쪽)만 사용
        else:
            bev_sw = bev_crop
            h_use = h_valid

        # BEV 다운스케일 (cfg.BEV_SCALE < 1.0 일 때)
        bev_scale = getattr(cfg, 'BEV_SCALE', 1.0)
        if bev_scale < 1.0:
            bev_small = cv2.resize(bev_sw, (0, 0), fx=bev_scale, fy=bev_scale,
                                   interpolation=cv2.INTER_AREA)
        else:
            bev_small = bev_sw

        t1 = time.time()

        # 2. Lane Detection (크롭된 BEV에서 수행 → 윈도우 9개가 유효 영역만 커버)
        lf_s, rf_s, out_small, ptsC_s, lyr_s, ryr_s, win_s = slidingmsg2(bev_small)

        t2 = time.time()

        # 3. 스케일백 + 좌표 변환 (SW좌표 → 풀 BEV 좌표)
        # yr_offset: SW 이미지 y좌표를 crop 좌표계로 변환하는 오프셋
        yr_offset = h_valid - h_use if (0 < vision_ratio < 1.0) else 0

        if bev_scale < 1.0:
            inv = 1.0 / bev_scale
        else:
            inv = 1.0

        # 3a. fit 스케일백 + yr_offset 적용
        #     fit: x = a*y + b  →  SW좌표 y_sw를 crop좌표 y_c = y_sw + offset으로 치환
        #     x = a*(y_c - offset) + b = a*y_c + (b - a*offset)
        def _remap_fit(fit_s):
            if fit_s is None:
                return None
            f = self._scale_poly(fit_s, inv) if inv != 1.0 else np.asarray(fit_s, dtype=np.float64)
            if f is None:
                return None
            f = f.reshape(-1)
            if yr_offset > 0 and f.size == 2:
                a, b = f[0], f[1]
                f = np.array([a, b - a * yr_offset])
            return f

        left_fit  = _remap_fit(lf_s)
        right_fit = _remap_fit(rf_s)

        def _remap_yr(yr_s):
            if yr_s is None:
                return None
            yr = self._scale_yr(yr_s, inv) if inv != 1.0 else yr_s
            return (yr[0] + yr_offset, yr[1] + yr_offset)

        l_yr = _remap_yr(lyr_s)
        r_yr = _remap_yr(ryr_s)

        # 3b. centerline pts 변환
        if ptsC_s is not None:
            ptsC = np.asarray(ptsC_s, dtype=np.float32).copy()
            if inv != 1.0:
                ptsC = (ptsC * inv).astype(np.int32)
            else:
                ptsC = ptsC.astype(np.int32)
            if yr_offset > 0:
                if ptsC.ndim == 3:
                    ptsC[:, :, 1] += yr_offset
                else:
                    ptsC[:, 1] += yr_offset
        else:
            ptsC = None

        # 3c. windows 변환
        if inv != 1.0:
            windows = [(int(x1*inv), int(y1*inv + yr_offset),
                        int(x2*inv), int(y2*inv + yr_offset))
                       for x1, y1, x2, y2 in win_s]
        else:
            windows = [(x1, y1 + yr_offset, x2, y2 + yr_offset)
                       for x1, y1, x2, y2 in win_s]

        # 3d. out_img → 풀 BEV 크기로 패드백
        if inv != 1.0:
            out_sw = cv2.resize(out_small, (w_bev, h_use),
                                interpolation=cv2.INTER_NEAREST)
        else:
            out_sw = out_small

        out_img = np.zeros((h_bev, w_bev, 3), dtype=out_sw.dtype)
        out_img[yr_offset:yr_offset + h_use, :] = out_sw

        h, w = h_bev, w_bev

        # Sliding window visualization (just windows/pixels, NO PATH)
        sw_vis_img = out_img.copy()

        # CLEAN overlay (black background) for final_result projection
        overlay_vis = np.zeros((h, w, 3), dtype=np.uint8)
        for rect in windows:
            cv2.rectangle(overlay_vis, (rect[0], rect[1]), (rect[2], rect[3]), (0, 255, 0), 1)

        # Centerline / Path
        pts_line = None
        if ptsC is not None and len(ptsC) >= 4:
            pts_line = np.asarray(ptsC, dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(out_img, [pts_line], False, (0, 255, 255), 2)
            cv2.polylines(overlay_vis, [pts_line], False, (0, 255, 255), 2)

        # Lane visualization
        vis         = utils.draw_lane_on_bev(out_img, left_fit, right_fit, ptsC, l_yr, r_yr)
        overlay_vis = utils.draw_lane_on_bev(overlay_vis, left_fit, right_fit, ptsC, l_yr, r_yr)

        final_img = None
        if getattr(cfg, 'PUBLISH_DEBUG_IMAGES', True):
            final_img = utils.project_bev_to_camera(frame, overlay_vis, minv)

        t3 = time.time()

        import rospy
        preprocess_ms = (t1 - t0) * 1000
        detect_ms     = (t2 - t1) * 1000
        total_ms      = (t3 - t0) * 1000
        rospy.loginfo_throttle(2.0,
            f"[vision] pre={preprocess_ms:.0f}ms  det={detect_ms:.0f}ms  "
            f"total={total_ms:.0f}ms  ({1000/max(total_ms,1):.1f}fps)  "
            f"scale={bev_scale}")

        return {
            "canny":    canny_img,
            "bev":      bev_img,
            "sw_vis":   sw_vis_img,
            "vis":      vis,
            "final":    final_img,
            "pts_line": pts_line,
            "pts_left": None,
            "pts_right": None
        }
