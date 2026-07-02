import numpy as np
import cv2
import config as cfg

# ============================================================
# 모든 파라미터는 config.py (cfg) 에서 통합 관리합니다.
# RQT Dynamic Reconfigure → main.py → cfg 값 변경 → 여기서 실시간 반영
# ============================================================

# ============================================================
# STATE (전역 상태)
# ============================================================
prev_width_bottom_px = None
prev_width_top_px = None

prevleftbase = None
prevrightbase = None
prev_left_fit = None
prev_right_fit = None
lost_count_left = 0
lost_count_right = 0
prev_lane_width_px = None
hold_count_left = 0
hold_count_right = 0
success_streak_left = 0
success_streak_right = 0
prev_l_yr = None
prev_r_yr = None
dynamic_mid_x = None
prev_center_mid_x = None


def _single_side_offset_px():
    """한쪽 차선만 있을 때 중심선 추정에 사용할 고정 오프셋(px)."""
    m_per_px = float(getattr(cfg, 'M_PER_PIXEL', 0.01))
    if m_per_px <= 0:
        m_per_px = 0.01
    offset_m = float(getattr(cfg, 'SINGLE_SIDE_OFFSET_M', 0.75))
    return max(1.0, offset_m / m_per_px)


def _resolve_mid(w, mid_override=None):
    """좌/우 판정용 기준 mid를 반환한다."""
    global dynamic_mid_x
    if mid_override is not None:
        return float(mid_override)
    if getattr(cfg, 'USE_DYNAMIC_MID', False) and dynamic_mid_x is not None:
        return float(dynamic_mid_x)
    return float(w // 2)


def _update_dynamic_mid_from_history(h, w):
    """
    이전 프레임 중심선(ptsC 하단점)만 사용해 현재 프레임 동적 mid를 업데이트.
    """
    global dynamic_mid_x, prev_center_mid_x

    if not getattr(cfg, 'USE_DYNAMIC_MID', False):
        dynamic_mid_x = float(w // 2)
        return dynamic_mid_x

    if prev_center_mid_x is not None:
        dynamic_mid_x = float(prev_center_mid_x)
    else:
        dynamic_mid_x = float(w // 2)
    return dynamic_mid_x


def invalidate_fit_if_bottom_cross(
    fit, y_pixels, w, h, side,
    guard=cfg.BOTTOM_GUARD,
    min_y_ratio=cfg.BOTTOM_MIN_Y_RATIO,
    mid_override=None
):
    """
    fit: np.polyfit(y, x, 1) -> x = a*y + b
    side='left' : 맨 아래 검출점에서 x_pred > mid-guard 이면 INVALID
    side='right': 맨 아래 검출점에서 x_pred < mid+guard 이면 INVALID
    """
    if fit is None:
        return None

    f = np.asarray(fit).reshape(-1)
    if f.size != 2:
        return None

    if y_pixels is None or len(y_pixels) == 0:
        return fit

    y_max = int(np.max(y_pixels))
    if y_max < int(h * float(min_y_ratio)):
        return fit

    a, b = float(f[0]), float(f[1])
    x_at_bottom = a * float(y_max) + b

    mid = _resolve_mid(w, mid_override)

    # 오른쪽에 대한 조건
    if side == 'right':
        if x_at_bottom < (mid + guard):
            return None
    
    # 왼쪽에 대한 조건
    elif side == 'left':
        if x_at_bottom > (mid - guard):
            return None

    return fit


def _x_at_y(fit, y):
    if fit is None:
        return None
    f = np.asarray(fit).reshape(-1)
    if f.size < 2:
        return None
    return float(np.polyval(f, y))


def min_width_pair_gate(left_fit, right_fit, w, y_ref,
                        min_width_px=cfg.MIN_LANE_WIDTH_PX, guard=cfg.WIDTH_GUARD,
                        mid_override=None):
    """
    좌/우 둘 다 있을 때 폭이 너무 좁으면:
    - 둘 다 한쪽으로 몰렸으면 반대쪽만 제거
    - 애매하면 둘 다 제거(None)
    """
    if left_fit is None or right_fit is None:
        return left_fit, right_fit

    mid = _resolve_mid(w, mid_override)
    xL = _x_at_y(left_fit, y_ref)
    xR = _x_at_y(right_fit, y_ref)
    if xL is None or xR is None:
        return left_fit, right_fit

    width = xR - xL
    if width >= float(min_width_px):
        return left_fit, right_fit

    both_right = (xL > (mid + guard)) and (xR > (mid + guard))
    both_left  = (xL < (mid - guard)) and (xR < (mid - guard))

    if both_right:
        return None, right_fit
    if both_left:
        return left_fit, None

    return None, None


def swap_guard(left_fit, right_fit, h):
    """y 범위 전체에서 left_x >= right_x인 구간이 있으면 둘 다 무효화."""
    if left_fit is None or right_fit is None:
        return left_fit, right_fit
    ys = np.linspace(0, h - 1, num=10)
    lxs = np.polyval(np.asarray(left_fit).reshape(-1), ys)
    rxs = np.polyval(np.asarray(right_fit).reshape(-1), ys)
    if np.any(lxs >= rxs):
        return None, None
    return left_fit, right_fit


def smooth_fit(prev_fit, cur_fit, alpha=cfg.SMOOTH_ALPHA):
    if cur_fit is None:
        return None
    if prev_fit is None:
        return cur_fit
    return alpha * prev_fit + (1 - alpha) * cur_fit


# ============================================================
# CENTERLINE (폭(y) 모델 포함)
# ============================================================
def compute_and_draw_centerline(
    vis, left_fit, right_fit,
    l_yr=None, r_yr=None,
    center_step=cfg.CENTER_STEP, center_radius=cfg.CENTER_RADIUS,
    center_color=(0, 255, 0), center_thickness=cfg.CENTER_THICKNESS
):
    global prev_lane_width_px
    global prev_width_bottom_px, prev_width_top_px

    if vis is None:
        return None, None, None

    h, w = vis.shape[:2]
    ploty = np.arange(h - 1, -1, -1, dtype=np.float32)   # bottom->top
    ys = ploty.astype(np.int32)

    if prev_lane_width_px is None:
        prev_lane_width_px = cfg.DEFAULT_LANE_WIDTH_PX

    def fit_ok(f):
        return (f is not None) and (np.asarray(f).reshape(-1).size == 2)

    hasL = fit_ok(left_fit)
    hasR = fit_ok(right_fit)
    if (not hasL) and (not hasR):
        return None, None, None

    # ── y_range 기반 유효 마스크 (FIT_DRAW_GAP 이상 벗어나면 끊기) ──
    gap_px = getattr(cfg, 'FIT_DRAW_GAP', 0)
    l_valid = np.ones(len(ploty), dtype=bool)
    r_valid = np.ones(len(ploty), dtype=bool)
    if gap_px > 0:
        if hasL and l_yr is not None:
            l_valid = (ploty >= (l_yr[0] - gap_px)) & (ploty <= (l_yr[1] + gap_px))
        if hasR and r_yr is not None:
            r_valid = (ploty >= (r_yr[0] - gap_px)) & (ploty <= (r_yr[1] + gap_px))

    xsL = None
    xsR = None
    if hasL:
        lf = np.asarray(left_fit, dtype=np.float32).reshape(-1)
        xsL = lf[0] * ploty + lf[1]
    if hasR:
        rf = np.asarray(right_fit, dtype=np.float32).reshape(-1)
        xsR = rf[0] * ploty + rf[1]

    # (1) 양쪽이 있을 때: bottom 폭 + top 폭 저장
    if hasL and hasR:
        width = (xsR - xsL).astype(np.float32)
        valid = (width > cfg.MIN_W) & (width < cfg.MAX_W)

        y_top_cut = int(h * cfg.TOP_BAND)
        y_bot_cut = int(h * (1.0 - cfg.BOTTOM_BAND))

        top_mask = (ploty <= float(y_top_cut)) & valid
        bot_mask = (ploty >= float(y_bot_cut)) & valid

        if np.any(valid):
            w_bottom_meas = float(np.median(width[bot_mask])) if np.any(bot_mask) else float(np.median(width[valid]))
            w_top_meas    = float(np.median(width[top_mask])) if np.any(top_mask) else w_bottom_meas

            prev_width_bottom_px = w_bottom_meas if prev_width_bottom_px is None \
                else (cfg.WIDTH_ALPHA * float(prev_width_bottom_px) + (1 - cfg.WIDTH_ALPHA) * w_bottom_meas)

            prev_width_top_px = w_top_meas if prev_width_top_px is None \
                else (cfg.WIDTH_ALPHA * float(prev_width_top_px) + (1 - cfg.WIDTH_ALPHA) * w_top_meas)

            prev_lane_width_px = float(prev_width_bottom_px)

        xc = 0.5 * (xsL + xsR)

    # (2) 한쪽만 있을 때: 고정 오프셋(0.75m)으로 중심선 추정
    else:
        offset_px = float(_single_side_offset_px())
        if hasL and (not hasR):
            xc = xsL + offset_px
        else:
            xc = xsR - offset_px

    # draw — y_range 밖은 그리지 않음
    xc = np.round(xc).astype(np.int32)
    # 양쪽 모두의 유효 범위 교집합 (한쪽만 있으면 그 쪽 유효 범위)
    if hasL and hasR:
        draw_mask = l_valid & r_valid & (xc >= 0) & (xc < w)
    elif hasL:
        draw_mask = l_valid & (xc >= 0) & (xc < w)
    else:
        draw_mask = r_valid & (xc >= 0) & (xc < w)
    xc2 = xc[draw_mask]
    yc2 = ys[draw_mask]

    xc2 = xc2[::center_step]
    yc2 = yc2[::center_step]
    if xc2.size < 2:
        return None, None, None

    for x, y in zip(xc2, yc2):
        cv2.circle(vis, (int(x), int(y)), center_radius, center_color, -1)

    ptsC = np.stack([xc2, yc2], axis=1).reshape(-1, 1, 2).astype(np.int32)
    cv2.polylines(vis, [ptsC], isClosed=False, color=center_color, thickness=center_thickness)
    return ptsC, xc2, yc2


# ============================================================
# SLIDING WINDOW (reset용)
# ============================================================
def sliding_window_left(
    binary_warped,
    nwindows=cfg.N_WINDOWS, margin=cfg.SW_MARGIN,
    minpix=cfg.MINPIX, minpix_bottom=cfg.MINPIX_BOTTOM,
    minlanepix=cfg.MINLANEPIX, guard=30,
    max_gap_windows=cfg.MAX_GAP_WINDOWS,
    whole_left_half=True,
    bottom_ratio=cfg.BOTTOM_RATIO,
    left_ratio=cfg.LEFT_RATIO,
    start_stat=cfg.START_STAT,
    fallback_ratio=cfg.FALLBACK_LEFT_RATIO,
    draw_windows=cfg.DRAW_WINDOWS
):
    h, w = binary_warped.shape[:2]
    mid = w // 2
    out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255

    window_height = int(h / nwindows)
    nonzeroy, nonzerox = binary_warped.nonzero()
    nonzeroy = np.array(nonzeroy)
    nonzerox = np.array(nonzerox)

    windows_rects = []

    left_limit = int(w * left_ratio)
    if whole_left_half:
        left_region = (nonzerox < left_limit)
    else:
        left_region = (nonzerox < (mid - guard))

    y_cut = int(h * (1.0 - bottom_ratio))
    bottom_mask = left_region & (nonzeroy >= y_cut)
    bottom_x = nonzerox[bottom_mask]

    if bottom_x.size > 0:
        leftx_current = int(np.mean(bottom_x)) if start_stat == "mean" else int(np.median(bottom_x))
    else:
        leftx_current = int(w * fallback_ratio)

    lane_inds = []
    gap = 0

    for window in range(nwindows):
        win_y_low  = h - (window + 1) * window_height
        win_y_high = h - window * window_height

        win_x_low  = max(0, leftx_current - margin)
        win_x_high = min(w, leftx_current + margin)

        windows_rects.append((win_x_low, win_y_low, win_x_high, win_y_high))

        if draw_windows:
            cv2.rectangle(out_img, (win_x_low, win_y_low),
                          (win_x_high, win_y_high), (0, 255, 0), 2)

        good = (
            left_region &
            (nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
            (nonzerox >= win_x_low) & (nonzerox < win_x_high)
        ).nonzero()[0]

        lane_inds.append(good)

        req = minpix_bottom if window <= 1 else minpix
        if len(good) >= req:
            gap = 0
            leftx_current = int(np.mean(nonzerox[good]))
        else:
            gap += 1
            if gap > max_gap_windows:
                break

    lane_inds = np.concatenate(lane_inds) if len(lane_inds) else np.array([], dtype=np.int64)

    fit = None
    y_range = None
    xs = np.array([])
    ys = np.array([])

    if lane_inds.size >= minlanepix:
        xs = nonzerox[lane_inds]
        ys = nonzeroy[lane_inds]
        fit = np.polyfit(ys, xs, 1)
        fit = invalidate_fit_if_bottom_cross(fit, ys, w, h, side='left')
        if fit is not None:
            y_range = (int(ys.min()), int(ys.max()))
            out_img[ys, xs] = [255, 0, 0]

    return fit, out_img, int(lane_inds.size), y_range, windows_rects, xs, ys


def sliding_window_right(
    binary_warped,
    nwindows=cfg.N_WINDOWS, margin=cfg.SW_MARGIN,
    minpix=cfg.MINPIX, minpix_bottom=cfg.MINPIX_BOTTOM,
    minlanepix=cfg.MINLANEPIX, guard=30,
    max_gap_windows=cfg.MAX_GAP_WINDOWS,
    whole_right_half=True,
    bottom_ratio=cfg.BOTTOM_RATIO,
    right_ratio=cfg.RIGHT_RATIO,
    start_stat=cfg.START_STAT,
    fallback_ratio=cfg.FALLBACK_RIGHT_RATIO,
    draw_windows=cfg.DRAW_WINDOWS
):
    h, w = binary_warped.shape[:2]
    mid = w // 2
    out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255

    window_height = int(h / nwindows)
    nonzeroy, nonzerox = binary_warped.nonzero()
    nonzeroy = np.array(nonzeroy)
    nonzerox = np.array(nonzerox)

    windows_rects = []

    right_start = int(w * (1.0 - right_ratio))
    if whole_right_half:
        right_region = (nonzerox >= right_start)
    else:
        right_region = (nonzerox > (mid + guard))

    y_cut = int(h * (1.0 - bottom_ratio))
    bottom_mask = right_region & (nonzeroy >= y_cut)
    bottom_x = nonzerox[bottom_mask]

    if bottom_x.size > 0:
        rightx_current = int(np.mean(bottom_x)) if start_stat == "mean" else int(np.median(bottom_x))
    else:
        rightx_current = int(w * fallback_ratio)

    lane_inds = []
    gap = 0

    for window in range(nwindows):
        win_y_low  = h - (window + 1) * window_height
        win_y_high = h - window * window_height

        win_x_low  = max(0, rightx_current - margin)
        win_x_high = min(w, rightx_current + margin)

        windows_rects.append((win_x_low, win_y_low, win_x_high, win_y_high))

        if draw_windows:
            cv2.rectangle(out_img, (win_x_low, win_y_low),
                          (win_x_high, win_y_high), (0, 255, 0), 2)

        good = (
            right_region &
            (nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
            (nonzerox >= win_x_low) & (nonzerox < win_x_high)
        ).nonzero()[0]

        lane_inds.append(good)

        req = minpix_bottom if window <= 1 else minpix
        if len(good) >= req:
            gap = 0
            rightx_current = int(np.mean(nonzerox[good]))
        else:
            gap += 1
            if gap > max_gap_windows:
                break

    lane_inds = np.concatenate(lane_inds) if len(lane_inds) else np.array([], dtype=np.int64)

    fit = None
    y_range = None
    xs = np.array([])
    ys = np.array([])

    if lane_inds.size >= minlanepix:
        xs = nonzerox[lane_inds]
        ys = nonzeroy[lane_inds]
        fit = np.polyfit(ys, xs, 1)
        fit = invalidate_fit_if_bottom_cross(fit, ys, w, h, side='right')
        if fit is not None:
            y_range = (int(ys.min()), int(ys.max()))
            out_img[ys, xs] = [0, 0, 255]

    return fit, out_img, int(lane_inds.size), y_range, windows_rects, xs, ys


# ============================================================
# TRACKING (around poly)
# ============================================================
def _clip_y_gap(xs, ys, max_gap):
    """
    y값이 큰 쪽(BEV 하단=가까운 곳)부터 시작해서,
    연속된 픽셀 사이 y-gap이 max_gap을 넘으면 거기서 끊고
    아래쪽(가까운) 클러스터만 반환.
    max_gap <= 0 이면 무제한(전부 반환).
    """
    if max_gap <= 0 or len(ys) < 2:
        return xs, ys
    order = np.argsort(-ys)           # y 큰 순(하단→상단)
    ys_s = ys[order]
    xs_s = xs[order]
    diffs = np.abs(np.diff(ys_s))     # 연속 y 차이
    cut = np.where(diffs > max_gap)[0]
    if len(cut) == 0:
        return xs, ys                 # gap 없음 → 전부 사용
    keep = order[:cut[0] + 1]         # 첫 번째 큰 gap 이전까지만
    return xs[keep], ys[keep]


def search_around_poly(binary_warped, left_fit, right_fit,
                       margin=None, minlanepix=None):
    if margin is None: margin = cfg.TRACK_MARGIN
    if minlanepix is None: minlanepix = cfg.MINLANEPIX
    margin_top = getattr(cfg, 'TRACK_MARGIN_TOP', margin)  # 먼 거리 마진
    out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255

    nonzeroy, nonzerox = binary_warped.nonzero()
    nonzeroy = np.array(nonzeroy)
    nonzerox = np.array(nonzerox)

    h, w = binary_warped.shape

    # 테이퍼 마진: y=0(상단/먼곳)→margin_top, y=h-1(하단/가까움)→margin
    # margin_arr[i] = margin_top + (margin - margin_top) * (y[i] / (h-1))
    denom = max(h - 1, 1)
    margin_arr = margin_top + (margin - margin_top) * (nonzeroy.astype(np.float32) / denom)

    lf_new = None
    rf_new = None
    lc = 0
    rc = 0
    l_yr = None
    r_yr = None

    max_y_gap = getattr(cfg, 'TRACK_MAX_Y_GAP', 0)

    if left_fit is not None:
        x_pred = left_fit[0] * nonzeroy + left_fit[1]
        mask = (np.abs(nonzerox - x_pred) < margin_arr)
        lc = int(np.sum(mask))
        if lc >= minlanepix:
            idx = np.where(mask)[0]
            xs = nonzerox[idx]
            ys = nonzeroy[idx]
            xs, ys = _clip_y_gap(xs, ys, max_y_gap)
            lc = len(ys)
        if lc >= minlanepix:
            lf_new = np.polyfit(ys, xs, 1)
            lf_new = invalidate_fit_if_bottom_cross(lf_new, ys, w, h, side='left')
            if lf_new is not None:
                out_img[ys, xs] = [255, 0, 0]
                l_yr = (int(ys.min()), int(ys.max()))
            else:
                lc = 0

    if right_fit is not None:
        x_pred = right_fit[0] * nonzeroy + right_fit[1]
        mask = (np.abs(nonzerox - x_pred) < margin_arr)
        rc = int(np.sum(mask))
        if rc >= minlanepix:
            idx = np.where(mask)[0]
            xs = nonzerox[idx]
            ys = nonzeroy[idx]
            xs, ys = _clip_y_gap(xs, ys, max_y_gap)
            rc = len(ys)
        if rc >= minlanepix:
            rf_new = np.polyfit(ys, xs, 1)
            rf_new = invalidate_fit_if_bottom_cross(rf_new, ys, w, h, side='right')
            if rf_new is not None:
                out_img[ys, xs] = [0, 0, 255]
                r_yr = (int(ys.min()), int(ys.max()))
            else:
                rc = 0

    return lf_new, rf_new, out_img, lc, rc, l_yr, r_yr


# ============================================================
# PROGRESSIVE FALLBACK — Anchored / Relocalize / Peaks
# (lane.py 에서 호출 — config.py 파라미터 사용)
# ============================================================

def sliding_window_anchored(binary_warped, start_x, side):
    """
    이전 프레임의 base x좌표에서 바로 윈도우를 쌓아 올리는 고속 탐색.
    히스토그램 계산 없이 시작 → 연속 프레임에서 가장 빠름.
    side='left'/'right'에 따라 영역 제한 + bottom cross 검증 적용.

    Returns: (fit, pixel_count, y_range, windows_rects, xs, ys)
    """
    h, w = binary_warped.shape[:2]
    mid = w // 2
    nwindows  = cfg.N_WINDOWS
    margin    = cfg.SW_MARGIN

    window_height = max(1, h // nwindows)

    nonzeroy, nonzerox = binary_warped.nonzero()
    nonzeroy = np.array(nonzeroy)
    nonzerox = np.array(nonzerox)

    # ── 좌/우 영역 제한 (기존 sliding_window_left/right 동일) ──
    if side == 'left':
        region_mask = (nonzerox < int(w * cfg.LEFT_RATIO))
    else:
        region_mask = (nonzerox >= int(w * (1.0 - cfg.RIGHT_RATIO)))

    current_x = int(start_x)
    lane_inds     = []
    windows_rects = []
    gap    = 0
    prev_dx = 0

    for win_idx in range(nwindows):
        win_y_high = h - win_idx * window_height
        win_y_low  = h - (win_idx + 1) * window_height
        if win_y_low < 0:
            win_y_low = 0

        # 커브 시 동적 마진 확장
        dyn_margin = margin
        if cfg.SW_CURVE_RATIO > 0 and abs(prev_dx) > 2:
            dyn_margin = int(margin + abs(prev_dx) * cfg.SW_CURVE_RATIO)

        win_x_low  = max(0, current_x - dyn_margin)
        win_x_high = min(w, current_x + dyn_margin)

        windows_rects.append((win_x_low, win_y_low, win_x_high, win_y_high))

        good = (
            region_mask &
            (nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
            (nonzerox >= win_x_low) & (nonzerox < win_x_high)
        ).nonzero()[0]

        lane_inds.append(good)

        req = cfg.MINPIX_BOTTOM if win_idx <= 1 else cfg.MINPIX
        if len(good) >= req:
            new_x   = int(np.mean(nonzerox[good]))
            prev_dx = new_x - current_x
            # 모멘텀 예측: 다음 윈도우를 곡선 방향으로 미리 밀어놓음
            current_x = new_x + int(prev_dx * (cfg.SW_PRED_AMP - 1.0))
            gap = 0
        else:
            gap += 1
            # 빈 윈도우에서도 모멘텀으로 전진
            current_x += int(prev_dx * cfg.SW_PRED_AMP)
            if gap > cfg.MAX_GAP_WINDOWS:
                break

    lane_inds = np.concatenate(lane_inds) if lane_inds else np.array([], dtype=np.int64)

    fit     = None
    y_range = None
    count   = int(lane_inds.size)
    xs = np.array([], dtype=np.intp)
    ys = np.array([], dtype=np.intp)

    if lane_inds.size > 0:
        xs = nonzerox[lane_inds]
        ys = nonzeroy[lane_inds]

        if count >= cfg.MINLANEPIX:
            fit = np.polyfit(ys, xs, cfg.POLY_ORDER)
            # ── bottom cross 검증 (기존 sliding_window_left/right 동일) ──
            fit = invalidate_fit_if_bottom_cross(fit, ys, w, h, side=side)
            if fit is not None:
                y_range = (int(ys.min()), int(ys.max()))
            else:
                count = 0

    return fit, count, y_range, windows_rects, xs, ys


def relocalize_base(binary_warped, prev_base, side, search_range=100):
    """
    이전 base 주변 ±search_range 에서 로컬 히스토그램 피크를 찾는다.
    전역 검색보다 훨씬 빠르면서, 일시적으로 소실된 차선을 재탐색.

    Returns: new_base_x (int) or None
    """
    h, w = binary_warped.shape[:2]

    x_low  = max(0, int(prev_base) - search_range)
    x_high = min(w, int(prev_base) + search_range)

    # 하단 영역에서만 히스토그램
    y_start = int(h * (1.0 - cfg.BOTTOM_RATIO))
    roi = binary_warped[y_start:h, x_low:x_high]

    if roi.size == 0:
        return None

    histogram = np.sum(roi, axis=0)

    if np.max(histogram) == 0:
        return None

    peak_local = int(np.argmax(histogram))
    return x_low + peak_local


def find_inner_peaks(binary_warped):
    """
    전체 BEV 하단의 히스토그램에서 좌/우 피크를 찾는 글로벌 검색.
    anchored + relocalize 모두 실패했을 때의 최종 폴백.

    Returns: (left_peak_x, right_peak_x) — 각각 int or None
    """
    h, w = binary_warped.shape[:2]
    mid = w // 2

    y_start = int(h * (1.0 - cfg.BOTTOM_RATIO))
    bottom_slice = binary_warped[y_start:h, :]
    histogram = np.sum(bottom_slice, axis=0)

    left_peak  = None
    right_peak = None

    left_hist  = histogram[:mid]
    right_hist = histogram[mid:]

    if left_hist.size > 0 and np.max(left_hist) > 0:
        left_peak = int(np.argmax(left_hist))

    if right_hist.size > 0 and np.max(right_hist) > 0:
        right_peak = int(np.argmax(right_hist)) + mid

    return left_peak, right_peak


# ============================================================
# ============================================================
# MAIN API (search_around_poly 기반 — 백업용)
# ============================================================
def slidingmsg2(
    binary_warped,
    center_step=cfg.CENTER_STEP, center_radius=cfg.CENTER_RADIUS,
    center_color=(0, 255, 0), center_thickness=cfg.CENTER_THICKNESS,
    guard=30
):
    global prev_left_fit, prev_right_fit, lost_count_left, lost_count_right
    global hold_count_left, hold_count_right
    global success_streak_left, success_streak_right
    global prev_l_yr, prev_r_yr
    global dynamic_mid_x, prev_center_mid_x

    h, w = binary_warped.shape[:2]

    # 현재 프레임에서 사용할 동적 mid 업데이트(이전 프레임 히스토리 기반)
    dynamic_mid_x = _update_dynamic_mid_from_history(h, w)

    out_base = np.dstack((binary_warped, binary_warped, binary_warped)) * 255

    l_yr, r_yr = None, None
    windows = []

    # 1) tracking — yr은 search_around_poly가 실제 칠한 픽셀에서 직접 계산
    lf, rf, out_trk, lc, rc = (None, None, None, 0, 0)
    l_yr_trk, r_yr_trk = None, None
    if (prev_left_fit is not None) or (prev_right_fit is not None):
        lf, rf, out_trk, lc, rc, l_yr_trk, r_yr_trk = search_around_poly(
            binary_warped, prev_left_fit, prev_right_fit)

    out_img = out_trk if out_trk is not None else out_base

    if lf is not None:
        l_yr = l_yr_trk
    if rf is not None:
        r_yr = r_yr_trk

    left_ok = (lf is not None)
    right_ok = (rf is not None)

    # 출처 추적: tracking 성공 = True, sliding window = False
    left_from_track = left_ok
    right_from_track = right_ok

    # 2) lost count
    lost_count_left = (lost_count_left + 1) if (not left_ok) else 0
    lost_count_right = (lost_count_right + 1) if (not right_ok) else 0

    if cfg.IMMEDIATE_RESET_ON_FAIL_RIGHT and (not right_ok):
        lost_count_right = cfg.MAX_LOST
    if cfg.IMMEDIATE_RESET_ON_FAIL_LEFT and (not left_ok):
        lost_count_left = cfg.MAX_LOST

    # 3) left reset
    if lost_count_left >= cfg.MAX_LOST:
        lf2, outL, _, l_yr2, winL, _, _ = sliding_window_left(
            binary_warped,
            whole_left_half=cfg.RESET_LEFT_WHOLE_LEFT_HALF,
            guard=guard
        )
        out_img = np.maximum(out_img, outL)
        windows += winL
        if lf2 is not None:                     # ★ 성공 시에만 카운터 리셋
            lf = lf2
            l_yr = l_yr2
            lost_count_left = 0
            left_from_track = False

    # 4) right reset
    if lost_count_right >= cfg.MAX_LOST:
        rf2, outR, _, r_yr2, winR, _, _ = sliding_window_right(
            binary_warped,
            whole_right_half=cfg.RESET_RIGHT_WHOLE_RIGHT_HALF,
            guard=guard
        )
        out_img = np.maximum(out_img, outR)
        windows += winR
        if rf2 is not None:                     # ★ 성공 시에만 카운터 리셋
            rf = rf2
            r_yr = r_yr2
            lost_count_right = 0
            right_from_track = False

    # 5) width gate — tracking 쪽 우선권
    y_ref = h - 1
    lf_before, rf_before = lf, rf
    lf_g, rf_g = min_width_pair_gate(lf, rf, w, y_ref, mid_override=dynamic_mid_x)
    # gate가 fit을 죽이려 할 때, tracking 쪽은 보호하고 SW 쪽만 버림
    if lf_g is None and lf_before is not None and rf_before is not None:
        if left_from_track and not right_from_track:
            lf, rf = lf_before, None     # tracking(L) 보호, SW(R) 제거
        else:
            lf = lf_g
    else:
        lf = lf_g
    if rf_g is None and rf_before is not None and lf_before is not None:
        if right_from_track and not left_from_track:
            lf, rf = None, rf_before     # tracking(R) 보호, SW(L) 제거
        else:
            rf = rf_g
    else:
        rf = rf_g
    if lf is None and lf_before is not None:
        l_yr = None
    if rf is None and rf_before is not None:
        r_yr = None

    # 5.5) swap guard — 좌우 역전 방지 (tracking 쪽 우선권)
    lf_before, rf_before = lf, rf
    lf_g, rf_g = swap_guard(lf, rf, h)
    if lf_g is None and rf_g is None and lf_before is not None and rf_before is not None:
        # swap 감지 시, tracking 쪽만 살림
        if left_from_track and not right_from_track:
            lf, rf = lf_before, None
        elif right_from_track and not left_from_track:
            lf, rf = None, rf_before
        else:
            lf, rf = lf_g, rf_g        # 둘 다 같은 출처 → 기존 로직
    else:
        lf, rf = lf_g, rf_g
    if lf is None and lf_before is not None:
        l_yr = None
    if rf is None and rf_before is not None:
        r_yr = None

    # 현재 프레임에서 실제 차선 검출 여부(hold 적용 전)
    any_lane_detected_now = (lf is not None) or (rf is not None)

    # 6) fit hold — 일시 소실 시 이전 fit 유지
    if lf is not None:
        success_streak_left += 1
        hold_count_left = 0
    else:
        if (success_streak_left >= cfg.FIT_HOLD_MIN_STREAK
                and hold_count_left < cfg.FIT_HOLD_FRAMES
                and prev_left_fit is not None):
            lf = prev_left_fit
            l_yr = prev_l_yr          # 이전 프레임의 yr 재사용
            hold_count_left += 1
        else:
            success_streak_left = 0
            hold_count_left = 0

    if rf is not None:
        success_streak_right += 1
        hold_count_right = 0
    else:
        if (success_streak_right >= cfg.FIT_HOLD_MIN_STREAK
                and hold_count_right < cfg.FIT_HOLD_FRAMES
                and prev_right_fit is not None):
            rf = prev_right_fit
            r_yr = prev_r_yr          # 이전 프레임의 yr 재사용
            hold_count_right += 1
        else:
            success_streak_right = 0
            hold_count_right = 0

    # 7) smoothing
    prev_left_fit = smooth_fit(prev_left_fit, lf, alpha=cfg.SMOOTH_ALPHA)
    prev_right_fit = smooth_fit(prev_right_fit, rf, alpha=cfg.SMOOTH_ALPHA)

    # yr은 실제 탐색(tracking/sliding)에서 나온 값을 유지
    if l_yr is not None:
        prev_l_yr = l_yr
    if r_yr is not None:
        prev_r_yr = r_yr

    # 8) centerline
    ptsC, _, _ = compute_and_draw_centerline(
        out_img, prev_left_fit, prev_right_fit,
        l_yr=l_yr, r_yr=r_yr,
        center_step=center_step,
        center_radius=center_radius,
        center_color=center_color,
        center_thickness=center_thickness
    )

    # 빨간 중심점(dynamic mid) 정책
    # - 한쪽이라도 실제 검출되면 중심선 하단점을 따라 이동
    # - 양쪽 모두 미검출이면 화면 중앙으로 고정
    center_default_x = float(w // 2)
    center_draw_y = int(max(0, h - 1 - cfg.SW_BOTTOM_OFFSET))
    prev_center_mid_x = center_default_x
    dynamic_mid_x = center_default_x

    if any_lane_detected_now and ptsC is not None and len(ptsC) > 0:
        pts = np.asarray(ptsC).reshape(-1, 2)
        idx_bottom = int(np.argmax(pts[:, 1]))
        prev_center_mid_x = float(pts[idx_bottom, 0])
        dynamic_mid_x = prev_center_mid_x
        center_draw_y = int(np.clip(round(pts[idx_bottom, 1]), 0, h - 1))

    if out_img is not None:
        x_mid = int(np.clip(round(dynamic_mid_x), 0, w - 1))
        cv2.circle(out_img, (x_mid, center_draw_y), 5, (0, 0, 255), -1)

    return prev_left_fit, prev_right_fit, out_img, ptsC, l_yr, r_yr, windows
