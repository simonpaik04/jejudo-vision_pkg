import numpy as np

# ============================================================
# ROS 토픽
# ============================================================
IMAGE_TOPIC = "/image_raw"  # 입력 카메라 토픽(실차 기본). sim 모드에서 main.py가 override 가능

# ============================================================
# 카메라 외부 파라미터 (Extrinsic)
# ============================================================
CAMERA_HEIGHT_M  = 1.40  # 카메라 높이[m] (지면~렌즈 중심)
CAMERA_PITCH_DEG = 25.0  # 카메라 하향 피치[deg] (+면 아래를 봄)
CAMERA_ROLL_DEG  = 0.0   # 카메라 기울기 보정 (BEV 대각선이면 ±1~3 조정)
CAMERA_YAW_DEG   = 0.0   # 카메라 좌우 틀어짐 보정

# ============================================================
# 카메라 내부 파라미터 (Intrinsic)
# ============================================================
CAMERA_FOV_DEG = 103.0  # K가 없을 때(sim) fx/fy 계산에 쓰는 수평 시야각
CENTER_X       = 320.0  # 주점 cx (픽셀)
CENTER_Y       = 180.0  # 주점 cy (픽셀)

# 실제 카메라 캘리브레이션 (렌즈 왜곡 보정) — 640×360 기준
K_REAL = np.array([
    [484.99055,   0.     , 309.09636],
    [  0.     , 454.50456, 186.28021],
    [  0.     ,   0.     ,   1.     ]
], dtype=np.float32)
DIST_COEFF_REAL = np.array([-0.337306, 0.076342, -0.003823, 0.004501, 0.0], dtype=np.float32)

# 가변 파라미터 (main.py에서 모드에 따라 결정됨)
K          = None                    # sim: None(FOV 기반), real: K_REAL
DIST_COEFF = np.zeros(5, dtype=np.float32)  # sim: 0, real: DIST_COEFF_REAL
ENABLE_UNDISTORT = True              # 렌즈 왜곡 보정 ON/OFF

# ============================================================
# IPM / BEV 설정
# ============================================================
M_PER_PIXEL    = 0.01  # BEV 해상도 [m/px] (작을수록 정밀, 연산량 증가)
BEV_WIDTH_M    = 4.60  # BEV 좌우 폭 [m]
BEV_LENGTH_M   = 4.40  # BEV 전방 길이 [m]

BEV_IMAGE_WIDTH  = int(BEV_WIDTH_M  / M_PER_PIXEL)   # 280 px
BEV_IMAGE_HEIGHT = int(BEV_LENGTH_M / M_PER_PIXEL)   # 270 px
BEV_SCALE        = 1.0   # BEV 다운스케일 비율 (1.0=원본, 0.5=절반)

# ============================================================
# 기초 비전 파라미터
# ============================================================
CANNY_LOW  = 100   # 낮으면 노이즈↑, 높으면 약한 차선 누락↑
CANNY_HIGH = 150  # 낮/밤/노면 대비에 따라 함께 튜닝

# ============================================================
# 슬라이딩 윈도우 (Sliding Window)
# ============================================================
N_WINDOWS        = 9   # 세로 윈도우 개수 (많을수록 곡선 추종↑, 연산량↑)
SW_MARGIN        = 30  # 각 윈도우 반폭[px]
SW_BOTTOM_OFFSET = 100         # 자동 계산 (process.py 첫 프레임에서 BEV 검은 영역 높이로 설정)
SW_VISION_RATIO  = 1.0       # BEV 유효영역 중 SW가 볼 비율 (1.0=전체, 0.5=하단 절반만)
SW_CURVE_RATIO   = 0.0      # 커브 시 마진 동적 확장 비율
SW_PRED_AMP      = 1.0      # 예측 이동 증폭 계수
POLY_ORDER       = 1       # 차선 피팅 차수 (1=선형)
DRAW_WINDOWS     = True     # 윈도우 사각형 시각화

MINPIX           = 20       # 윈도우 이동 갱신 최소 픽셀 (일반)
MINPIX_BOTTOM    = 20      # 하단 2개 윈도우 최소 픽셀
MAX_GAP_WINDOWS  = 0       # 연속 빈 윈도우 허용 개수
MINLANEPIX       = 100      # fit 생성 최소 누적 픽셀

# 리셋 슬라이딩 시작점 추정
BOTTOM_RATIO         = 0.30  # 히스토그램을 볼 하단 ROI 비율
LEFT_RATIO           = 0.50  # left 후보영역 x < w*LEFT_RATIO
RIGHT_RATIO          = 0.50  # right 후보영역 x >= w*(1-RIGHT_RATIO)
START_STAT           = "median"   # "median" or "mean"
FALLBACK_LEFT_RATIO  = 0.25       # 픽셀 없을 때 left 시작 x = w * 0.25
FALLBACK_RIGHT_RATIO = 0.75       # 픽셀 없을 때 right 시작 x = w * 0.75

RESET_INTERVAL = 50000               # 강제 리셋 주기 (프레임)

# ============================================================
# Tracking (Around Poly)
# ============================================================
TRACK_MARGIN        = 50    # 이전 fit 주변 탐색 폭 — 하단(가까움) (px)
TRACK_MARGIN_TOP    = 50    # 이전 fit 주변 탐색 폭 — 상단(먼거리) (px) — 커브 시 옆차선 혼입 방지
TRACK_MAX_Y_GAP     = 5    # 밴드 내 픽셀 y-gap 허용 한계 (px) — 이상 끊기면 먼쪽 클러스터 버림 (0=OFF)
FIT_DRAW_GAP        = 10    # fit 렌더링 시 y_range 밖으로 이 px 이상 벗어나면 그리지 않음

# ============================================================
# Tracking / Reset 정책
# ============================================================
MAX_LOST                      = 10    # 이 횟수 이상 연속 실패 시 전역 리셋 탐색
IMMEDIATE_RESET_ON_FAIL_RIGHT = True  # right 실패 즉시 lost를 MAX_LOST로 올려 재탐색 유도
IMMEDIATE_RESET_ON_FAIL_LEFT  = True  # left 실패 즉시 lost를 MAX_LOST로 올려 재탐색 유도
RESET_LEFT_WHOLE_LEFT_HALF    = True
RESET_RIGHT_WHOLE_RIGHT_HALF  = True

# ============================================================
# 차선 적합성 검증 (Fit Validity Gates)
# ============================================================
BOTTOM_GUARD       = 15     # 중앙 침범 판정 가드 (px) — 클수록 겹침 방지 강화
BOTTOM_MIN_Y_RATIO = 0.80   # bottom cross 체크 최소 y비율 — 낮을수록 항상 체크
MIN_LANE_WIDTH_PX  = 60     # 좌우 최소 차폭 (px)
WIDTH_GUARD        = 15     # 좌/우 판정 가드 (px)

# ============================================================
# 가변 중앙(mid) 기준
# ============================================================
USE_DYNAMIC_MID      = True   # True면 고정 w/2 대신 동적 mid로 좌/우 게이트 판정
DYN_MID_ALPHA        = 0.85   # (현재 구현 미사용) EMA 계수: 1에 가까울수록 변화 완만
DYN_MID_MAX_SHIFT_PX = 70     # (현재 구현 미사용) 동적 mid 클램프 한계

# ============================================================
# 차선 폭 모델 (Width Model — 한쪽 차선만 있을 때 중심선 추정)
# ============================================================
LANE_WIDTH_M          = 1.50    # 대회 차선 간격 고정값 [m]
SINGLE_SIDE_OFFSET_M  = 0.75    # 한쪽 차선만 보일 때 중심선 오프셋 [m]
DEFAULT_LANE_WIDTH_PX = 60      # 양쪽 미검출/초기상태에서 쓰는 기본 차폭
WIDTH_ALPHA           = 0.90    # 폭 업데이트 smoothing (클수록 과거값 유지)
TOP_BAND              = 0.25    # 위쪽 폭 측정 영역 비율
BOTTOM_BAND           = 0.25    # 아래쪽 폭 측정 영역 비율
WIDTH_GAMMA           = 0.70    # 폭 프로파일 감마 (1=선형, <1=위로 갈수록 더 좁아짐)
MIN_W                 = 40      # 폭 최소 클립 (px)
MAX_W                 = 2000    # 폭 최대 클립 (px)

# ============================================================
# Smoothing / Filtering
# ============================================================
SMOOTH_ALPHA          = 0.70    # fit EMA smoothing (클수록 안정, 반응 느림)
BASE_JUMP_PX          = 60      # 히스토그램 base 점프 필터 임계값 (px)
LANE_STABILITY_FRAMES = 15      # (보조 파라미터) 연속 검출 안정 판정 기준 프레임
FIT_HOLD_FRAMES       = 2       # 소실 시 이전 fit 유지 최대 프레임 수
FIT_HOLD_MIN_STREAK   = 3       # hold 활성화 최소 연속 성공 프레임

# ============================================================
# 중심선 / 경로 (Centerline)
# ============================================================
SINGLE_LANE_OFFSET = 0.5        # (현재 centerline 폭모델 사용 시 보조값) 단일 차선 오프셋 비율
CENTER_MOVE_MIN_SPAN_RATIO = 0.35  # 양쪽 차선이 이 높이비율 이상 검출될 때만 중심점(빨간점) 이동
CENTER_STEP        = 8          # 중심선 점 간격 (클수록 점 적음)
CENTER_RADIUS      = 3          # 중심선 점 반지름
CENTER_THICKNESS   = 2          # 중심선 폴리라인 두께

# ============================================================
# 시각화
# ============================================================
PUBLISH_DEBUG_IMAGES = True  # False면 BEV/final/canny/sw_vis 퍼블리시 최소화

# ============================================================
# STATE — 슬라이딩 윈도우 전역 상태 (sliding_window.py 와 공유)
# ※ 코드에서 직접 참조하지 말고 sliding_window.py 내부에서만 사용
# ============================================================
_prev_width_bottom_px = None
_prev_width_top_px    = None
_prevleftbase         = None
_prevrightbase        = None
_prev_left_fit        = None
_prev_right_fit       = None
_lost_count_left      = 0
_lost_count_right     = 0
_prev_lane_width_px   = None