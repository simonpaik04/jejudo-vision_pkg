# vision_pkg Mermaid Diagrams

이 문서는 `catkin_ws/src/vision_pkg`의 현재 소스코드를 기준으로 차선 인식 파이프라인, ROS 메시지 흐름, 주요 클래스/모듈 관계를 Mermaid Markdown으로 정리한 문서입니다.

## Rendered Images

전체 다이어그램 합본 PDF: [Vision_Mermaid_Diagrams.pdf](Vision_Mermaid_Diagrams.pdf)

| Diagram | High-res PNG | SVG | PDF |
|---|---|---|---|
| 전체 파이프라인 | [01_pipeline_large.png](diagrams/high_res/01_pipeline_large.png) | [01_pipeline_large.svg](diagrams/high_res/01_pipeline_large.svg) | [01_pipeline_large.pdf](diagrams/high_res/01_pipeline_large.pdf) |
| ROS 런타임 시퀀스 | [02_ros_runtime_sequence_large.png](diagrams/high_res/02_ros_runtime_sequence_large.png) | [02_ros_runtime_sequence_large.svg](diagrams/high_res/02_ros_runtime_sequence_large.svg) | [02_ros_runtime_sequence_large.pdf](diagrams/high_res/02_ros_runtime_sequence_large.pdf) |
| sliding_window2 내부 시퀀스 | [03_sliding_window_sequence_large.png](diagrams/high_res/03_sliding_window_sequence_large.png) | [03_sliding_window_sequence_large.svg](diagrams/high_res/03_sliding_window_sequence_large.svg) | [03_sliding_window_sequence_large.pdf](diagrams/high_res/03_sliding_window_sequence_large.pdf) |
| 클래스/모듈 다이어그램 | [04_class_module_diagram_large.png](diagrams/high_res/04_class_module_diagram_large.png) | [04_class_module_diagram_large.svg](diagrams/high_res/04_class_module_diagram_large.svg) | [04_class_module_diagram_large.pdf](diagrams/high_res/04_class_module_diagram_large.pdf) |
| 토픽 입출력 요약 | [05_topic_io_large.png](diagrams/high_res/05_topic_io_large.png) | [05_topic_io_large.svg](diagrams/high_res/05_topic_io_large.svg) | [05_topic_io_large.pdf](diagrams/high_res/05_topic_io_large.pdf) |

기존 저용량 PNG/SVG는 `diagrams/`에 남겨두었고, 발표/보고서 삽입용은 `diagrams/high_res/`의 `_large` 파일을 사용하면 됩니다.

## 1. 전체 파이프라인

```mermaid
flowchart TD
    A["카메라 이미지 토픽<br/>real: /image_raw<br/>sim: /image_jpeg/compressed"] --> B["LaneNode.callback()"]
    B --> C{"입력 메시지 타입"}
    C -->|"sensor_msgs/Image"| D["CvBridge.imgmsg_to_cv2()<br/>BGR frame"]
    C -->|"sensor_msgs/CompressedImage"| E["cv2.imdecode()<br/>BGR frame"]
    D --> F["LaneProcessor.process(frame)"]
    E --> F

    F --> G{"ENABLE_UNDISTORT<br/>K, DIST_COEFF 유효?"}
    G -->|"yes"| H["cv2.undistort()"]
    G -->|"no"| I["원본 frame 사용"]
    H --> J["utils.to_gray()"]
    I --> J

    J --> K["utils.bilateralmsg()<br/>노이즈 완화"]
    K --> L["utils.canny_edge()<br/>Canny edge"]
    L --> M["utils.birds_eye_view()<br/>IPM/BEV 변환"]
    M --> N["utils.compute_bev_black_rows()<br/>첫 프레임 하단 검은 영역 자동 계산"]
    N --> O["BEV crop / SW_VISION_RATIO / BEV_SCALE 적용"]
    O --> P["sliding_window2.slidingmsg2()<br/>차선 fit + 중심선 ptsC 검출"]

    P --> Q["fit/center/window 좌표를<br/>원본 BEV 좌표계로 복원"]
    Q --> R["utils.draw_lane_on_bev()<br/>lane_fitting 시각화"]
    Q --> S{"PUBLISH_DEBUG_IMAGES"}
    S -->|"true"| T["utils.project_bev_to_camera()<br/>final_result 생성"]
    S -->|"false"| U["final_result 생략"]

    R --> V["LaneNode publishes debug images"]
    T --> V
    U --> V
    Q --> W{"pts_line 존재?"}
    W -->|"yes"| X["ros_utils.create_path_msg()<br/>픽셀 좌표 -> meter Path"]
    W -->|"no"| Y["빈 nav_msgs/Path 생성"]
    X --> Z["/vision/lane_path<br/>/vision/left_lane<br/>/vision/right_lane publish"]
    Y --> Z
```

## 2. ROS 런타임 시퀀스 다이어그램

```mermaid
sequenceDiagram
    autonumber
    participant Launch as vision.launch
    participant ROS as ROS Parameter Server
    participant Node as LaneNode<br/>(main.py)
    participant Dyn as dynamic_reconfigure<br/>Server
    participant Camera as Camera Topic
    participant Bridge as CvBridge / OpenCV
    participant Processor as LaneProcessor<br/>(process.py)
    participant Utils as utils.py
    participant SW as sliding_window2.py
    participant RosUtils as ros_utils.py
    participant Pub as ROS Publishers

    Launch->>ROS: set /driving_mode = sim | real
    Launch->>Node: start vision_pkg/main.py
    Node->>Node: rospy.init_node("lane_node")
    Node->>Processor: LaneProcessor()
    Node->>ROS: get_param("/driving_mode", "real")

    alt mode == "sim"
        Node->>Node: cfg.IMAGE_TOPIC = "/image_jpeg/compressed"
        Node->>Node: cfg.K = None, cfg.DIST_COEFF = zeros
    else mode == "real"
        Node->>Node: cfg.K = cfg.K_REAL
        Node->>Node: cfg.DIST_COEFF = cfg.DIST_COEFF_REAL
    end

    Node->>Dyn: Server(VisionConfig, dyn_callback)
    Node->>Dyn: update_configuration(initial_config)
    Dyn->>Node: dyn_callback(config, level)
    Node->>Node: cfg 전역 파라미터 갱신

    Node->>Pub: advertise debug image topics
    Node->>Pub: advertise Path topics
    Node->>Camera: subscribe cfg.IMAGE_TOPIC

    loop every image frame
        Camera-->>Node: Image or CompressedImage

        alt compressed topic
            Node->>Bridge: np.frombuffer() + cv2.imdecode()
            Bridge-->>Node: BGR frame
        else raw Image topic
            Node->>Bridge: imgmsg_to_cv2(..., "bgr8")
            Bridge-->>Node: BGR frame
        end

        Node->>Processor: process(frame)

        opt real camera undistortion enabled
            Processor->>Processor: cv2.undistort(frame, cfg.K, cfg.DIST_COEFF)
        end

        Processor->>Utils: to_gray(frame)
        Utils-->>Processor: gray_img
        Processor->>Utils: bilateralmsg(gray_img)
        Utils-->>Processor: gaussian_img
        Processor->>Utils: canny_edge(gaussian_img, cfg.CANNY_LOW, cfg.CANNY_HIGH)
        Utils-->>Processor: canny_img
        Processor->>Utils: birds_eye_view(canny_img)
        Utils-->>Processor: bev_img, M, Minv

        opt first processed frame
            Processor->>Utils: compute_bev_black_rows(frame width, frame height)
            Utils-->>Processor: black row count
            Processor->>Processor: cfg.SW_BOTTOM_OFFSET 자동 설정
        end

        Processor->>Processor: crop BEV bottom black area
        Processor->>Processor: apply SW_VISION_RATIO and BEV_SCALE
        Processor->>SW: slidingmsg2(bev_small)
        SW-->>Processor: left_fit, right_fit, out_img, ptsC, l_yr, r_yr, windows

        Processor->>Processor: fit/yr/pts/window 좌표를 full BEV로 remap
        Processor->>Utils: draw_lane_on_bev(out_img, left_fit, right_fit, ptsC, l_yr, r_yr)
        Utils-->>Processor: lane_fitting vis

        opt cfg.PUBLISH_DEBUG_IMAGES == true
            Processor->>Utils: project_bev_to_camera(frame, overlay_vis, Minv)
            Utils-->>Processor: final_img
        end

        Processor-->>Node: results dict
        Node->>Pub: publish /vision/lane_fitting

        opt cfg.PUBLISH_DEBUG_IMAGES == true
            Node->>Pub: publish /vision/bev
            Node->>Pub: publish /vision/final_result
            Node->>Pub: publish /vision/canny
            Node->>Pub: publish /vision/sliding_window
        end

        alt pts_line exists
            Node->>RosUtils: create_path_msg(pts_line, bev_shape)
            RosUtils-->>Node: nav_msgs/Path
            Node->>Pub: publish /vision/lane_path
            Node->>RosUtils: create_path_msg(pts_left, bev_shape)
            RosUtils-->>Node: nav_msgs/Path
            Node->>Pub: publish /vision/left_lane
            Node->>RosUtils: create_path_msg(pts_right, bev_shape)
            RosUtils-->>Node: nav_msgs/Path
            Node->>Pub: publish /vision/right_lane
        else no lane detected
            Node->>Node: create empty Path(frame_id="stier")
            Node->>Pub: publish empty /vision/lane_path, /vision/left_lane, /vision/right_lane
        end
    end
```

## 3. sliding_window2 내부 시퀀스 다이어그램

```mermaid
sequenceDiagram
    autonumber
    participant Processor as LaneProcessor.process()
    participant SW as slidingmsg2()
    participant Track as search_around_poly()
    participant ResetL as sliding_window_left()
    participant ResetR as sliding_window_right()
    participant Gates as validity gates
    participant Center as compute_and_draw_centerline()
    participant State as module global state

    Processor->>SW: slidingmsg2(binary_warped)
    SW->>State: read prev_left_fit, prev_right_fit, lost counts, dynamic_mid_x
    SW->>SW: _update_dynamic_mid_from_history(h, w)

    alt previous fit exists
        SW->>Track: search_around_poly(binary_warped, prev_left_fit, prev_right_fit)
        Track->>Track: 이전 fit 주변 tapered margin으로 픽셀 검색
        Track->>Track: _clip_y_gap()로 먼 쪽 끊긴 픽셀 제거
        Track->>Gates: invalidate_fit_if_bottom_cross()
        Track-->>SW: lf, rf, out_trk, lc, rc, l_yr, r_yr
    else no previous fit
        SW->>SW: out_base 사용, lf/rf 없음
    end

    SW->>State: update lost_count_left/right

    alt left lost_count >= MAX_LOST
        SW->>ResetL: sliding_window_left(binary_warped)
        ResetL->>ResetL: 하단 ROI에서 left 시작 x 추정
        ResetL->>ResetL: 아래에서 위로 window 탐색
        ResetL->>Gates: invalidate_fit_if_bottom_cross(side="left")
        ResetL-->>SW: lf2, outL, count, l_yr2, winL
        SW->>SW: 성공 시 lf 갱신, lost_count_left = 0
    end

    alt right lost_count >= MAX_LOST
        SW->>ResetR: sliding_window_right(binary_warped)
        ResetR->>ResetR: 하단 ROI에서 right 시작 x 추정
        ResetR->>ResetR: 아래에서 위로 window 탐색
        ResetR->>Gates: invalidate_fit_if_bottom_cross(side="right")
        ResetR-->>SW: rf2, outR, count, r_yr2, winR
        SW->>SW: 성공 시 rf 갱신, lost_count_right = 0
    end

    SW->>Gates: min_width_pair_gate(lf, rf, w, y_ref)
    Gates-->>SW: 너무 좁거나 한쪽으로 몰린 fit 제거
    SW->>Gates: swap_guard(lf, rf, h)
    Gates-->>SW: 좌우 역전 fit 제거

    SW->>State: apply fit hold when temporary loss
    SW->>State: prev_left_fit = smooth_fit(prev_left_fit, lf)
    SW->>State: prev_right_fit = smooth_fit(prev_right_fit, rf)
    SW->>State: update prev_l_yr, prev_r_yr

    SW->>Center: compute_and_draw_centerline(out_img, prev_left_fit, prev_right_fit, l_yr, r_yr)
    Center->>Center: both lanes -> center = (left + right) / 2
    Center->>Center: single lane -> offset by SINGLE_SIDE_OFFSET_M
    Center-->>SW: ptsC, xc, yc

    SW->>State: update prev_center_mid_x and dynamic_mid_x
    SW-->>Processor: prev_left_fit, prev_right_fit, out_img, ptsC, l_yr, r_yr, windows
```

## 4. 클래스/모듈 다이어그램

```mermaid
classDiagram
    direction LR

    class LaneNode {
        +CvBridge bridge
        +LaneProcessor processor
        +str mode
        +bool is_compressed
        +Server srv
        +Publisher pub_canny
        +Publisher pub_bev
        +Publisher pub_sw
        +Publisher pub_fit
        +Publisher pub_final
        +Publisher pub_path
        +Publisher pub_left_lane
        +Publisher pub_right_lane
        +__init__()
        +dyn_callback(config, level)
        +callback(msg)
        +run()
    }

    class LaneProcessor {
        +__init__()
        +process(frame) dict
        +_scale_poly(fit, inv)$
        +_scale_yr(yr, inv)$
    }

    class Config {
        <<module>>
        IMAGE_TOPIC
        CAMERA_HEIGHT_M
        CAMERA_PITCH_DEG
        CAMERA_FOV_DEG
        K_REAL
        DIST_COEFF_REAL
        K
        DIST_COEFF
        ENABLE_UNDISTORT
        M_PER_PIXEL
        BEV_WIDTH_M
        BEV_LENGTH_M
        BEV_IMAGE_WIDTH
        BEV_IMAGE_HEIGHT
        BEV_SCALE
        CANNY_LOW
        CANNY_HIGH
        N_WINDOWS
        SW_MARGIN
        MINPIX
        MINLANEPIX
        SMOOTH_ALPHA
        PUBLISH_DEBUG_IMAGES
    }

    class Utils {
        <<module>>
        +to_gray(frame)
        +bilateralmsg(img)
        +canny_edge(img, low_threshold, high_threshold)
        +compute_ipm_matrix(w, h)
        +birds_eye_view(img, pad)
        +compute_bev_black_rows(w_img, h_img)
        +draw_lane_on_bev(roi_img, left_fit, right_fit, ptsC, l_yr, r_yr)
        +project_bev_to_camera(frame_bgr, bev_vis, minv)
    }

    class SlidingWindow2 {
        <<module>>
        prev_left_fit
        prev_right_fit
        lost_count_left
        lost_count_right
        prev_lane_width_px
        dynamic_mid_x
        prev_center_mid_x
        +slidingmsg2(binary_warped)
        +search_around_poly(binary_warped, left_fit, right_fit)
        +sliding_window_left(binary_warped)
        +sliding_window_right(binary_warped)
        +compute_and_draw_centerline(vis, left_fit, right_fit, l_yr, r_yr)
        +invalidate_fit_if_bottom_cross(fit, y_pixels, w, h, side)
        +min_width_pair_gate(left_fit, right_fit, w, y_ref)
        +swap_guard(left_fit, right_fit, h)
        +smooth_fit(prev_fit, cur_fit, alpha)
    }

    class RosUtils {
        <<module>>
        +create_path_msg(pts_line, bev_shape) Path
    }

    class ConfigManager {
        <<module>>
        +save_config_to_files(rqt_config)
        -_regex_replace_file(filepath, values_map, pattern_replacement_func)
    }

    class VisionConfig {
        <<dynamic_reconfigure cfg>>
        canny_low
        canny_high
        camera_height
        camera_pitch
        camera_fov
        bev_length
        bev_width
        n_windows
        sw_margin
        minpix
        minpix_bottom
        max_gap_windows
        sw_bottom_offset
        minlanepix
        poly_order
        reset_interval
        lane_width_px
        min_lane_width_px
        smooth_alpha
        sw_pred_amp
        sw_curve_ratio
        publish_debug_images
        save_config
    }

    class Path {
        <<nav_msgs/Path>>
        header
        poses
    }

    class ImageMsg {
        <<sensor_msgs/Image>>
    }

    class CompressedImageMsg {
        <<sensor_msgs/CompressedImage>>
    }

    LaneNode *-- LaneProcessor : owns
    LaneNode ..> VisionConfig : dynamic_reconfigure
    LaneNode ..> Config : read/write cfg
    LaneNode ..> ConfigManager : save_config
    LaneNode ..> RosUtils : create Path
    LaneNode ..> ImageMsg : subscribe/publish
    LaneNode ..> CompressedImageMsg : subscribe
    LaneNode ..> Path : publish

    LaneProcessor ..> Config : reads params
    LaneProcessor ..> Utils : preprocessing/BEV/rendering
    LaneProcessor ..> SlidingWindow2 : lane detection

    SlidingWindow2 ..> Config : reads params
    RosUtils ..> Config : M_PER_PIXEL
    Utils ..> Config : camera/IPM params
    ConfigManager ..> Config : updates config.py
    ConfigManager ..> VisionConfig : updates Vision.cfg defaults
```

## 5. 토픽 입출력 요약

```mermaid
flowchart LR
    subgraph Input
        A1["real 기본 입력<br/>/image_raw<br/>sensor_msgs/Image"]
        A2["sim 입력<br/>/image_jpeg/compressed<br/>sensor_msgs/CompressedImage"]
    end

    subgraph Node["vision_node / lane_node"]
        B["LaneNode"]
        C["LaneProcessor"]
        D["sliding_window2"]
        B --> C --> D
    end

    subgraph DebugImages["Debug image outputs"]
        E1["/vision/canny<br/>sensor_msgs/Image mono8"]
        E2["/vision/bev<br/>sensor_msgs/Image mono8"]
        E3["/vision/sliding_window<br/>sensor_msgs/Image bgr8"]
        E4["/vision/lane_fitting<br/>sensor_msgs/Image bgr8"]
        E5["/vision/final_result<br/>sensor_msgs/Image bgr8"]
    end

    subgraph Paths["Path outputs"]
        F1["/vision/lane_path<br/>nav_msgs/Path"]
        F2["/vision/left_lane<br/>nav_msgs/Path"]
        F3["/vision/right_lane<br/>nav_msgs/Path"]
    end

    A1 --> B
    A2 --> B
    B --> E1
    B --> E2
    B --> E3
    B --> E4
    B --> E5
    B --> F1
    B --> F2
    B --> F3
```

## 6. 코드 기준 핵심 포인트

- `main.py`: ROS 노드 초기화, `driving_mode`에 따른 입력 토픽/카메라 파라미터 선택, dynamic_reconfigure 콜백, 이미지/Path 퍼블리시를 담당합니다.
- `process.py`: 프레임 1장을 받아 왜곡 보정, grayscale, bilateral filter, Canny, BEV, crop/scale, sliding window, 시각화, 결과 dict 생성을 담당합니다.
- `sliding_window2.py`: 이전 프레임 fit 기반 tracking을 먼저 시도하고, 실패하면 좌/우 sliding window reset을 수행합니다. fit 검증, hold, smoothing, 중심선 계산까지 포함합니다.
- `utils.py`: OpenCV 기반 전처리, IPM homography 계산, BEV 변환, BEV 시각화, 카메라 이미지 재투영을 제공합니다.
- `ros_utils.py`: BEV 픽셀 중심선을 `nav_msgs/Path`의 meter 좌표로 변환합니다. 좌표계는 차량 기준 전방 `x`, 좌측 `y`, frame_id는 `"stier"`입니다.
- `config.py`: 대부분의 런타임 파라미터를 전역 변수로 관리합니다. RQT 변경은 `main.py`의 `dyn_callback()`을 통해 이 값들에 반영됩니다.
- `config_manager.py`: RQT에서 `save_config`가 켜지면 현재 파라미터를 `src/config.py`와 `cfg/Vision.cfg`에 정규식으로 저장합니다.
