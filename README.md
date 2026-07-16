# Jejudo Vision Package

## Jeju Autonomous Driving Competition

### Vision-Based Lane Perception & Local Path Generation

전방 카메라 영상에서 차선을 검출하고, 차량 제어기가 추종할 수 있는 로컬 경로를 생성하는 ROS 기반 비전 패키지입니다.

1/5 스케일 자율주행 차량의 제한된 연산 환경을 고려하여 딥러닝 모델 대신 OpenCV 기반의 전통적인 영상처리 방식을 사용했습니다.

---

## 1. Project Overview

- **Period:** 2026.01.01 – 2026.03.27
- **Type:** Team Project / Autonomous Driving Competition
- **My Role:** Vision Perception & Local Path Generation
- **Environment:** ROS1, Ubuntu 20.04, Python, OpenCV
- **Output:** `nav_msgs/Path`
- **Average Processing Speed:** 약 80 FPS

### Project Goal

전방 카메라 영상에서 좌·우 차선을 검출하고 차로 중심을 계산하여, 차량 제어 모듈이 추종할 수 있는 로컬 경로를 실시간으로 생성하는 것을 목표로 했습니다.

대회 차량은 1/5 스케일 플랫폼이었으며, 차량 위에 노트북을 탑재하여 인지, 경로 생성 및 제어 알고리즘을 실행했습니다.

노트북의 제한된 연산 성능에서 딥러닝 기반 차선 인식을 사용할 경우 처리 속도 저하와 추론 지연이 발생할 수 있다고 판단했습니다. 따라서 실시간성을 우선하여 OpenCV 기반의 차선 검출 파이프라인을 설계했습니다.

---

## 2. My Contribution

다음 기능을 직접 설계하고 구현했습니다.

- 카메라 영상 입력 및 ROS 이미지 토픽 구독
- 카메라 왜곡 보정 및 IPM 기반 Bird's-Eye View 변환
- Canny Edge 기반 차선 후보 추출
- Sliding Window 기반 좌·우 차선 검출
- Search Around Polynomial 기반 이전 프레임 차선 추적
- 추적 실패 시 Sliding Window 전역 재탐색
- 좌·우 차선 및 단일 차선 상황에서의 중심 경로 생성
- 이전 경로 기반 동적 중심점 생성
- 좌·우 차선 역전 및 비정상 차선 폭 검증
- 차선 소실 시 이전 차선 fitting 결과 유지
- EMA smoothing을 통한 경로 흔들림 완화
- 칼만 필터 기반 경로 smoothing 적용 및 한계 분석
- 픽셀 좌표를 차량 기준 미터 좌표로 변환
- `nav_msgs/Path` 메시지 생성 및 ROS 토픽 발행
- RQT Dynamic Reconfigure 기반 실시간 파라미터 조정 환경 구성

---

## 3. System Architecture

```text
Camera Image
    ↓
Camera Undistortion
    ↓
Canny Edge Detection
    ↓
IPM / Bird's-Eye View
    ↓
Lane Pixel Extraction
    ↓
Sliding Window / Search Around Polynomial
    ↓
Lane Validation
    ↓
Center Path Generation
    ↓
Outlier Validation & Temporal Smoothing
    ↓
Metric Coordinate Conversion
    ↓
ROS nav_msgs/Path Publishing
```

---

## 4. Key Implementation

### 4.1 Camera Undistortion

카메라 캘리브레이션을 통해 얻은 내부 파라미터와 왜곡 계수를 사용하여 렌즈 왜곡을 보정했습니다. 영상 가장자리에서 휘어져 보이는 차선 형상을 보정하여 BEV 변환과 차선 fitting의 정확도를 높였습니다.

### 4.2 IPM-Based BEV Transformation

카메라의 외부 파라미터와 FOV를 이용하여 IPM 기반 Bird's-Eye View를 생성했습니다.

원근 영상에서는 멀리 있는 차선의 간격이 좁아지고 차선의 기하학적 형태가 왜곡됩니다. 이를 위에서 내려다보는 형태의 BEV 영상으로 변환하여 차선을 안정적으로 검출하고, 픽셀 좌표를 실제 거리 좌표로 변환할 수 있도록 구성했습니다.

### 4.3 Edge-Based Lane Detection

전처리된 영상에 Canny Edge Detection을 적용하여 차선 경계 후보를 추출했습니다. 딥러닝 모델을 사용하지 않고 영상의 밝기와 경계 정보를 이용하여 제한된 연산 환경에서도 높은 처리 속도로 차선 검출을 수행했습니다.

### 4.4 Sliding Window Lane Detection

초기 프레임이거나 이전 차선 추적에 실패한 경우, BEV 영상 하단에서 차선 후보 픽셀의 시작점을 찾은 뒤 Sliding Window 방식으로 차선 픽셀을 추적했습니다.

좌측과 우측 탐색 영역을 분리하고 각 윈도우 내부의 차선 픽셀을 수집하여 다항식 기반으로 차선을 fitting했습니다.

### 4.5 Search Around Polynomial

매 프레임마다 전체 영역에서 차선을 다시 검출하면 불필요한 연산이 발생하고, 그림자나 도로 경계 등의 노이즈를 차선으로 선택할 가능성이 커집니다.

이를 줄이기 위해 이전 프레임에서 계산된 차선 다항식 주변을 우선 탐색하는 Search Around Polynomial 방식을 적용했습니다. 이전 차선 주변에서 충분한 픽셀이 검출되면 해당 결과를 새로운 차선으로 사용하고, 추적에 실패한 경우에만 Sliding Window 기반 전역 재탐색을 수행했습니다.

### 4.6 Dynamic Center-Based Lane Association

초기에는 영상의 고정된 중앙값을 기준으로 왼쪽 차선과 오른쪽 차선을 구분했습니다. 그러나 급커브를 통과한 후 차량이 새로운 차선 방향으로 진입하는 과정에서 차선이 영상 중앙을 넘어가면 좌·우 차선의 관계가 반대로 판단되는 문제가 발생했습니다.

이를 해결하기 위해 이전 프레임에서 생성된 중심 경로의 하단점 중 차량과 가장 가까운 점을 추출하고, 이를 다음 프레임의 동적 중심점으로 사용했습니다.

동적 중심점을 기준으로 다음 조건을 검증했습니다.

- 왼쪽 차선이 중심점 오른쪽으로 넘어갔는지 여부
- 오른쪽 차선이 중심점 왼쪽으로 넘어갔는지 여부
- 좌·우 차선의 위치가 서로 역전되었는지 여부
- 두 차선 사이의 폭이 정상 범위에 포함되는지 여부
- 두 차선이 한쪽 영역에 비정상적으로 몰려 있는지 여부

이를 통해 고정된 영상 중심이 아니라 차량의 이전 진행 방향과 중심 경로를 반영하여 좌·우 차선의 연관성을 유지했습니다.

### 4.7 Lane Loss Recovery

강한 조명, 그림자, 급커브 및 훼손된 차선으로 인해 일부 프레임에서 차선 픽셀이 충분히 검출되지 않는 경우가 있었습니다. 프레임별 검출 결과만 사용하면 짧은 차선 소실에도 중심 경로가 즉시 사라지거나 크게 흔들릴 수 있습니다.

이를 완화하기 위해 다음 복구 로직을 적용했습니다.

- 이전 차선 fitting 결과 주변 우선 탐색
- 추적 실패 시 Sliding Window 기반 전역 재탐색
- 최소 차선 픽셀 수 검증
- 좌·우 차선 폭 및 역전 여부 검증
- 짧은 차선 소실 시 이전 fitting 결과를 최대 2프레임 유지
- EMA smoothing을 이용한 시간적 연속성 확보

### 4.8 Single-Lane Path Estimation

한쪽 차선만 검출되는 경우에도 경로 생성을 중단하지 않도록 구성했습니다. 검출된 차선과 사전에 정의하거나 이전 프레임에서 추정한 차선 폭을 이용하여 차로 중심 위치를 추정했습니다.

이를 통해 급커브나 부분적인 차선 소실 상황에서도 중심 경로를 지속적으로 생성할 수 있도록 했습니다.

### 4.9 Center Path Generation

좌·우 차선이 모두 검출된 경우 두 차선의 중간점을 이용하여 중심 경로를 생성했습니다. 한쪽 차선만 검출된 경우에는 차선 폭에 해당하는 오프셋을 적용하여 중심 경로를 추정했습니다.

생성된 중심 경로에는 EMA smoothing을 적용하여 프레임 간 작은 위치 변화를 완화하고, 제어 모듈에 전달되는 경로의 시간적 연속성을 확보했습니다.

### 4.10 Metric Coordinate Conversion and ROS Path Publishing

BEV 영상에서 생성된 픽셀 경로를 BEV 스케일 정보를 이용하여 차량 기준의 미터 좌표로 변환했습니다.

변환된 경로점을 `geometry_msgs/PoseStamped` 배열로 구성하고, 이를 `nav_msgs/Path` 메시지로 생성하여 ROS 토픽으로 발행했습니다. 이를 통해 비전 패키지와 차량 제어 모듈을 분리하고 표준 ROS 메시지로 연결했습니다.

### 4.11 Real-Time Parameter Tuning

RQT Dynamic Reconfigure를 이용하여 주행 중 차선 검출 파라미터를 실시간으로 변경할 수 있도록 구성했습니다.

- Canny Edge 임계값
- ROI 및 BEV 변환 범위
- Sliding Window 크기
- 탐색 마진
- 최소 차선 픽셀 수
- 차선 폭 허용 범위
- 차선 fitting 유지 조건
- smoothing 계수

이를 통해 코드를 다시 실행하지 않고도 실제 조명과 노면 상태에 맞게 차선 검출 조건을 조정할 수 있었습니다.

---

## 5. Problems & Solutions

### 5.1 Strong Sunlight and Reflection

#### Problem

야외 주행 환경의 강한 햇빛과 노면 반사로 인해 영상 일부가 과도하게 밝아졌습니다. 이로 인해 차선과 노면의 밝기 차이가 감소하고, 반사 영역의 경계가 차선 후보로 검출될 가능성이 있었습니다.

#### Solution

카메라 전면에 UV 필터를 부착하여 햇빛과 노면 반사의 영향을 줄였습니다. 또한 Dynamic Reconfigure 환경을 구성하여 조명 상태에 따라 Canny 임계값과 검출 조건을 실시간으로 조정했습니다.

#### Limitation

UV 필터는 반사와 과도한 밝기를 줄이는 데 도움을 주지만 그림자나 급격한 노출 변화에 대한 근본적인 해결책은 아닙니다. 향후에는 자동 노출 제어, 색 공간 기반 차선 분리, 명암 정규화 및 편광 필터 등을 함께 검토할 필요가 있습니다.

### 5.2 Temporary Lane Loss

#### Problem

강한 조명, 그림자, 급커브 또는 훼손된 차선으로 인해 일부 프레임에서 차선 픽셀이 부족해졌습니다. 프레임별 검출 결과만 사용하면 차선이 짧게 소실되었을 때도 중심 경로가 즉시 사라지거나 크게 흔들렸습니다.

#### Solution

이전 프레임의 차선 fitting 결과 주변을 우선 탐색하는 Search Around Polynomial 방식을 적용했습니다. 추적에 실패한 경우에는 좌·우 영역을 분리한 Sliding Window 기반 전역 재탐색을 수행했습니다.

최소 차선 픽셀 수, 좌·우 차선 폭, 좌·우 위치 및 역전 여부를 검증하여 잘못된 fitting을 제거했습니다. 짧은 차선 소실 상황에서는 이전 fitting 결과를 최대 2프레임 동안 유지했으며, 한쪽 차선만 검출된 경우에는 차선 폭 오프셋을 이용해 중심 경로를 계속 생성했습니다.

### 5.3 Left/Right Lane Misclassification

#### Problem

초기에는 영상 가운데를 고정 기준점으로 사용하여 왼쪽과 오른쪽 차선을 구분했습니다. 그러나 급커브를 통과한 후 차량이 새로운 방향으로 진입하는 과정에서 차선 위치가 영상 중앙을 넘어가면 좌·우 차선이 반대로 인식되는 문제가 발생했습니다.

#### Solution

이전 프레임에서 생성된 중심 경로의 가장 가까운 하단점을 추출하고 이를 다음 프레임의 동적 중심점으로 사용했습니다.

동적 중심점을 기준으로 좌·우 차선 위치, 좌우 역전 여부, 차선 폭 및 한쪽 영역 집중 여부를 검증하여 비정상 fitting을 제거했습니다. 이를 통해 차량의 이전 진행 방향을 반영한 기준점에서 프레임 사이의 좌·우 차선 연관성을 유지했습니다.

### 5.4 Path Jump and Kalman Filter Limitation

#### Problem

노이즈나 잘못된 차선 검출로 인해 생성된 중심 경로가 순간적으로 크게 이탈하는 문제가 발생했습니다. 초기에는 이를 줄이기 위해 칼만 필터를 적용하여 경로를 시간적으로 smoothing하는 방식을 시도했습니다.

#### Attempt and Result

칼만 필터를 통해 연속 프레임 사이의 작은 위치 변화와 측정 노이즈를 완화하려고 했습니다.

그러나 문제의 핵심은 정상 경로 주변의 작은 노이즈가 아니라 도로 경계나 그림자를 차선으로 오인하여 완전히 잘못된 경로가 생성되는 것이었습니다.

칼만 필터는 입력된 경로를 기반으로 상태를 추정하기 때문에 잘못된 경로 자체를 판별하거나 계산에서 자동으로 제외하지 못했습니다. 따라서 크게 이탈한 경로가 측정값으로 입력되면 필터 결과도 해당 이상치의 영향을 받았고, 기대했던 만큼의 개선 효과를 얻지 못했습니다.

#### Lesson Learned

칼만 필터의 성능보다 smoothing 이전에 경로의 유효성을 검증하는 단계가 부족했던 것이 핵심 원인이었습니다.

다음 조건으로 비정상 경로를 먼저 제거했어야 합니다.

- 이전 경로 대비 횡방향 위치 변화
- 이전 경로 대비 heading 변화
- 경로의 기울기 및 곡률 변화
- 좌·우 차선 폭의 변화
- 차선 fitting에 사용된 픽셀 수
- 연속 프레임에서의 검출 신뢰도
- 차량의 속도와 조향 범위에서 물리적으로 가능한 경로인지 여부

더 적절한 처리 순서는 다음과 같습니다.

```text
Lane Detection
    ↓
Path Generation
    ↓
Path Validity Check
    ↓
Outlier Rejection
    ↓
Kalman Filter or EMA Smoothing
    ↓
Path Publishing
```

향후에는 이전 경로에서 일정 거리 이상 이탈하거나 곡률이 비정상적으로 변화한 경로를 먼저 제거하고, 검증을 통과한 경로에만 칼만 필터 또는 EMA smoothing을 적용할 계획입니다.

---

## 6. Results

- OpenCV 기반 차선 검출 및 로컬 경로 생성 파이프라인 구현
- 제한된 노트북 연산 환경에서 약 80 FPS의 처리 속도 확보
- 좌·우 차선이 모두 보이는 상황에서 중심 경로 생성
- 한쪽 차선만 보이는 상황에서도 중심 경로 추정
- 짧은 차선 소실 상황에서 이전 차선 정보 기반 경로 유지
- 급커브 진입 시 동적 중심점을 이용한 좌·우 차선 연관성 유지
- 픽셀 기반 경로를 차량 기준 미터 좌표로 변환
- `nav_msgs/Path` 기반 비전–제어 모듈 연결
- RQT Dynamic Reconfigure 기반 실시간 파라미터 튜닝 환경 구축

### Demo

실행 GIF 또는 주행 영상을 아래에 추가할 예정입니다.

```markdown
![Lane Detection Demo](assets/lane_detection_demo.gif)
```

### Performance

| Item | Result |
|---|---:|
| Processing Speed | 약 80 FPS |
| Path Output | `nav_msgs/Path` |
| Lane Detection | Left / Right / Single Lane |
| Recovery | Previous Fit + Global Re-detection |
| Parameter Tuning | RQT Dynamic Reconfigure |

> FPS에는 향후 입력 영상 해상도, 노트북 사양 및 측정 구간을 함께 기록할 예정입니다.

---

## 7. Limitations & Lessons Learned

### 7.1 Independent Mode Switching Between Vision and LiDAR

전체 주행 시스템은 차선 기반의 Vision Mode와 장애물 기반의 LiDAR Mode를 분리하여 운영했습니다. 전방 LiDAR 클러스터의 수가 일정 기준을 넘으면 LiDAR Mode로 전환하고, 그 외 구간에서는 Vision Mode로 주행하는 방식이었습니다.

그러나 차선이 없는 라바콘 구간에서도 비전 노드는 계속 동작했습니다. 이 과정에서 도로 경계와 노이즈가 차선으로 잘못 검출되었고, 잘못된 차선 결과가 이전 fitting 결과로 유지되는 문제가 발생했습니다.

이 상태에서 Vision Mode로 전환되면 잘못된 좌·우 차선 정보가 경로 생성에 사용되어 차량이 주행 경로를 이탈할 수 있었습니다.

반대로 차선 구간에 라바콘이 들어오는 동적 장애물 상황에서는 LiDAR Mode로 전환되면서 정지보다 장애물 회피가 우선되는 문제도 확인했습니다.

이를 통해 센서별 주행 모드를 완전히 분리하고 단순한 조건으로 전환하는 구조는 복합적인 주행 상황에 대한 범용성이 부족하다는 점을 확인했습니다.

향후에는 비전 기반 차선 위치와 신뢰도, LiDAR 기반 장애물 위치, 장애물과 주행 차로의 관계, 이전 경로 및 차량 상태를 동시에 고려하는 센서 융합 구조가 필요합니다.

### 7.2 Limited Camera Look-Ahead Distance

카메라가 노면 방향으로 과도하게 기울어져 있어 최대 인식 거리가 약 5 m로 제한되었습니다.

차량의 최고 속도가 약 15 km/h였기 때문에 근거리 차선 정보만으로도 제어가 가능하다고 판단했습니다. 그러나 제어기는 차량 전방 약 2 m에 위치한 경로점을 추종하도록 구성되어 있었습니다.

대회 코스에는 일반적인 연속 곡선이 아니라 짧은 직선 구간이 급격하게 연결되는 형태의 커브가 포함되어 있었습니다. 따라서 짧은 전방 주시 거리와 고정된 추종점만으로는 조향 변화에 선제적으로 대응하기 어려웠습니다.

향후에는 카메라 설치 각도를 조정하여 더 긴 전방 경로를 확보하고, 차량 속도와 경로 곡률에 따라 look-ahead distance를 동적으로 변경하는 방식을 적용할 필요가 있습니다.

- 저속 및 급커브: 짧은 look-ahead distance 적용
- 고속 및 완만한 커브: 긴 look-ahead distance 적용
- 경로 곡률과 차량 속도를 동시에 고려한 추종점 생성
- 여러 전방 경로점을 이용한 조향 명령 계산

### 7.3 Hardware Weight and Durability

카메라, LiDAR, GPS 및 노트북을 장착하기 위해 알루미늄 프로파일 기반의 센서 거치대를 제작했습니다. 그러나 완성된 구조물의 무게가 1/5 스케일 차량의 허용 하중에 비해 지나치게 컸습니다.

초기 주행에서는 문제가 뚜렷하게 나타나지 않았지만 반복 주행 과정에서 차량의 차고가 점차 낮아지고 조향 응답이 느려졌습니다. 최종적으로는 조향 기어가 파손되어 대회를 완주하지 못했습니다.

대회 직전에 문제를 확인했지만 새로운 프로파일과 거치 구조를 제작하기에는 시간이 부족했습니다. 소프트웨어 성능 검증에 집중한 나머지 다음 항목에 대한 사전 평가가 부족했습니다.

- 차량의 최대 허용 하중
- 센서 및 노트북을 포함한 전체 시스템 중량
- 센서 배치에 따른 무게중심 변화
- 서스펜션 변형
- 조향 모터와 조향 기어에 전달되는 부하
- 반복 주행에 따른 기구부 내구성

이를 통해 자율주행 시스템에서는 인지 및 제어 알고리즘뿐만 아니라 센서 배치, 전체 중량, 무게중심, 액추에이터 허용 하중과 기구부 내구성을 함께 검증해야 한다는 점을 배웠습니다.

### Overall Lesson

대회 주행 중 차선 구간에서는 비전 기반 차선 검출과 로컬 경로 생성이 안정적으로 동작했습니다. 그러나 전체 자율주행 시스템의 성능은 개별 인지 알고리즘의 정확도만으로 결정되지 않았습니다.

센서 간 정보 통합, 잘못된 인지 결과의 배제, 속도와 곡률에 따른 추종점 선정, 하드웨어의 중량과 내구성까지 함께 고려해야 안정적인 시스템을 구성할 수 있다는 점을 확인했습니다.

특히 다음 세 가지를 핵심적인 교훈으로 얻었습니다.

1. **Smoothing 이전에 잘못된 인지 결과를 제거해야 합니다.**
2. **센서를 독립된 모드로 분리하기보다 상호 보완적으로 사용해야 합니다.**
3. **소프트웨어와 하드웨어를 하나의 통합 시스템으로 평가해야 합니다.**

---

## 8. Future Work

### Perception

- 경로 smoothing 이전에 Outlier Rejection 단계 추가
- 이전 경로 대비 위치, heading 및 곡률 변화 검증
- 차선 검출 결과에 대한 신뢰도 점수 생성
- 조명 변화에 강한 색 공간 및 명암 정규화 적용
- 경량 학습 기반 차선 인식 모델의 실시간성 검증
- 딥러닝 기반 차선 검출과 기존 OpenCV 방식 비교

### Sensor Fusion

- Vision과 LiDAR 정보를 동시에 활용하는 센서 융합 구조 설계
- 차선과 장애물 정보를 함께 고려한 로컬 경로 생성
- 단순한 모드 전환 대신 센서 신뢰도 기반 가중치 적용
- 동적 장애물 상황에서 정지와 회피 판단 로직 개선

### Path Planning & Control

- 차량 속도에 따른 동적 look-ahead distance 적용
- 경로 곡률을 고려한 추종점 생성
- 경로의 횡방향 오차 및 heading error 정량 평가
- 급커브 구간에서의 조향 응답성 개선
- 경로 생성과 제어 알고리즘의 통합 검증

### Hardware

- 차량 허용 하중을 고려한 경량 센서 마운트 설계
- 센서 배치에 따른 무게중심 분석
- 조향부와 서스펜션의 반복 하중 시험
- 주행 전 하드웨어 체크리스트 및 내구성 평가 수행

### Simulation & Evaluation

- MORAI 환경에서 인지–경로계획–제어 통합 검증
- 실제 주행 데이터와 시뮬레이션 결과 비교
- 주행 성공률, 횡방향 오차 및 처리 지연 정량 평가
- 다양한 조명, 곡률 및 차선 소실 조건의 테스트 시나리오 구성

---

## 9. Repository Structure


```text
vision_stack/
├── .gitignore
└── src/
    └── vision_pkg/
        ├── CMakeLists.txt
        ├── package.xml
        ├── RQT_파라미터_설명.txt
        ├── Vision_Logic_Explanation.txt
        ├── cfg/
        │   └── Vision.cfg
        ├── config/
        │   └── vision_params.yaml
        ├── launch/
        │   └── vision.launch
        └── src/
            ├── main.py
            ├── process.py
            ├── sliding_window.py
            ├── lane.py
            ├── ros_utils.py
            ├── config.py
            ├── config_manager.py
            └── utils.py
```

---

## 10. How to Run

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch jejudo_vision_pkg vision.launch
```

### Published Topic

```bash
rostopic echo /local_path
```

| Topic | Message Type | Description |
|---|---|---|
| `/local_path` | `nav_msgs/Path` | 차량 기준 로컬 중심 경로 |

> 실제 토픽 이름과 launch 파일 이름에 맞게 수정해야 합니다.

---

## 11. Tech Stack

- ROS1
- Ubuntu 20.04
- Python
- OpenCV
- NumPy
- `sensor_msgs/Image`
- `nav_msgs/Path`
- `geometry_msgs/PoseStamped`
- RQT Dynamic Reconfigure
