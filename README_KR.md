# Isaac Sim Spot 원격 제어 데모

NVIDIA Isaac Sim에서 키보드 입력, 자동 실험 데이터 수집, 고급 시각화 도구를 사용하여 Boston Dynamics Spot 사족 보행 로봇을 제어하는 종합 시뮬레이션 프로젝트입니다.

> **English Version**: See [README.md](README.md).

## 개요

이 프로젝트는 Spot 로봇을 위한 완전한 시뮬레이션 환경을 제공하며 다음 기능을 포함합니다:
- **실시간 키보드 제어**: 부드러운 가속/감쇠 모델
- **자동 실험 데이터 수집**: CSV, 카메라 이미지, 설정 파일
- **다양한 객체 유형**: 다양한 작업 시나리오를 위한 박스, 구, 게이트
- **듀얼 카메라 시스템**: 동기화된 이미지 캡처를 위한 에고뷰 및 탑다운 카메라
- **대화형 시각화 도구**: 실험 결과 분석
- **환경 랜덤화**: 다양한 훈련 시나리오

## 주요 기능

### 핵심 시뮬레이션 (`quadruped_example.py`)
- 🤖 **Spot 로봇 제어**: Isaac Sim의 `SpotFlatTerrainPolicy`를 사용한 완전한 제어
- ⌨️ **키보드 컨트롤러**: 부드러운 속도 래핑을 갖춘 스레드 안전 키보드 입력
- 🎲 **환경 랜덤화**: 시작/목표 위치 및 객체의 구성 가능한 랜덤화
- 📦 **다양한 객체 유형**: 동적 박스, 구, 정적 게이트 지원
- 📊 **실시간 상태 추적**: 10Hz에서 로봇 및 객체 위치/방향
- 💾 **자동 데이터 저장**: CSV 로그, 카메라 이미지, 설정 파일
- 🎥 **듀얼 카메라 시스템**: 에고뷰(로봇 장착) 및 탑다운(오버헤드) 카메라
- 📏 **L1 거리 메트릭**: 객체 유형에 따른 작업 완료 추적
- 🎯 **작업 완료 감지**: 작업 완료 시 목표 색상 자동 변경

### 시각화 도구 (`plot_experiment.py`)
- 📈 **종합 플롯**: 궤적 맵, 속도 플롯, 거리 메트릭
- 🎨 **시간 그라데이션 시각화**: 시간적 진행을 보여주는 색상 코딩된 궤적
- 🖼️ **통합 이미지 뷰어**: 동기화된 에고 및 탑 카메라 이미지 표시
- 🎚️ **대화형 타임스탬프 슬라이더**: 실험 타임라인 탐색
- 🔄 **자동 최신 감지**: 디렉토리가 지정되지 않은 경우 가장 최근 실험 찾기

## 사전 요구사항

- **NVIDIA Isaac Sim** (버전 4.5.0 이상 또는 호환 버전)
- **Python 3.8+**
- Isaac Sim 패키지가 설치된 **Conda 환경**
- **필수 Python 패키지**: `pygame`, `numpy`, `pandas`, `matplotlib`, `pillow`

## 설치
0. 하드웨어 & NVIDIA 드라이버 & OS 
- 테스트 환경:
   - RTX 2060 12G
   - NVIDIA-SMI 535.216.01
   - Driver Version: 535.216.01
   - CUDA Version: 12.2  
   - Ubuntu 22.04.5 LTS

1. **Conda 환경 생성**:
   ```bash
   # 새로운 conda 환경 생성 (아직 생성하지 않은 경우)
   conda create -n isc-demo python=3.10 -y && conda activate isc-demo
   ```

2. **Python 의존성 및 Isaac Sim 4.1.0 설치**:
   ```bash
   # Isaac Sim 설치
   pip install isaacsim-rl==4.1.0 isaacsim-replicator==4.1.0 isaacsim-extscache-physics==4.1.0 isaacsim-extscache-kit-sdk==4.1.0 isaacsim-extscache-kit==4.1.0 isaacsim-app==4.1.0 --extra-index-url https://pypi.nvidia.com
   # 의존성 설치
   pip install torch==2.2.2 --index-url https://download.pytorch.org/whl/cu121 
   pip install pygame numpy pandas matplotlib pillow
   pip install tensordict==0.3.2
   ```

4. **설치 확인**:
   ```bash
   python -c "from isaacsim import SimulationApp; print('Isaac Sim ready')"
   ```

## 빠른 시작

### 시뮬레이션 실행

```bash
conda activate isc-demo
python quadruped_example.py
```

프롬프트가 나타나면 실험 이름을 입력하세요 (또는 'NULL'을 사용하려면 Enter를 누르세요):
```
Enter experiment name (press Enter for 'NULL'): my_experiment
```

### 결과 시각화

```bash
# 특정 실험 플롯
python plot_experiment.py expr_data/251118_185550-my_experiment

# 또는 최신 실험 사용 (프롬프트 시 Enter 누르기)
python plot_experiment.py
```

## 조작 방법

### 키보드 조작

| 키 | 동작 |
|-----|--------|
| `i` | 전진 (+x 방향) |
| `k` | 후진 (-x 방향) |
| `j` | 왼쪽 이동 (+y 방향) |
| `l` | 오른쪽 이동 (-y 방향) |
| `u` | 왼쪽 회전 (+yaw) |
| `o` | 오른쪽 회전 (-yaw) |
| `ESC` | 시뮬레이션 종료 |

### 제어 기능

- **부드러운 가속**: 키를 누르면 속도 명령이 점진적으로 증가
- **감쇠 모델**: 키를 놓으면 속도가 부드럽게 감소
- **구성 가능한 제한**: 최대 속도 및 가속률을 설정을 통해 조정 가능

## 주요 구성 요소

### `quadruped_example.py` - 메인 시뮬레이션

핵심 시뮬레이션 클래스 `SpotSimulation`은 다음을 제공합니다:

#### 주요 메서드

- `initialize()`: Isaac Sim 월드 및 스테이지 생성
- `setup()`: 완전한 시뮬레이션 설정 (환경, 로봇, 카메라, 컨트롤러)
- `run()`: 메인 시뮬레이션 루프
- `cleanup()`: 리소스 정리 및 파일 닫기

#### 기능

1. **환경 설정**
   - 경계 벽이 있는 구성 가능한 정사각형 환경
   - 시작 및 목표 마커 (빨간색 구와 파란색 반구)
   - 다양한 객체 유형 지원:
     - **Box**: 구성 가능한 질량 및 마찰을 가진 동적 밀 수 있는 박스
     - **Sphere**: 동적 구 객체
     - **Gate**: 두 개의 벽과 간격이 있는 정적 게이트 (항해 작업용)

2. **로봇 설정**
   - 시작 위치에 Spot 로봇 생성
   - 목표 방향을 향하도록 로봇 자동 방향 설정
   - 이동을 위해 Isaac Sim의 `SpotFlatTerrainPolicy` 사용

3. **카메라 시스템**
   - **에고 카메라**: 로봇 본체에 장착 (1280×800, 16:10 종횡비)
   - **탑 카메라**: 오버헤드 뷰 (1600×1600, 정사각형 종횡비)
   - 두 카메라 모두 실험 중 10Hz로 이미지 캡처
   - 타임스탬프가 포함된 파일명으로 JPEG 파일로 저장

4. **실험 데이터 수집**
   - **자동 시작**: 첫 번째 키보드 명령 후 데이터 저장 시작
   - **CSV 로깅**: 10Hz에서 로봇 및 객체 위치/방향
   - **이미지 캡처**: 동기화된 에고 및 탑 카메라 이미지
   - **설정 저장**: 실제 사용된 값 (범위 아님)
   - **터미널 로깅**: 모든 콘솔 출력이 `terminal.log`에 저장

5. **작업 메트릭**
   - **L1 거리**: 작업 완료를 위한 맨해튼 거리 메트릭
   - **객체별**:
     - 게이트 작업: 로봇 ↔ 목표 거리
     - 박스/구 작업: 객체 ↔ 목표 거리
   - **완료 감지**: 거리 < 1.0m일 때 목표가 녹색으로 변경

#### 설정

시뮬레이션은 다음을 통해 구성할 수 있습니다:
1. **기본값** (`DEFAULT_CONFIG`에 하드코딩됨)
2. **JSON 파일** (`config_file` 매개변수)
3. **키워드 인수** (`SpotSimulation()`에 전달됨)

예제:
```python
sim = SpotSimulation(
    experiment_name="my_test",
    randomize=True,
    map_size=12.0,
    object_type="gate",  # 또는 "box", "sphere"
    use_object=True,
    max_vx=2.5
)
```

### `plot_experiment.py` - 시각화 도구

실험 결과를 분석하기 위한 대화형 시각화 도구입니다.

#### 기능

1. **궤적 맵**
   - 시간 기반 색상 그라데이션을 가진 로봇 및 객체 궤적
   - 일정 간격으로 로봇 방향을 보여주는 방향 화살표
   - 시작/목표 마커 및 경계 벽
   - 현재 위치 마커 (슬라이더로 업데이트)

2. **속도 플롯**
   - 시간에 따른 XY 속도 구성 요소 (vx, vy)
   - 시간 그라데이션 색상
   - 현재 시간 위치를 보여주는 수직선

3. **L1 거리 메트릭**
   - 시간에 따른 목표까지의 거리
   - 성공 임계값 선 (1.0m)
   - 현재 거리 표시

4. **이미지 뷰어** (카메라 이미지가 있는 경우)
   - **탑 카메라 뷰**: 오버헤드 관점
   - **에고 카메라 뷰**: 로봇의 관점
   - 타임스탬프 슬라이더로 이미지 자동 업데이트

5. **대화형 슬라이더**
   - 실험 타임라인 탐색
   - 모든 플롯 및 이미지를 동기적으로 업데이트
   - 현재 타임스탬프 표시

#### 사용법

```bash
# 실험 디렉토리 지정
python plot_experiment.py expr_data/251118_185550-my_experiment

# 최신 실험 사용 (Enter 누르기)
python plot_experiment.py
```

도구는 자동으로:
- 디렉토리가 지정되지 않은 경우 최신 실험 찾기
- 설정 및 CSV 데이터 로드
- 사용 가능한 카메라 이미지 감지
- 종합 시각화 생성
- 실험 디렉토리에 `experiment_plot.png`로 플롯 저장

## 실험 데이터 구조

각 실험은 타임스탬프와 이름이 있는 디렉토리를 생성합니다:
```
expr_data/
└── YYMMDD_HHMMSS-Experiment_Name/
    ├── config.json          # 실험에 사용된 설정
    ├── data.csv             # 시계열 데이터 (10Hz)
    ├── terminal.log         # 콘솔 출력 로그
    ├── experiment_plot.png  # 생성된 시각화 (플롯 도구에서)
    └── camera/
        ├── ego/            # 에고뷰 카메라 이미지
        │   ├── frame0-0.000-ego.jpg
        │   ├── frame1-0.100-ego.jpg
        │   └── ...
        └── top/             # 탑다운 카메라 이미지
            ├── frame0-0.000-top.jpg
            ├── frame1-0.100-top.jpg
            └── ...
```

### CSV 데이터 형식

`data.csv` 파일에는 다음이 포함됩니다:

| 열 | 설명 |
|--------|-------------|
| `timestamp` | 첫 번째 명령 이후 경과 시간 (초) |
| `frame_num` | 프레임 번호 (0부터 시작) |
| `robot_pos_x/y/z` | 로봇 위치 (미터) |
| `robot_orient_w/x/y/z` | 로봇 방향 (쿼터니언) |
| `object_pos_x/y/z` | 객체 위치 (미터) |
| `object_orient_w/x/y/z` | 객체 방향 (쿼터니언) |
| `l1_distance_to_goal` | 목표까지의 L1 거리 (미터) |

### 이미지 명명 규칙

이미지는 다음과 같이 명명됩니다: `frame{N}-{timestamp}-{camera_type}.jpg`

- `N`: 프레임 번호 (0부터 시작)
- `timestamp`: 경과 시간 (초, 소수점 3자리)
- `camera_type`: `ego` 또는 `top`

예제: `frame42-4.200-ego.jpg` (4.2초에 43번째 프레임, 에고뷰)

## 설정 매개변수

### 로봇 제어
- `max_vx`, `max_vy`, `max_yaw`: 최대 속도 (m/s, rad/s)
- `acc_vx`, `acc_vy`, `acc_yaw`: 가속률
- `decay_vx`, `decay_vy`, `decay_yaw`: 감쇠 계수
- `update_dt`: 컨트롤러 업데이트 주기 (기본값: 0.02s = 50Hz)

### 환경
- `map_size`: 정사각형 환경 크기 (미터)
- `wall_height`: 경계 벽 높이 (미터)
- `start_position`, `goal_position`: 로봇 시작/목표 [x, y] 위치
- `randomize`: 환경 랜덤화 활성화
- `wall_inset`: 유효한 스폰 영역을 위한 벽으로부터의 여백

### 객체
- `use_object`: 객체 스폰 활성화/비활성화
- `object_type`: `"box"`, `"sphere"`, 또는 `"gate"`
- `box_position`, `box_scale`, `box_mass`: 박스 속성
- `box_friction_static`, `box_friction_dynamic`: 박스 마찰
- `box_scale_range`, `box_mass_range`: 랜덤화 범위

### 게이트 전용 (`object_type="gate"`일 때)
- `wall_depth_min/max`: 게이트 벽 깊이 범위 (미터)
- `gate_y_offset_min/max`: Y축 오프셋 범위 (미터)
- `gap_min/max`: 게이트 벽 사이의 간격 (미터)
- `gate_location_min/max`: 시작-목표 선을 따른 위치 (0.0-1.0)

### 카메라
- `ego_camera_resolution`: 에고 카메라용 [width, height] (기본값: [1280, 800])
- `top_camera_resolution`: 탑 카메라용 [width, height] (기본값: [1600, 1600])

## 시뮬레이션 아키텍처

### 물리 루프

시뮬레이션은 다음 속도로 실행됩니다:
- **물리 타임스텝**: 500Hz (2ms)
- **렌더링 타임스텝**: 50Hz (20ms)
- **명령 업데이트**: 50Hz (10 물리 스텝마다)
- **데이터 로깅**: 10Hz (50 물리 스텝마다)

### 데이터 저장 로직

1. **실험 디렉토리**는 `setup()` 중에 생성됨
2. **데이터 저장 시작**: 첫 번째 키보드 명령 수신 후
3. **실험 시작 시간**: 첫 번째 데이터 포인트 저장 시 기록됨
4. **데이터 수집 계속**: 작업이 진행 중인 동안 (L1 거리 > 1.0m)
5. **작업 완료**: 목표 색상을 녹색으로 변경
6. **정리**: 파일 닫기 및 카메라 어노테이터 분리

## 프로젝트 구조

```
.
├── README.md                    # 이 파일
├── README_KR.md                 # 한국어 버전
├── quadruped_example.py         # 메인 시뮬레이션 (SpotSimulation 클래스)
├── plot_experiment.py           # 시각화 도구
├── keyboard_controller.py       # Pygame 키보드 컨트롤러
├── example_config.json          # 예제 설정 파일
├── custom_robots/
│   └── spot.py                  # Spot 로봇 구현
├── expr_data/                   # 실험 데이터 디렉토리
│   └── YYMMDD_HHMMSS-Name/     # 개별 실험 폴더
└── docs/                        # 문서 파일
    ├── EXPERIMENT_DATA_GUIDE.md
    ├── D455_CAMERA_GUIDE.md
    └── ...
```

## 예제

### 기본 사용법

```python
from quadruped_example import SpotSimulation

# 시뮬레이션 생성
sim = SpotSimulation(experiment_name="test_run")
sim.setup()
sim.run()
sim.cleanup()
```

### 사용자 정의 설정

```python
# JSON에서 로드
sim = SpotSimulation(
    config_file="example_config.json",
    experiment_name="custom_test"
)

# 또는 특정 값 덮어쓰기
sim = SpotSimulation(
    experiment_name="gate_navigation",
    randomize=True,
    map_size=15.0,
    object_type="gate",
    max_vx=2.5
)
```

### 박스 밀기 작업

```python
sim = SpotSimulation(
    experiment_name="box_push",
    object_type="box",
    box_mass=10.0,
    box_friction_static=0.8,
    randomize=True
)
```

## 문제 해결

### 일반적인 문제

1. **Pygame 창이 나타나지 않음**
   - 디스플레이가 사용 가능한지 확인 (헤드리스 모드 아님)
   - `SimulationApp` 초기화에서 `headless=False` 확인

2. **로봇이 응답하지 않음**
   - 키보드 컨트롤러 스레드가 시작되었는지 확인
   - pygame 창에 포커스가 있는지 확인
   - 물리 콜백에서 컨트롤러가 업데이트되는지 확인

3. **실험 데이터가 저장되지 않음**
   - 데이터 저장은 첫 번째 키보드 명령 후에만 시작됨
   - `expr_data/` 디렉토리가 존재하는지 확인
   - 실험 디렉토리가 생성되었는지 확인

4. **카메라 이미지가 캡처되지 않음**
   - `omni.replicator`가 사용 가능한지 확인
   - 카메라 렌더 제품이 초기화되었는지 확인
   - 카메라 경로가 유효한지 확인

5. **플롯 도구 오류**
   - 실험 디렉토리에 `config.json` 및 `data.csv`가 있는지 확인
   - 카메라 이미지 디렉토리가 존재하는지 확인 (비어 있어도 됨)
   - matplotlib 백엔드가 대화형 디스플레이를 지원하는지 확인

## 성능

- **물리**: 500Hz (2ms 타임스텝)
- **렌더링**: 50Hz (20ms 타임스텝)
- **데이터 로깅**: 10Hz (100ms 간격)
- **이미지 캡처**: 10Hz (로깅과 동기화)

## 문서

`docs/`에서 추가 문서를 사용할 수 있습니다:
- `EXPERIMENT_DATA_GUIDE.md`: 완전한 데이터 저장 가이드
- `D455_CAMERA_GUIDE.md`: 카메라 시스템 문서
- `ENVIRONMENT_PARAMS.md`: 완전한 매개변수 참조

## 라이선스

Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION과 그 라이선스 제공자는 이 소프트웨어, 관련 문서 및 모든 수정 사항에 대한 지적 재산권 및 독점권을 보유합니다. NVIDIA CORPORATION의 명시적 라이선스 계약 없이 이 소프트웨어 및 관련 문서의 사용, 복제, 공개 또는 배포는 엄격히 금지됩니다.

## 참고 자료

- [Isaac Sim 문서](https://docs.isaacsim.omniverse.nvidia.com/)
- [Isaac Sim Core API 튜토리얼](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/core_api_tutorials/tutorial_core_hello_world.html)
- [Boston Dynamics Spot](https://www.bostondynamics.com/products/spot)

