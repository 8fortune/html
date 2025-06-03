# 하이브리드 차량 동적 성능 시뮬레이터

## 프로젝트 설명

본 프로젝트는 하이브리드 차량의 상세 제원을 기반으로 동적 성능을 예측하는 Python 프로그램입니다.
차량의 가속 시간 (0-100 km/h, 80-120 km/h 등), 최고 속도 (하이브리드 모드, 엔진 단독 모드),
그리고 주요 성능 관련 그래프 (속도-시간, 토크-RPM, 파워-RPM, 기어별 가속도 맵)를 시뮬레이션을 통해 계산하고 시각화합니다.

## 프로젝트 구조

-   `hybrid_vehicle_simulation/`: 프로젝트 루트 디렉토리
    -   `main.py`: 시뮬레이션 실행을 위한 메인 스크립트.
    -   `src/`: 핵심 소스 코드가 위치한 디렉토리.
        -   `core.py`: 차량, 엔진, 모터, 변속기, 배터리 등 핵심 구성 요소 클래스 정의.
        -   `simulation.py`: 가속 및 최고 속도 시뮬레이션 로직 구현.
        -   `outputs.py`: 시뮬레이션 결과 포맷팅 및 데이터 생성 함수.
        -   `plotting.py`: Matplotlib를 사용한 그래프 생성 함수.
    -   `data/`: 시뮬레이션에 사용될 데이터 파일 (예: 토크 커브 CSV).
        -   `sample_engine_torque.csv`: 샘플 엔진 토크 커브 데이터.
        -   `sample_motor_torque.csv`: 샘플 모터 토크 커브 데이터.
    -   `tests/`: 단위 테스트 코드가 위치한 디렉토리.
        -   `test_core.py`: `core.py`의 클래스들에 대한 단위 테스트.
        -   `test_simulation.py`: `simulation.py`의 함수들에 대한 단위 테스트.
    -   `README.md`: 본 프로젝트 설명 파일.

## 요구 사항

-   Python 3.7 이상
-   필수 라이브러리:
    -   `pandas`
    -   `matplotlib`
    -   `numpy` (matplotlib 또는 pandas의 의존성으로 설치될 수 있음)

## 설치 방법

1.  **프로젝트 클론 (또는 다운로드)**:
    ```bash
    # git clone <repository_url> # (레포지토리가 있다면)
    # cd hybrid_vehicle_simulation
    ```

2.  **필수 라이브러리 설치**:
    터미널 또는 명령 프롬프트에서 다음 명령을 실행하여 필요한 라이브러리를 설치합니다.
    (가상 환경 사용을 권장합니다.)
    ```bash
    pip install pandas matplotlib numpy
    ```
    또는, 프로젝트에 `requirements.txt` 파일이 있다면 (현재는 없음):
    ```bash
    # pip install -r requirements.txt
    ```

## 실행 방법

### 1. 메인 시뮬레이션 실행

프로젝트 루트 디렉토리 (`hybrid_vehicle_simulation/`)에서 다음 명령을 실행합니다:
```bash
python main.py
```
또는, `hybrid_vehicle_simulation` 디렉토리 외부에서 실행하는 경우:
```bash
python path/to/hybrid_vehicle_simulation/main.py
```
시뮬레이션 결과 (가속 시간, 최고 속도 등)가 터미널에 한국어로 출력되고, 성능 그래프가 화면에 표시됩니다.

### 2. 단위 테스트 실행

프로젝트 루트 디렉토리 (`hybrid_vehicle_simulation/`)에서 다음 명령을 실행하여 모든 테스트를 실행합니다:
```bash
python -m unittest discover tests
```
특정 테스트 파일만 실행하려면 (예: `test_core.py`):
```bash
python -m unittest tests.test_core
```

## 주요 기능 및 출력

-   **가속 시간**:
    -   0-60 km/h
    -   0-100 km/h
    -   0-120 km/h
    -   80-120 km/h
-   **최고 속도**:
    -   하이브리드 모드
    -   엔진 단독 모드
-   **시각화 그래프**:
    -   속도 vs 시간
    -   토크 vs RPM (엔진, 모터, 결합 토크 및 기어 변속 지점)
    -   파워 vs RPM (엔진, 모터, 결합 파워 및 기어 변속 지점)
    -   기어별 최대 가속도 맵
-   **출력 언어**: `main.py` 및 `outputs.py`의 터미널 출력은 한국어로 제공됩니다.
