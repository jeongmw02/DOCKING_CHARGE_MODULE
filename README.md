# 🛰️ CubeSat Docking & Charging Module

> 연세대학교 기계공학부 창의제품설계 (2025)  
> 12U CubeSat 프레임 탑재용 **4U 자율 도킹·충전 모듈** 프로토타입

---

## 📌 프로젝트 소개

소형 위성(CubeSat)이 궤도 상에서 다른 위성 또는 도킹 스테이션에 자율적으로 접근·결합하고 전력을 전달받는 시스템을 지상 환경에서 시연한다.

**도킹 시퀀스**

```
IDLE  →  APPROACH  →  SOFT CAPTURE  →  HARD LOCK  →  SOLAR DEPLOY  →  CHARGING
                                                             ↘
                                                            ERROR
```

- **IDLE**: 대기, ToF 센서로 전방 감시
- **APPROACH**: 목표물 감지 후 스테퍼 모터로 선형 접근
- **SOFT CAPTURE**: 전자석 ON + Maxwell Kinematic Coupling으로 정밀 정렬
- **HARD LOCK**: 기계식 Pin/Cam Lock으로 구조 고정
- **SOLAR DEPLOY**: 서보 모터로 솔라 패널 전개
- **CHARGING**: Pogo Pin 접촉 확인 후 전력 전달 시작

---

## 🔧 하드웨어 구성 및 배선

### 구성 요소

| 컴포넌트 | 모델 | 역할 |
|---|---|---|
| 중앙 제어 | Raspberry Pi 4 | 전체 시퀀스 제어 |
| ToF 센서 × 3 | VL53L1X | 전방·좌·우 거리 측정 (I2C) |
| 전자석 | 12V Electromagnet | Soft Capture 흡착 |
| 정렬 구조 | Maxwell Kinematic Coupling | V-groove 3점 접촉 정밀 정렬 |
| Hard Lock | 서보/솔레노이드 (TBD) | 기계식 고정 |
| 서보 모터 × 2 | MG996R | 솔라 패널 전개/수납 |
| Pogo Pin | Spring-loaded contact | 도킹 후 전력 전달 |
| 시연 리그 | NEMA17 + 리드스크류 | 4U 모듈 선형 접근 구동 |

---

### GPIO 핀 배정 (BCM 기준)

| 핀 번호 | 신호 | 연결 대상 |
|---|---|---|
| GPIO 17 | XSHUT_0 | ToF 센서 #0 (전방) |
| GPIO 27 | XSHUT_1 | ToF 센서 #1 (좌) |
| GPIO 22 | XSHUT_2 | ToF 센서 #2 (우) |
| GPIO 18 | PWM | 전자석 MOSFET Gate |
| GPIO 23 | STEP | 스테퍼 드라이버 (DRV8825) |
| GPIO 24 | DIR | 스테퍼 드라이버 방향 |
| GPIO 25 | EN | 스테퍼 드라이버 활성화 |
| GPIO 12 | PWM (HW) | 서보 A (패널 좌) |
| GPIO 13 | PWM (HW) | 서보 B (패널 우) |
| GPIO 16 | IN (PUD_UP) | Pogo Pin 접촉 감지 |
| GPIO 20 | OUT | 충전 회로 활성화 |

> I2C: SDA → GPIO2, SCL → GPIO3 (Raspberry Pi 기본)

---

### 배선 주의사항

- 전자석 코일 양단에 **프리휠링 다이오드(flyback diode)** 필수
- 전자석·스테퍼는 별도 전원(12V)으로 구성, GPIO는 MOSFET으로 절연
- ToF 센서 3개는 I2C 버스 공유 → XSHUT 핀으로 순차 초기화 후 주소 재할당 (`0x30`, `0x31`, `0x32`)
- 서보는 GPIO 12/13 (Hardware PWM) 사용 권장

---

## 📁 폴더 구조 및 파일 역할

```
cubeSat_docking/
│
├── main.py              # 진입점. DockingStateMachine 생성 → setup → run → cleanup
├── config.py            # 모든 GPIO 핀 번호, 임계값, 속도, 타임아웃을 한 곳에서 관리
├── state_machine.py     # 핵심 로직. 7개 상태 전이 + 각 상태 핸들러 + 오류 처리
├── requirements.txt     # pip 의존 라이브러리 목록
│
├── hardware/            # 하드웨어별 드라이버 (1 파일 = 1 장치)
│   ├── __init__.py      # 외부에서 `from hardware import ...` 가능하게 묶음
│   ├── tof_sensor.py    # VL53L1X I2C 통신, 복수 센서 주소 할당, 정렬 오차 계산
│   ├── electromagnet.py # PWM Pull→Hold 자동 전환, MOSFET 구동
│   ├── stepper.py       # STEP/DIR 펄스 생성, mm 단위 이동, 비동기 실행 지원
│   ├── servo.py         # 각도→듀티 변환, 부드러운 이동, SolarPanelArray 헬퍼
│   └── pogo_pin.py      # GPIO 인터럽트 기반 접촉 감지, 디바운싱, 충전 회로 제어
│
├── utils/
│   ├── __init__.py
│   └── logger.py        # 콘솔·파일 동시 출력 공통 로거 (모든 모듈 공유)
│
└── tests/               # 하드웨어별 단독 동작 확인 스크립트
    ├── test_tof.py              # ToF 실시간 거리 출력 + 정렬 오차 확인
    ├── test_electromagnet.py    # ON 3초 → OFF 순서 확인
    ├── test_stepper.py          # 전진 50mm → 후진 50mm 왕복
    ├── test_servo.py            # 패널 전개 → 수납 확인
    ├── test_pogo.py             # 접촉 감지 + 충전 활성화 확인
    └── test_state_machine.py    # 상태 전이 수동 시뮬레이션
```

---

## ⚙️ 설치 및 실행

```bash
# 의존 라이브러리 설치
pip install -r requirements.txt

# 전체 시퀀스 실행
python main.py

# 하드웨어 단독 테스트
python tests/test_tof.py
python tests/test_stepper.py
python tests/test_electromagnet.py
python tests/test_servo.py
python tests/test_pogo.py
```

---

## 📦 의존 라이브러리

| 라이브러리 | 용도 |
|---|---|
| `RPi.GPIO` | GPIO 핀 제어 |
| `VL53L1X` | ToF 센서 I2C 드라이버 |
| `smbus2` | I2C 저수준 통신 |

---

## 👥 팀 정보

| | |
|---|---|
| 소속 | 연세대학교 기계공학부 4학년 |
| 과목 | 창의제품설계 |
| 인원 | 5명 |
| 예산 | 100만 원 이내 |
| 시연 | 2025년 6월 첫째 주 |

---

## 🗺️ 개발 로드맵

- [x] 프로젝트 구조 설계 및 GitHub 세팅
- [ ] 하드웨어 배선 확정 및 회로도 작성
- [ ] 하드웨어별 드라이버 구현
- [ ] 상태 머신 로직 구현
- [ ] 단위 테스트 통과
- [ ] Hard Lock 메커니즘 방식 확정 (서보 vs 솔레노이드)
- [ ] 통합 시연 및 발표
