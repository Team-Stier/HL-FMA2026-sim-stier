# Control

`/ego_status`, `/local_path`, `/speed_limit`만 구독하는 IONIQ 6 로컬 패스
추종 제어기다. 횡방향은 선형 운동학 bicycle MPC, 종방향은 acceleration PID다.
팀의 기존 `interfaces/ControlCommand`를 사용하며 다른 패키지 소스를 import하지 않는다.

## 현재 안전 상태

`config/control.yaml`의 `enable_output=false`, `calibration_verified=false`가
기본값이다. 이 상태에서는 `/control/candidate`만 발행하고 `/ctrl_cmd`는 발행하지
않는다. `config/vehicle.yaml`도 calibration 미확정이고 Sim Bridge의
`allow_motion=false`이므로 실제 주행은 이중 차단된다.

`wheelbase_m=2.944`, `maximum_steering_rad=0.48`은
`config/vehicle.yaml`의 `HyundaiIoniq6_23.xml` 모델 설정 출처다. 조향 변화율,
PID gain, 가감속과 path-end 감속은 모의 검증 초기값이며 실측 보정값이 아니다.

## 계약

- `/ego_status`: `interfaces/EgoStatus`, `map`, 후륜축, m/rad/m/s, source stamp
- `/local_path`: `nav_msgs/Path`, `base_link`, path 생성시각 `t0`의 좌표
- `/speed_limit`: `std_msgs/Float32`, m/s, Header가 없어 local receipt timeout 사용
- `/control/candidate`, `/ctrl_cmd`: `interfaces/ControlCommand`, `base_link`,
  road-wheel steering rad와 target acceleration m/s²
- `/control/status`: `std_msgs/String` 진단

차량좌표 path는 `t0`의 Ego pose로 map에 복원한 뒤 현재 Ego pose로 변환한다.
NaN/Inf, 중복점, 반대방향 segment, 빈/짧은/stale path를 거부한다. 최근접 선택은
진행방향과 이전 map 투영점 연속성을 함께 사용한다. 로컬 path 끝은 종료가 아니며,
남은 path가 horizon보다 짧으면 외삽하지 않고 제동 가능 속도로 제한한다.

timestamp 역행 또는 위치 점프에서는 path, warm start, PID, 이전 입력과 최근접
hint를 초기화하고 새 path를 기다린다. stale/solver 실패 시 candidate는 조향을
rate-limit으로 0에 접근시키며 설정된 음의 emergency acceleration을 요청한다.
통신 단절 때 이 요청이 VTD에 전달됐다고 간주하지 않는다.

## 빌드와 테스트

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select interfaces control
colcon test --packages-select control
colcon test-result --verbose
source install/setup.bash
ros2 launch control control.launch.py
```

현재 작성 환경에는 ROS 2 Jazzy가 없어 ROS node/ament 빌드는 미실행이다. 독립
C++17 core는 다음과 같이 빌드해 직선·곡선·중복/빈/짧은 path·capture pose 보상·
yaw 경계·path-end·solver fail·조향 rate·속도 단위를 모의 검증했다.

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude \
    src/controller_core.cpp test/control_core_test.cpp -o /tmp/control_core_test
/tmp/control_core_test
```

실제 출력 활성화 전에는 VTD에서 조향 부호/응답, actuator delay, 명령 가속도와
달성 가감속, 속도별 제동거리, stale/reset을 측정하고 두 confirmation flag를
true로 바꾼다. 방향지시등 의도 입력은 현재 계약에 없으므로 `turn_signal=0`이다.
