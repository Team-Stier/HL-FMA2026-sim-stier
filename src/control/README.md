# Control

`/ego_status`, `/local_path`, `/speed_limit`만 구독하는 IONIQ 6 로컬 패스
추종 제어기다. 횡방향은 선형 운동학 bicycle MPC, 종방향은 acceleration PID다.
팀의 기존 `interfaces/ControlCommand`를 사용하며 다른 패키지 소스를 import하지 않는다.

## 현재 안전 상태

node 자체의 `enable_output`과 `calibration_verified` 기본값은 false다.
저장소의 시뮬레이션용 `config/control.yaml`은 두 값을 true로 지정하므로
launch 시 `/ctrl_cmd`도 발행한다. false 상태의 독립 노드 검사는
`/control/candidate`만 사용한다.

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
NaN/Inf 점이 포함된 path는 전체를 거부하며 중복점은 제거한다. 진행방향,
최대 투영 거리, capture 높이, 이전 map 투영점 연속성을 확인한다. 빈 Path는
즉시 이전 경로 실행 권한을 해제한다. 유효하지만 최소 MPC horizon보다 짧은
경로는 STOP_SHORT_PATH로 감속하고, 원점의 한 pose는 정지 요청/HOLD로 처리한다.
한 점 경로는 정지 완료를 뜻하지 않으며 이동 중이면 먼저 제동한다.
새 다점 경로가 오면 정상 제어를 재개한다.

경로 끝은 전체 목적지 도착을 뜻하지 않는다. 경로 끝의 제동거리 속도 상한은
preview 길이와 관계없이 적용한다. 곡률 계산의 공간 간격은 MPC 시간 단계와
분리하며, 각 단계의 곡률은 현재 속도 × dt에 해당하는 진행거리에서 얻는다.

중복/과거 stamp는 수신 시간을 갱신하지 않는다. 위치·높이·큰 heading 점프나
입력 공백에서는 path, capture pose 이력, warm start, PID, 최근접 hint를 초기화하고
reset 이후의 새 path를 기다린다. 실제 직전 발행 조향은 reset에서 지우지 않는다. stale/solver 실패 시 candidate는 조향을
rate-limit으로 0에 접근시키며 설정된 음의 emergency acceleration을 요청한다.
모든 발행 분기에 같은 조향 변화율·크기·유한값 검사를 적용한다. fault는 가속하지
않으며 zero cap/HOLD는 정지 제동을 유지한다. 통신 단절 때 이 요청이 VTD에
전달됐다고 간주하지 않는다. 실제 발행 가속도는 PID의 변화율 상태에 반영하므로
긴급제동/과속 override 해제 후에도 직전 제동값에서 가속을 완만하게 증가시킨다.
PID reset은 적분·속도 이력을 비우고 실제 마지막 가속도는 보존한다.

`/speed_limit`는 계속 Float32다. speed_annotator는 동일 source stamp의 EgoPose와
DynamicStatus를 맞추고 새 dynamic snapshot마다 한 번만 발행한다. 새 Ego 때문에
오래된 dynamic cap을 재발행하지 않는다. 새 trajectory/speed/ETA 메시지는 없다.

## 빌드와 테스트

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select interfaces control
colcon test --packages-select control
colcon test-result --verbose
source install/setup.bash
ros2 launch control control.launch.py
```

ROS 노드 검사는 별도 ROS domain에서 출력 비활성화 상태로 실행한다. 실제 노드의
fallback, empty/HOLD와 재출발, stale 입력, yaw/높이/위치 reset, 조향 제한을 검사한다.
Release 빌드에서도 core 회귀 assert를 활성화한다. 독립 C++ core 확인도 가능하다.

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude \
    src/controller_core.cpp test/control_core_test.cpp -o /tmp/control_core_test
/tmp/control_core_test
```

실제 출력 활성화 전에는 VTD에서 조향 부호/응답, actuator delay, 명령 가속도와
달성 가감속, 속도별 제동거리, stale/reset을 측정하고 두 confirmation flag를
true로 바꾼다. 방향지시등 의도 입력은 현재 계약에 없으므로 `turn_signal=0`이다.
