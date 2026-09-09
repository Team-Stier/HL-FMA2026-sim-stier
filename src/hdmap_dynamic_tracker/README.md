# HDMap Dynamic Tracker

`/ego_pose`, `/objects`, `/traffic_light`를 받아 정적 HDMap Cell에 동적 상태를 겹쳐
`/ego_status`, `/dynamic_status`를 발행하는 ROS 2 C++ 노드다. Cell ID는 0부터 시작하며
출력 배열의 인덱스로 직접 사용한다. 개수는 지도에서 읽고 하드코딩하지 않는다.

## 현재 구현

- `/ego_status`: pose 입력마다 즉시 발행한다. 속도는 연속 source stamp 사이의 XY 이동량으로
  계산한다. 첫 샘플, 비양수/0.25초 초과 dt, 300km/h 초과 점프는 0이고 추적 이력을 reset한다.
- `/dynamic_status`: 가장 최근에 도착한 Ego와 Objects를 결합한다. 두 입력의 stamp가 같을
  필요는 없으며 `/traffic_light`가 아직 오지 않아도 20Hz worker가 처리한다. 계산을 시작한
  결과는 중간에 새 입력이 와도 발행한다.
- 정적 speed cap: 부모 Lanelet의 명시적 `speed_limit`을 Cell별로 캐시한다. 속성이 없거나
  파싱할 수 없으면 안전하게 0m/s다.
- 신호 speed cap: 현재 통합 경로에서는 적용하지 않는다. `/traffic_light`는 구독하지만
  `/dynamic_status.speed_cap_mps`에는 정적 지도 cap만 넣는다. 신호 상태 계약이 확정되면
  정지선 감속장을 다시 연결한다.
- 객체 점유: 현재 OBB와 `[min_z, min_z + height]`를 CellTree의 실제 polygon 교차로 계산한다.
  ID별 6상태 CTRA EKF(`x/y`, 속력, 진행방향, 회전율, 가속도)로 6초를 예측한다. 비선형 상태와
  공분산은 최대 0.05초 간격으로 전파하고, 각 0.5초 점유 구간의 실제 EKF 곡선을 기본 0.1초
  간격으로 다시 예측한 OBB들의 convex hull로 조회한다. 별도의 sigma·방향 미확정·yaw 외접·
  chord 오차 팽창은 적용하지 않는다. 첫 관측은 API speed와 heading으로 시작하고, 이후 위치차가
  있으면 그 진행방향을 즉시 EKF에 반영한다. 현재 관측 bin 0만 raw OBB와 실제 Z 범위를 쓰고,
  미래 bin은 2D Cell 교차를 사용한다.
- 관측 소실 객체는 기본 0.5초 동안 유지한다. 객체 API는 80m/최대 30개이므로 기본 설정은
  객체가 없다는 이유만으로 Cell을 free=0으로 만들지 않고 unknown=-1을 유지한다.
  free 반경을 켜려면 80m 이하 값과 `free_space_assumption_verified=true`를 함께 명시해야 한다.

따라서 현재 Planner는 `-1`을 단순 free로 간주하면 안 되고, blocked 또는 별도 unknown 비용으로
처리할 정책이 필요하다. 반대로 모두 blocked로만 처리하면 기본 설정에서 경로가 나오지 않을 수
있다. 주최측 API가 보장하는 free-space 범위·30개 초과 표시가 확인되기 전에는 Tracker가 이를
임의로 결정하지 않는다.

모든 주행 토픽 QoS는 `BEST_EFFORT / VOLATILE / KEEP_LAST(1)`이며 출력 Header는 입력 Ego
Header를 계승한다. `occupancy[cell_id * 13 + 0]`은 현재, bin 1..12는 각각
`((bin-1)*0.5, bin*0.5]`초다.

## 정지선 선형 speed cap (현재 비활성)

아래 계산 코드는 남아 있지만 현재 `/dynamic_status` 생성 경로에서는 호출하지 않는다. 신호등
ID·상태와 정지선 적용 규칙이 확정되기 전까지 speed cap은 지도에 저장된 정적 값만 발행한다.

재연결할 때 사용할 계획식은 아래와 같다. 현재 실행에서는 이 값들을 읽더라도 cap에 반영하지 않는다.

```text
B_cal(v)   = VTD HyundaiIoniq6_23_Dyn의 감속 시작→완전 정지 거리
d_linear   = max(v² / a_design, 2 × B_cal(v))
d_profile  = braking_distance_factor × d_linear + v × latency_budget
d_start    = stop_margin + d_profile
cap(d)     = v × clamp((d - stop_margin) / d_profile, 0, 1)
```

신호 상태 계약을 정한 뒤 정지선 predecessor 연결, 아이오닉 6 제동거리 보정값, 황색 처리와
Controller HOLD 정책을 함께 검증하고 이 경로를 활성화해야 한다.

## 실행

루트 `run.sh`가 HDMap 코어를 먼저 설치한 뒤 이 패키지를 colcon으로 빌드하고 통합 launch에
포함한다. 단독 실행은 다음과 같다.

```bash
export HDMAP_PATH=/absolute/path/to/hdmap.bin
ros2 launch hdmap_dynamic_tracker tracker.launch.py
```

## 아직 대회 주행에 사용하면 안 되는 이유

이 구현은 통신·배열·기하·예측 파이프라인을 검증하기 위한 1차 버전이다.

- `config/runtime.yaml`의 실제 설계 감속도와 `config/vehicle.yaml`의 제동표가 아직
  `null`/빈 배열이다. tracker ROS 설정의 2.0m/s²는 시뮬레이션 개발용이며
  `signals.calibration_verified=false`다.
- 신호 감속장은 현재 출력에 연결하지 않았다. 황색 통과 latch, 현재 접근로/movement 선택,
  정지선 통과와 clearing 판정은 경로 입력 또는 접근 Cell 판정 계약이 확정된 뒤 추가해야 한다.
- 향후 신호 감속장을 연결할 때 `Cell::previous()`의 단일 predecessor 한계, stopline 미해결
  controller, sibling controller와 `permitted_states` 충돌을 먼저 해결해야 한다.
- DynamicStatus의 원시 배열은 현재 지도에서 프레임당 약 5.03MiB, 20Hz에서 구독자당 약
  100.6MiB/s다. 대상 Ubuntu/VTD 장비에서 p95 계산시간과 DDS 지연/drop을 계측하고,
  명시적 상태 필드·소비자 watchdog·취소 가능한 Cell 조회를 포함한 fail-closed deadline 경로를
  추가해야 한다. 오래된 정상 배열의 stamp만 바꿔 발행해서는 안 된다.

위 항목이 해결되고 `config/runtime.yaml: allow_motion`을 별도로 승인하기 전에는 차량 구동을
허용하지 않는다.

## 테스트

ROS/Lanelet과 무관한 코어는 표준 C++17만으로 검사할 수 있다.
테스트에는 기본 추적·유지 시간과 CTRA의 선회, 가속, 정지 후 역주행 방지 및 곡선 다중 표본
swept footprint 회귀 검사가 포함된다.

```bash
clang++ -std=c++17 -Wall -Wextra -Wpedantic -Werror \
  -I src/hdmap_dynamic_tracker/include \
  src/hdmap_dynamic_tracker/src/tracker_core.cpp \
  src/hdmap_dynamic_tracker/src/ekf_predictor.cpp \
  src/hdmap_dynamic_tracker/test/tracker_core_test.cpp \
  -o /tmp/tracker_core_test && /tmp/tracker_core_test
```

ROS 2 Jazzy 환경에서는 루트 `run.sh` 빌드 또는 `colcon test --packages-select
hdmap_dynamic_tracker`로 실행한다.
