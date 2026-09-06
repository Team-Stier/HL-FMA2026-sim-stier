# HDMap Dynamic Tracker

`/ego_pose`, `/objects`, `/traffic_light`를 받아 정적 HDMap Cell에 동적 상태를 겹쳐
`/ego_status`, `/dynamic_status`를 발행하는 ROS 2 C++ 노드다. Cell ID는 0부터 시작하며
출력 배열의 인덱스로 직접 사용한다. 개수는 지도에서 읽고 하드코딩하지 않는다.

## 현재 구현

- `/ego_status`: pose 입력마다 즉시 발행한다. 속도는 연속 source stamp 사이의 XY 이동량으로
  계산한다. 첫 샘플, 비양수/0.25초 초과 dt, 300km/h 초과 점프는 0이고 추적 이력을 reset한다.
- `/dynamic_status`: 같은 exact stamp의 세 입력만 결합한다. 최대 4개 stamp만 잠시 보관하고
  최신 완성 snapshot을 20Hz worker가 처리한다. 계산 중 더 최신 완성 snapshot이 생기면 이전
  결과는 발행하지 않는다.
- 정적 speed cap: 부모 Lanelet의 명시적 `speed_limit`을 Cell별로 캐시한다. 속성이 없거나
  파싱할 수 없으면 안전하게 0m/s다.
- 신호 speed cap: `signalRegistry()`의 controller ID와 `permitted_states`, `stoplineCells()`를
  사용한다. 정지선 Cell에서 `previous()`를 거슬러 `centerline_length_m`을 누적하고 선형 감속장을
  정적 cap과 `min` 결합한다. 미관측/unknown/red/yellow/flashing/미허용 상태는 통과 허용으로
  추정하지 않는다.
- 객체 점유: 현재 OBB와 `[min_z, min_z + height]`를 CellTree의 실제 polygon 교차로 계산한다.
  ID별 Cartesian constant-velocity Kalman filter(선형 운동에서는 EKF의 특수형)로 6초를 예측하고,
  각 0.5초 구간의 시작·중간·끝 OBB의 convex hull을 swept footprint로 조회한다. 위치 공분산은
  설정한 sigma 배수만큼 footprint를 팽창한다. `heading`을 속도 방향으로 간주하지 않는다.
  두 위치 관측 전에는 속력×시간의 전방위 도달 반경을 점유시키고, 이후 위치 차이로 XY 속도를
  갱신한다.
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

## 실행

루트 `run.sh`가 HDMap 코어를 먼저 설치한 뒤 이 패키지를 colcon으로 빌드하고 통합 launch에
포함한다. 단독 실행은 다음과 같다.

```bash
export HDMAP_PATH=/absolute/path/to/hdmap.bin
ros2 launch hdmap_dynamic_tracker tracker.launch.py
```

## 아직 대회 주행에 사용하면 안 되는 이유

이 구현은 통신·배열·기하·예측 파이프라인을 검증하기 위한 1차 버전이다.

- `config/runtime.yaml`의 실제 설계 감속도와 제동표가 아직 `null`이다. tracker ROS 설정의
  2.0m/s²는 시뮬레이션 개발용이며 `signals.calibration_verified=false`다.
- 전체 UNKNOWN/GO/STOP_REQUIRED/HOLD/COMMITTED/CLEARING 상태기 중 현재는 보수적인
  permitted/stop 판단만 구현했다. 황색 통과 latch, 현재 접근로/movement 선택, 정지선 통과와
  clearing 판정은 경로 입력 또는 접근 Cell 판정 계약이 확정된 뒤 추가해야 한다.
- `Cell::previous()`는 유일한 predecessor만 제공한다. 합류부의 모든 상류 차선으로 감속장을
  퍼뜨리려면 Lanelet2 RoutingGraph predecessor 탐색이 필요하다. 필요한 감속거리 전에 체인이
  끊긴 controller도 관측 시 전체 0m/s로 fail-closed하므로 해당 접근에서는 주행이 막힐 수 있다.
- 지도에는 stopline 미해결 controller와 API green code/`permitted_states` 충돌이 남아 있다.
  이 경우 임의로 green으로 넓히지 않는다. 관측된 controller의 규칙 하나라도 미해결이면
  전체 cap을 0m/s로 내려 fail-open을 막으므로 정상 신호에서도 정지할 수 있다.
- 일부 stopline은 API가 동시에 관측하지 못하는 sibling controller와 공유된다. 기본
  `restrict_unobserved=true`에서는 관측 controller가 허용이어도 sibling UNKNOWN 제한이 남을 수
  있으며 노드가 throttle warning을 낸다. 주최측의 phase/movement 규칙 없이는 이를 임의 해제하지 않는다.
- DynamicStatus의 원시 배열은 현재 지도에서 프레임당 약 5.03MiB, 20Hz에서 구독자당 약
  100.6MiB/s다. 대상 Ubuntu/VTD 장비에서 p95 계산시간, DDS 지연/drop을 계측해야 한다.

위 항목이 해결되고 `config/runtime.yaml: allow_motion`을 별도로 승인하기 전에는 차량 구동을
허용하지 않는다.

## 테스트

ROS/Lanelet과 무관한 코어는 표준 C++17만으로 검사할 수 있다.

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
