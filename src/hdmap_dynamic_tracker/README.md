# HDMap Dynamic Tracker

`/ego_pose`, `/objects`, `/traffic_light`를 받아 정적 HDMap Cell에 동적 상태를 겹쳐
`/ego_status`, `/dynamic_status`를 발행하는 ROS 2 C++ 노드다. Cell ID는 0부터 시작하며
출력 배열의 인덱스로 직접 사용한다. 개수는 지도에서 읽고 하드코딩하지 않는다.

## 현재 구현

- `/ego_status`: pose 입력마다 즉시 발행한다. 속도는 연속 source stamp 사이의 XY 이동량으로
  계산한다. 첫 샘플과 비양수 dt는 0이다.
- `/dynamic_status`: 같은 exact stamp의 세 입력만 결합한다. 최대 4개 stamp만 잠시 보관하고
  최신 완성 snapshot을 20Hz worker가 처리해 발행한다.
- 정적 speed cap: 각 Cell의 명시적 `speed_limit`을 Cell ID별로 캐시한다. 값이 없거나
  파싱할 수 없으면 지도 로딩이 실패한다.
- 신호 speed cap: 관측 controller와 일치하는 `signalRegistry()` 규칙만 적용한다. 정지선이나
  정지선 Cell 매핑이 없는 규칙은 건너뛰며, 전역 제한이나 미관측 controller 제한을 만들지 않는다.
- 황색 신호 상태기: 현재 앞차축이 위치한 접근 Cell과 controller 규칙을 결합해
  `UNKNOWN / GO / STOP_REQUIRED / HOLD / COMMITTED / CLEARING`을 관리한다. 황색을 처음
  관측한 순간의 정지/통과 판단은 해당 접근 동안 고정한다.
- 객체 점유: 현재 OBB와 6초 등속 예측 swept footprint를 Cell polygon과 교차시킨다.
  두 유효 위치 샘플부터 이동 방향을 바로 사용하며 관측 소실 track을 별도로 만료하지 않는다.

- 미래 객체의 yaw를 등속 모델이 예측하지 않으므로 L/W 반대각을 포함하는 외접 여유를 추가한다.
  미래 min-Z도 고정값으로 추정하지 않고 2D Cell 교차를 사용한다. 이로써 예측 중심 주위의 형상
  회전과 Z 변화로 인한 누락은 피하지만, 아직 관측되지 않은 미래 중심 궤적의 선회까지 결정론적으로
  보장하는 것은 아니다. 그 부분은 CV 공분산과 속도 잔차가 정한 통계적 envelope에 의존한다.
  현재 관측 bin 0만 실제 OBB와 Z 범위를 그대로 사용한다.
- `known_free_radius_m`가 양수이면 해당 반경 Cell을 free로 표시한다.


따라서 현재 Planner는 `-1`을 단순 free로 간주하면 안 되고, blocked 또는 별도 unknown 비용으로
처리할 정책이 필요하다. 반대로 모두 blocked로만 처리하면 기본 설정에서 경로가 나오지 않을 수
있다. 주최측 API가 보장하는 free-space 범위·30개 초과 표시가 확인되기 전에는 Tracker가 이를
임의로 결정하지 않는다.

모든 주행 토픽 QoS는 `BEST_EFFORT / VOLATILE / KEEP_LAST(1)`이며 출력 Header는 입력 Ego
Header를 계승한다. `occupancy[cell_id * 13 + 0]`은 현재, bin 1..12는 각각
`((bin-1)*0.5, bin*0.5]`초다.

## 정지선 선형 speed cap

실행 설정은 `config/tracker.yaml`의 `signals`다. `v_entry`는 정지 판단이 고정된 순간의
Ego 속도와 각 Cell의 정적 `speed_limit` 중 작은 값이며, 출력 cap은 정적 cap과 신호
감속 cap 중 작은 값이다.

```text
B_cal(v)   = VTD HyundaiIoniq6_23_Dyn의 감속 시작→완전 정지 거리
d_brake    = max(v² / (2 × a_design), B_cal(v))
d_profile  = braking_distance_factor × d_brake + v × latency_budget
d_start    = stop_margin + d_profile
cap(d)     = v × clamp((d - stop_margin) / d_profile, 0, 1)
```

기본 `a_design=11.0 m/s²`은 아이오닉 6의 최단 실측 100→0 km/h 제동거리 35.1m에서 얻은 값이다.
`braking_distance_factor=1`이 이 최단 제동거리이고, 값을 키우면 감속 구간이 같은 비율로 길어진다.
실측표는 속도 오름차순·거리 비감소여야 하며 표 사이에는 다음 상위 속도 bin을 사용한다.
표가 비었거나 범위를 벗어나면 설계 감속도 식을 사용한다.
기본 `stop_margin_m=7.0`이므로 정지선 전 7m부터는 0m/s cap을 적용한다.

`STOP_REQUIRED`에 진입한 순간의 속도를 고정해 접근로 전체 Cell의 감속 profile을 계산한다.
따라서 일반 Cell 상한이 30 m/s여도 50 km/h로 접근하다 황색에 정지 판단을 내리면
30 m/s가 아니라 13.89 m/s를 profile 진입 속도로 사용한다.

## 황색 신호 정지/통과 판단

`/ego_pose`는 IONIQ 6 후륜축 기준이므로 heading 방향으로 `wheelbase_m=2.944`를 더한
앞차축 위치를 사용한다. 앞차축이 속한 약 1 m Cell 안의 위치까지 투영해 정지선 잔여거리를
계산한다. 필요 정지거리는 다음과 같다.

```text
B_theory(v) = v² / (2 × yellow_decision_deceleration)
B(v)        = max(B_theory(v), B_cal(v))
d_required  = braking_distance_factor × B(v) + v × latency_budget + stop_margin
```

- 황색 최초 관측에서 `앞차축-정지선 거리 >= d_required`이면 `STOP_REQUIRED`로 고정하고
  기존 선형 감속 cap을 적용한다. 정지선 1 m 안에서 0.2 m/s 이하가 되면 `HOLD`다.
- 거리가 부족하면 `COMMITTED`로 고정한다. 신호 감속 cap만 제거하고 Cell의 정적 상한은
  그대로 유지하므로 `/speed_limit`과 Control은 해당 구간 상한 이하에서 통과 속도를 추종한다.
  Tracker는 가속 명령을 직접 발행하지 않는다.
- `COMMITTED` 뒤 적색으로 변해도 교차로 안에서 급정지하지 않는다. 정지선을 지난 뒤
  `CLEARING`을 거쳐 접근 상태를 초기화한다.
- 상태가 바뀔 때 controller/raw state, 현재 속도, 앞차축 거리, 필요 정지거리와 통과 시
  적용되는 정적 cap을 로그로 남긴다.

기본값에서 50 km/h(`13.89 m/s`)의 필요 정지거리는 약 `36.62 m`다. 이는
`3.0 m/s²` 감속, `0.25 s` 지연, `1.0 m` 정지 여유를 사용한 값이다. 실제 대회 전에는
VTD `HyundaiIoniq6_23_Dyn`으로 속도별 감속 시작→완전 정지 거리를 측정해 calibration 배열을
채워야 한다.

루트 `config/vehicle.yaml`과 `config/runtime.yaml`은 전체 시스템 설계/SimBridge 설정이고,
Tracker 실행값은 ROS parameter 파일인 `src/hdmap_dynamic_tracker/config/tracker.yaml`에서 읽는다.
VTD 보정 후 세 파일의 값과 검증 플래그를 함께 갱신해야 한다. 현재 통합 브랜치에서는
Speed Annotator가 현재 차량 footprint의 Cell cap 최솟값을 `/speed_limit`로 발행하고,
Control은 cruise speed와 정적/동적 상한 중 가장 낮은 속도를 추종한다.

## 실행

루트 `run.sh`가 HDMap 코어를 먼저 설치한 뒤 이 패키지를 colcon으로 빌드하고 통합 launch에
포함한다. 단독 실행은 다음과 같다.

```bash
export HDMAP_PATH=/absolute/path/to/hdmap.bin
ros2 launch hdmap_dynamic_tracker tracker.launch.py
```

## 테스트

ROS/Lanelet과 무관한 코어는 표준 C++17만으로 검사할 수 있다.
테스트에는 ego 속도 계산, 파서와 감속 profile, geometry, 객체 예측과 수치 입력 검사가 포함된다.

```bash
clang++ -std=c++17 -Wall -Wextra -Wpedantic -Werror \
  -I src/hdmap_dynamic_tracker/include \
  src/hdmap_dynamic_tracker/src/tracker_core.cpp \
  src/hdmap_dynamic_tracker/src/ekf_predictor.cpp \
  src/hdmap_dynamic_tracker/src/signal_state_manager.cpp \
  src/hdmap_dynamic_tracker/test/tracker_core_test.cpp \
  -o /tmp/tracker_core_test && /tmp/tracker_core_test
```

ROS 2 Jazzy 환경에서는 루트 `run.sh` 빌드 또는 `colcon test --packages-select
hdmap_dynamic_tracker`로 실행한다.
