# 코드 변경 설계안

2026-09-11. 실제 구현·컴파일·소스/설정 변경은 하지 않았다. 아래 타입, 함수 서명, 상수 이름은 **새로 제안하는 계약과 의사코드**다. 현재 존재하는 API와 구분해서 읽어야 한다. 기존 진단의 31개 재생 표본을 회귀 입력으로 사용한다.

## 변경 단위와 적용 순서

| 묶음 | 주요 파일 | 목적 |
|---|---|---|
| A | `src/control/src/mpc_control_node.cpp`, `src/control/test/control_core_test.cpp` 및 작은 노드 출력 검사 | 오류 시 최대 조향·전진 가속 차단 |
| B | `src/path_planner/src/path_planner_node.cpp`, `src/path_planner/test/cost_check.cpp` | route의 거리 기반 reference/허용 도로/목표 평가를 함께 변경 |
| C | 같은 planner 파일·테스트·필요한 Boost 의존성 선언 | 연속 기준선, 직접 곡선 샘플링, 곡률/차체/높이 검증 |
| D | `src/interfaces/msg/LocalTrajectoryPoint.msg`, `LocalTrajectory.msg`, interfaces 빌드 정의, planner/control의 송수신, controller core | 속도·시간·정지 상태를 실행까지 전달 |
| E | planner registry/node, tracker core/node, controller reset | 리스폰 입력/계산/발행의 일관성 |
| F | tracker 제동 램프 및 planner/control 설정·테스트 | 실제 제동 능력과 보호구역/신호 감속 일치 |

A는 독립적으로 먼저 적용한다. B의 reference·allowed·goal 변경은 한 묶음으로 비교한다. C~F까지 검증하기 전에는 전체 문제가 해결됐다고 판단하지 않는다. 지도 lanelet을 합치거나 MPC를 교체하는 작업은 이 설계에 필요하지 않다.

## A. 모든 제어 출력에 같은 제한 적용

현재 `publishFallback(reason, maximum_steering)`(mpc_control_node.cpp:298)의 최대 조향 분기와 8m/s 전진 목표를 제거한다. stale ego, invalid path, MPC failure는 초기 단계에서 기존 `publishStop()`으로 보낸다. 단순 정지 분기는 개선된 경로를 대신하지 못하므로 커브 이탈 여부를 함께 검증한다.

제안 필드:

```cpp
// 새 필드: 직전 실제 발행 명령. 분기에서 미리 바꾸지 않는다.
double last_published_steering_rad_{};
double last_command_steady_seconds_{};
```

`publishCommand()`에서 조향 제한과 가속 한도를 공통 적용한다. 기존 분기 안 `previous_steering_rad_ = ...`와 `publishStop()`의 중복 rate 계산은 이 위치로 모은다.

```cpp
// 설계용 의사코드. finite 검사와 최초 호출 dt 초기화는 먼저 처리한다.
const double dt = std::clamp(steady_now - last_command_steady_seconds_,
                            0.0, mpc_config_.dt_seconds);
const double bounded = std::clamp(requested_steering,
    -mpc_config_.maximum_steering_rad, mpc_config_.maximum_steering_rad);
const double emitted = approach(last_published_steering_rad_, bounded,
    mpc_config_.maximum_steering_rate_radps * dt);
// 명령 발행 시 emitted를 사용한다.
last_published_steering_rad_ = emitted;
previous_steering_rad_ = emitted; // 다음 MPC도 실제 제한된 값에서 시작
last_command_steady_seconds_ = steady_now;
```

nonfinite 요청은 발행하지 않고 제한된 중립 조향과 직접 제동으로 치환한다. timer가 장시간 멈췄다가 재개돼도 한 번에 큰 조향 변화가 허용되지 않도록 dt를 제한한다. reset에서도 실제 발행값의 이력을 임의로 0으로 덮어쓰지 않는다.

`LongitudinalController::reset()`은 적분뿐 아니라 이전 가속도도 0으로 만든다. 정상 계획 정지는 매 tick reset하지 않고 기존 `update()`로 수행한다. fault 진입 시 한 번 reset하고 긴급 제동은 직접 음의 가속을 발행한다. stale ego로 PID를 돌리지 않는다.

D 이후에는 명시적 상태에 따라 처리한다: 유효 DRIVE/STOPPING은 횡제어+목표속도 추종, HOLD는 정지 유지, INVALID/EMERGENCY는 제동. MPC 자체가 실패했으면 정상 횡제어가 가능하다고 가정하지 않는다.

## B. reference와 allowed를 후보별로 함께 구성

현재 `ReferenceSet`의 공통 allowed 집합을 후보별 reference로 옮긴다. 주행할 경로 A가 후보 B 때문에 허용되는 것을 막기 위해서다. 원본 `Primitive` 평행 배열을 전부 다른 컨테이너로 바꾸지는 않는다.

```cpp
// 기존 Reference에 추가할 메타데이터의 예
struct Reference {
    // 기존 points/station + C에서 추가하는 연속 평가 자료
    std::vector<lanelet::Id> lane_ids;
    std::set<lanelet::Id> allowed_lanes;
    double ego_s{};
    double target_s{};
};
// 기존 Primitive에 추가
// std::size_t reference_index;
// std::vector<double> z_m;
// std::vector<double> curvature_per_m;
```

새 `ReferenceBuilder::build(const PlanningSnapshot&, double forward_m, double rear_m)`는 다음 순서로 처리한다.

1. 새 current가 기존 route의 어느 진행 위치인지 확인한다. 없는 경우 route 재탐색을 요청하며 `following.front()`로 임의 연결하지 않는다.
2. 확인된 route predecessor를 뒤차체/투영 여유까지 포함한다. 모든 predecessor를 합치지 않는다.
3. 같은 route에서 직접 연결되는 following을 순서대로 더한다. 조건은 lane 개수가 아니라 **ego 투영점 이후 남은 길이**다.
4. 생성하려는 경로 길이와 제동·예측 범위를 확보하되, 기존 실행 경로 상한과 별개로 앞범퍼 여유까지 reference/허용 도로를 더 확보한다. 상한 때문에 제동거리를 확보할 수 없으면 속도를 제한하거나 infeasible을 명시한다.
5. route의 lateral edge는 중심선 끝끼리 이어 붙이지 않는다. 기존 left/right 후보가 합법적으로 목표 route에 합류할 수 있는지 검증해 별도 reference로 만든다.
6. route vector의 cursor로 진행하므로 길이가 짧은 lanelet이나 유한 route의 반복 ID 때문에 무한 탐색하지 않는다. 지도 topology 불연속은 명시적 실패다.

`upcomingMatch()`의 10개 탐색 한계를 정상 route iterator 진행으로 대체한다. `previous_next == current`만 정상 전진으로 보던 조건은 기존 route의 연속된 앞쪽 구간으로 진행했는지를 본다. 0.1m lanelet을 건너뛰어도 정렬 상태를 리셋하지 않는다.

투영은 raw polyline이 아니라 C의 연속 reference 위에서 수행한다. 제한된 앞뒤 검색 구간·heading·높이를 확인한다. s=0 clamp로 종방향 잔차를 숨기지 않으며, 생성 함수의 t=0을 평가한 XY가 ego를 재현하는지 확인한다. 첫 waypoint만 ego로 덮어써서는 안 된다.

### 목표 비용

현재 `goalPoints(goal_lanes)`에 대한 끝점 거리 비용을 중간 주행에서 제거한다. 후보의 진행량은 원시 polyline 길이가 아닌 **공통 글로벌 route에서의 station 변화량**으로 평가한다. 옆으로 볼록한 경로가 길어졌다는 이유로 진행 점수를 얻지 못하게 한다.

```cpp
// 새 API 개념: actual_terminal은 최종 checkpoint 접근에서만 존재
// evaluate(Primitive&, const RouteProgressContext&, optional<TerminalGoal>)
const double progress = end_route_s - ego_route_s;
const double desired = target_route_s - ego_route_s;
const double deficit = std::max(0.0, desired - progress);
const double progress_cost = config.progress_cost_weight * deficit * deficit;
// 기존 lateral/time/centerline/heading 비용을 더한다.
// 최종 목적지 접근 시에만 actual_terminal과의 거리/끝속도 조건을 적용한다.
```

`target_route_s`는 전방 계획 거리, 실제 최종 checkpoint, 첫 정지 제약 중 가장 먼저 도달해야 하는 위치로 정한다. 경로가 실제로 끝나는 경우는 선택 가능한 길이가 짧다고 무조건 벌점을 주지 않는다. `goal_lanes`가 담당하던 차선 선택 의도는 reference의 합법성/선호 조건에 남긴다.

`ids.size()==1 → goals={} → planner return`은 제거한다. 마지막 lane에 진입했어도 **최종 checkpoint XY를 해당 reference에 투영한 정지 station**까지 계획한다. 도착은 거리와 실제 속도 조건을 함께 확인한다.

## C. 기준선/샘플링/곡률/도로 포함을 같은 형상에 적용

현재 `sample()`은 polyline의 위치와 구간별 불연속 yaw를 반환한다. 그 상태에서 Frenet polynomial만 미분하거나 t 간격만 줄이는 것은 충분한 수정이 아니다.

선택하는 기본안은 **기존 Boost.Math의 cardinal cubic B-spline을 사용한 연속 reference 평가**다. 현재 환경에 해당 헤더가 설치되어 있다. 새 수학 라이브러리나 자체 spline solver는 추가하지 않는다. 빌드에는 실제 사용하는 Boost 의존성을 명시한다.

1. topology로 연결한 raw centerline을 중복점 제거 후 일정 간격으로 재표본화한다. x/y/z를 동일 parameter grid로 만들고 endpoint tangent는 연속 도로의 접선으로 준다.
2. x(u), y(u), z(u) spline을 구성한다. 샘플 부족 구간은 충분한 연결 구간 확보를 먼저 시도하고, 명백한 직선만 직선 모델을 사용한다. 극단적으로 짧은/불연속 곡선은 오류로 남긴다.
3. spline parameter u를 거리라고 가정하지 않는다. 실제 곡선 길이를 누적한 단조 lookup으로 s↔u를 대응시킨다. `sample(s)`와 `project(point)`는 같은 곡선을 사용한다.
4. 실제 후보점은 `q(t)=reference(s(t)) + normal(s(t))*d(t)`로 직접 평가한다. 현재 `appendWaypoint()`의 두 끝점 사이 직선 분할은 제거한다.
5. 공간 간격과 중간점 곡선 오차로 구간을 재귀/반복 분할한다. horizon, reference 끝, 실제 길이 상한에 도달하는 t를 구해 거기서 평가하므로 한 시간 step만큼 50m를 초과하지 않는다.
6. q의 실제 곡선 위에서 등거리 표본과 접선을 구한다. 곡률은 그 표본의 삼점 원 곡률 또는 동일 곡선의 도함수로 평가한다. 기본 구현은 현재 controller의 signedCurvature 계산식을 재사용한 삼점식으로 하고, h와 h/2 결과 수렴을 검사한다. 기준선 knot 양쪽도 검사한다.
7. 초기 heading, 전진성, 최대 곡률, `atan(wheelbase*kappa)` 조향 한도를 모두 유지한다. v=0의 시작점에서 시간 도함수 norm으로 나누지 않는다.

삼점식은 `kappa=2*cross(B-A,C-A)/(|AB|*|BC|*|CA|)`다. 분모가 작은 중복점은 DRIVE의 유효 곡률로 취급하지 않고, 정지 중복점은 HOLD 계약에서 처리한다. 단순히 collinear point를 지워 통과시키지 않는다.

Spline은 기하를 바꾸므로 다음 도로 검사가 필수다. spline이 인도 쪽으로 corner를 자르면 해당 reference/candidate를 거부한다. 원본 centerline과의 과도한 편차도 검출한다. 이 안의 spline 선택은 코드 구조를 구체화한 제안이며 실제 지도 전체에서 검증 완료된 것은 아니다.

### 높이와 전체 차체

`Reference`가 도로 z를 보존하고 `Primitive::z_m[i]`를 채운다. `VelocityPlanner`와 `CollisionValidator`는 현재ego.z 대신 해당 구간 차체가 차지하는 도로 높이 범위를 사용한다. 경사에서 앞/뒤 차체 높이 차이와 다른 층 도로를 구분한다.

허용 차선의 **동일한 도로 층 polygon**으로 corridor를 만든 뒤 `footprint - corridor`의 잔여가 명시한 작은 수치 허용치 이내인지 확인한다. 구현 시 이 프로젝트에서 발생한 Boost union 면적 오검출 표본도 회귀에 넣어야 한다. 네 모서리만 확인하면 중앙 교통섬을 놓치므로 불충분하다.

기존 “겹친 셀의 parent가 다른 lane이면 실패”는 평면 교차로에서 정상 경로도 거부할 수 있다. 도로 합집합 포함 검사를 차선 허용의 기준으로 삼고, 겹친 셀은 높이를 맞춘 점유·속도 검사에 사용한다. 교차로/초기1초 전체 면제를 제거한다. 빈 셀 조회는 unknown/missing-map 실패로 보고한다.

점 사이에서도 차체가 빠져나가지 않도록 샘플 간격을 차체 이동/회전량으로 제한하고 경계 근처는 추가 표본으로 검증한다. 이미 도로 밖인 ego를 일반 주행 경로로 억지 복구시키는 것은 이 수정 범위에 포함하지 않는다.

## D. 제어용 trajectory 계약

새 메시지는 두 개만 추가한다. 아래는 제안 msg 정의다.

```text
# LocalTrajectoryPoint.msg
geometry_msgs/Pose pose
float64 speed_mps
float64 time_from_start_s
```

```text
# LocalTrajectory.msg
std_msgs/Header header                  # 원본 ego stamp, frame_id=map
builtin_interfaces/Time dynamic_stamp  # 실제 사용한 dynamic의 원본 stamp
uint8 INVALID=0
uint8 DRIVE=1
uint8 STOPPING=2
uint8 HOLD=3
uint8 EMERGENCY=4
uint8 mode
float64 current_speed_limit_mps        # 현재 footprint의 원래 cap; 계획 첫 속도와 구분
LocalTrajectoryPoint[] points
```

새 `/local_trajectory`는 map 좌표를 사용한다. 현재 map 좌표인 `Primitive`를 그대로 발행하므로 제어기가 과거 ego를 찾아 base_link 경로를 다시 map으로 바꾸는 의존성을 제거할 수 있다. 기존 `/local_path`는 같은 선택 결과에서 시각화용 base_link 경로를 계속 만든다. 제어기는 새 토픽만 권한 있는 입력으로 사용하며, invalid 때 옛 Path로 자동 전환하지 않는다.

현재 `PathBuilder::build()`는 position.x/y와 orientation.z/w만 채우며 position.z에 속도를 담지 않는다. 새 메시지도 pose.z는 도로 높이에만 사용한다. `speed_mps`는 별도 필드다.

현재 Float32 `/speed_limit`는 제어용 권위 입력에서 제외하고, trajectory의 current cap과 dynamic_stamp로 신선도를 판단한다. 따라서 별도 StampedSpeedLimit 메시지는 추가하지 않아도 된다. 기존 speed_annotator의 출력은 시각화/호환 용도로 남길 수 있다.

interfaces CMake/package.xml에 두 메시지 및 builtin_interfaces 의존성을 등록한다. planner/control은 이미 interfaces에 의존한다. controller 입력 topic 설정을 바꾸고 경로/속도/상태 수신을 하나의 `onTrajectory()`에서 검증한다.

수신 계약:

- map frame, finite 좌표·속도·시간, 속도>=0, source age/receipt age/dynamic age, reset cutoff, 메시지 순서를 검사한다.
- DRIVE/STOPPING은 서로 다른 점 두 개 이상과 증가하는 시간이 필요하다. HOLD는 한 pose, speed=0, time=0으로 표현한다. INVALID/EMERGENCY는 경로가 비어도 의미가 있다.
- STOPPING의 마지막 속도는0이어야 한다. HOLD에서의 점 반복이나 ETA 무한대를 정상 moving 경로에 섞지 않는다.
- 현재 차량에서 지나치게 먼 경로·방향 불일치·이미 지나친 정지 목표를 거부한다. onEgo에서 ego z를 별도로 보관하고 현재 투영점의 보간된 road z와 비교해 다른 층의 trajectory도 거부한다. 모든 미래점의 z를 현재 ego z와 비교하는 방식은 쓰지 않는다. raw NaN 점을 제거해 경로를 이어 붙이지 않는다.
- 순서가 더 오래된 메시지의 무시는 별도 처리한다. 새 입력의 검증 실패·unknown mode·INVALID/EMERGENCY는 기존 DRIVE 실행 권한을 즉시 해제하고 제동한다. 단순 return으로 기존 trajectory를 계속 사용하지 않는다.

### VelocityPlanner 변경

```cpp
// 현재 bool plan(...) 대신 제안하는 결과
// PlanMode plan(Primitive&, const PlanningSnapshot&, optional<double> stop_s);
```

첫 0-cap 영역에 footprint가 닿기 전의 후륜축 station을 찾아 유효한 정지점을 삽입한다. 최종 목적지도 stop_s로 전달한다. 거기까지 geometry를 잘라 terminal speed=0을 강제하고, 기존 전방 가속/후방 감속 전파를 재사용한다.

이동 중에는 `eta[i]=eta[i-1]+2*ds/(v[i-1]+v[i])`를 정지점까지만 적분한다. 마지막 양의 거리 구간에서 두 속도가 모두0이면 이동 가능한 구간이 아니므로 해당 후보를 거부한다. 이미 정지해 대기해야 하는 경우는 처음부터 HOLD를 만들고, 정지 footprint의 미래 점유 bin을 검사한다. ETA 분모에 작은 양수를 넣지 않는다.

현재 속도가 해당 정지점에 도달 가능한 속도보다 높으면 EMERGENCY/정지 불가능 이유를 발행한다. 후보 전멸 때 침묵하지 않는다. geometry가 비정상인 경우에는 INVALID다. 프레임마다 원인별 후보 수는 기존 ROS 로그 또는 진단 출력으로 남기되 프로토콜에 범용 진단 프레임워크를 추가하지 않는다.

`FrenetPlanner` private `resize()`를 같은 파일의 공용 내부 함수로 옮기고 z/곡률을 포함해 모든 배열을 함께 자른다. 중간 정지점을 삽입할 때도 모든 속성을 같은 위치에서 평가한다. publish 전에 배열 길이·ETA 단조성을 확인한다.

### Controller core 변경

제안 데이터는 `TrajectorySnapshot`의 map 좌표 점별 pose/speed/time, source/dynamic stamp, mode, current_speed_limit_mps다. current cap은 finite 및0이상인지 검사하고 기존 overspeed_override의 비교값으로 사용한다. 프로파일의 첫 speed가 현재 실측속도라는 이유로 cap을 대체하지 않는다. `PreparedReference`에는 단일 valid 대신 `INVALID/TRACKABLE/STOP_ONLY/HOLD` 종류와 같은 미래 단계의 속도·곡률을 반환한다.

```cpp
// 제안 시그니처
PreparedReference prepare(const TrajectorySnapshot&, const Pose2&,
                          double now_s, double measured_speed_mps,
                          double dt_s, std::size_t steps);
MpcResult solve(double lateral_error, double heading_error,
                const std::vector<double>& predicted_speed_mps,
                const std::vector<double>& curvature,
                double previous_steering, const std::vector<double>* warm_start) const;
```

시간 원점은 수신 시각이 아닌 `header.stamp`다. geometry로 현재 투영 오차를 구하고, `tau=now-header.stamp`와 `tau+k*dt`에 대응하는 목표를 보간한다. 현재 pose와 시간상의 목표가 허용 범위를 벗어나면 재계획/제동한다. 끝을 넘는 외삽은 명시적 정지 trajectory의 속도0 유지에만 허용한다.

곡률 추정에 쓰는 공간 간격과 MPC 시간 단계는 분리한다. 현재0.5m 최소 간격을 그대로 각0.05초 step에 대응시키지 않는다. 같은 예측 진행 위치에서 v[k], kappa[k]를 가져온다.

`LateralMpc::solve()`의 a01=v*dt, b1=v*dt/wheelbase를 단계별 값으로 바꾼다. **전방 rollout뿐 아니라 adjoint 역전파와 gradient도 동일한 v[k]를 사용**해야 한다. warm start 길이가 바뀌거나 실행 출력이 제약 투영으로 달라지면 다시 투영/초기화한다.

종방향에는 v(t)와 그 기울기의 feedforward 가속을 쓰도록 `LongitudinalController::update()`에 인자 하나를 추가하는 안을 추천한다. 측정속도 오차의 PI 보정, 실제 가감속/jerk 제한은 유지한다. STOP_ONLY는 최소 MPC horizon만 부족한 유효 정지 경로이며 긴급 오류와 구분한다. HOLD는 실제 속도가 낮다는 조건도 확인하고 경사에서 정지 제동을 유지한다.

## E. 리스폰 및 입력 처리 순서

Planner `rclcpp::spin()`은 현재 단일 executor다. 별도 epoch나 이벤트 버스를 만들기 전에 기존 두 timer를 하나의 계획 callback으로 정리한다.

PlanningRegistry에는 작은 EgoStatus ring, shared_ptr로 보관한 latest/pending DynamicStatus, 최신 완성 입력쌍, reset_cutoff_stamp를 둔다. exact stamp로 짝을 맞춘다. **latest ego와 latest dynamic이 같아야만 실행**하는 구현은 tracker가 한 프레임 늦을 때 계속 기다릴 수 있으므로 쓰지 않는다. 완성쌍의 실제 age도 제한한다.

```text
planningTick():
  input = newest_complete_ego_dynamic_pair_after_reset()
  if absent or stale: publish INVALID; return
  current = direction_height_route_aware_match(input.ego)
  route = refresh_or_replan(current, input.ego)
  goals/progress = refresh_from_same_input(route, input.ego)
  generate -> geometry/corridor -> speed/time -> occupancy -> cost -> publish
```

매 계획마다 current/진행 위치는 갱신한다. shortestPath 계산만 기존10Hz 또는 route 이탈/리스폰/목표 변화 시에 수행한다. 10Hz 호출 횟수로 정렬 시간을 누적하던 updateGoals는 실제 경과시간으로 바꾼다. 교차로에서는 선택된 분기를 일관되게 유지하되, 이전 goal과 route 전체를 무조건 고정하지 않는다.

리스폰은 이전 정상 pose 대비 위치·높이·heading·dt의 물리적 연속성으로 감지한다. 이미 점프로 계산된 speed를 기준값으로 쓰지 않는다. 긴 수신 공백은 별도 재동기화로 처리한다. 감지 즉시 route/current/transient goal을 무효화하고, 새 ego만으로 가능한 글로벌 재탐색을 우선 수행한다. 로컬 생성은 reset 이후 완성 입력쌍을 기다리고 그 쌍의 ego로 route/current를 다시 확인한다. 이어서 planner의 정렬 이력과 controller의 warm start/적분/이전 경로를 비운다. **checkpoint 진행도는 일반 respawn에서 유지**한다. 명시적 전체 시나리오 재시작은 필요하면 시작 checkpoint로 초기화하는 별도 이벤트 의미를 갖는다.

Tracker `EgoSpeedEstimator::update()`는 jump를 distance/dt로 발행하기 전에 판정하고 speed0/history_reset을 반환하도록 확장한다. node는 이미 존재하는 predictor/signal_state_manager reset을 재사용한다.

Tracker는 두 스레드로 동작하므로 여기에는 추가 동기화가 필요하다. `receiveEgo` reset과 `processLatest`의 build/publish 사이를 기존 tracker_state_mutex로 보호하고, build 전과 발행 전에 snapshot stamp가 reset cutoff 이전이면 버린다. `buildDynamicStatus()`의 중복 잠금은 제거/재배치해 자기 교착을 피한다. 이미 꺼낸 과거 snapshot이 reset 후 predictor를 다시 채우지 못하게 한다. queue_mutex와 state_mutex의 잠금 순서도 일관되게 유지한다.

현재 bridge가 부여하는 단조·고유 수신 stamp에서는 cutoff로 충분하다. 시계 역행/동일 stamp 재사용까지 지원한다면 producer session id가 필요하다. 이는 현재 VTD restart의 기본안에 억지로 추가하지 않는다.

## F. 제동 램프와 설정

현재 tracker `stopRampCap()`은 속도를 남은 거리에 선형 비례시킨다. 이때 시작부 요구 감속이 설정값을 초과할 수 있으므로 design_deceleration의 숫자만 바꾸면 충분하지 않다.

기존 `conservativeBrakingDistance()`와 보정 테이블을 재사용해 다음 조건을 만족하는 속도 상한을 구하는 방식을 추천한다.

```text
B(v) + v * latency_budget <= distance_to_stop - stop_margin
```

B(v)는 실제 측정한 보수적 제동거리와 물리 모델의 보수적 값이다. 단조 함수이므로 [0, entry_speed]에서 이분 탐색으로 상한을 구할 수 있다. 테이블 범위를 넘어 외삽하지 않는다. 데이터가 없을 때 v²/(2a)를 쓰는 계산은 jerk/경사/지연 검증을 거쳐야 한다.

Tracker, planner, controller의 가감속 값은 동일 차량 능력을 기준으로 맞춘다. planner가 제어기보다 큰 가속/감속을 전제로 feasible 판정을 내리지 않게 한다. 측정 테이블의 감속 시작 이후 거리와 별도로 더하는 통신/명령 지연을 중복 계산하지 않는다. 이 수정은 보호구역의 미래 cap을 trajectory로 전달하는 D와 함께 검증한다.

## 회귀 및 완료 조건

기존 assert 기반 cost_check/control_core_test/tracker_core_test를 확장한다. 이름과 파일을 늘리는 것보다 실제 실패를 재현하는 최소 검사를 둔다. 원본 소스 복사본의 31표본 replay는 변경된 코드 경로를 직접 호출하도록 붙이고 기존 출력과 동일해야 한다고 assert하지 않는다. 바뀌어야 할 성질을 검사한다.

| 입력 | 반드시 확인할 조건 |
|---|---|
| 27224→27227→28004, 28075→28078→29001 | 충분한 전방 연결, 현재차량에서 시작, 정상 후속 도로 접촉으로 거부되지 않음 |
| 99175의58m/14m 후보 | 즉시 goal lane 거리 때문에 긴 정상 후보가 지는 현상 제거; 실제 진행 station 기준 평가 |
| 동일 곡선 h/h/2 sampling | 곡률 값과 판정 수렴, steering limit 유지, 가짜90° corner는 계속 거부 |
| 커브·경사·교차로 | 전체 footprint/높이 검사 통과; 보도 일부 침범·중앙 교통섬은 거부; union 오검출 입력 대조 |
| INVALID/stale/MPC failure | 양의 가속 없음, 모든 조향 출력 변화 제한, 다음 MPC는 실제 발행 조향에서 시작 |
| 북쪽217 두 정지 pose | 적색에서 유효 STOPPING/HOLD, 녹색에서 fresh DRIVE 재개, 평균속도0 ETA 무효화 반복 없음 |
| Dynamic 선도착/1frame지연/reset중 worker | 입력쌍 진행성 유지, reset 이전 결과 발행/상태 재유입 없음 |
| 최종 checkpoint와리스폰 | 실제좌표에서정지, lane 진입만으로publish중단안함, 새위치에서일관된global/local |

그 다음 같은5개 구간을 원본 시나리오 조건에서 반복한다. 차체 도로 이탈, 신호선 앞범퍼 여유, 보호구역 진입 속도, 조향/가속의 시간 변화, 경로 시작점·끝점·후보 탈락 이유를 함께 기록한다. 31개 표본 통과만으로 모든 주행을 보증하지 않는다.
