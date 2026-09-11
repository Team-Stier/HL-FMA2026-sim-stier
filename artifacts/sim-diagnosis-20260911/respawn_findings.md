# 리스폰·글로벌 경로 진단 (소스 수정 없음)

## 1. 교차로로 리스폰하면 이전 글로벌 경로가 무기한 유지됨 — 원본 코드로 재현

- 원인: `RouteUpdater::tick()`이 새 위치의 lanelet을 찾은 뒤, `intersection && !input.global_path.empty()`이면 이전 `global_path`와 `goal_lanes`를 그대로 저장·발행하고 즉시 반환한다. 새 `current_lane`만 바뀐다. 위치 점프/리스폰 구분이나 기존 경로와 새 위치의 일치 검사는 없다.
- 근거: `/home/stier/HL-FMA2026-sim-stier/src/path_planner/src/path_planner_node.cpp:209` (특히 211–216). Ego 입력 저장만 하는 56–60, 925–926; 체크포인트와 정렬 상태 필드 297–303에도 리스폰 초기화 경로가 없다.
- 재현: 원본 CPP를 그대로 include한 `/tmp/respawn_probe.cpp`를 기존 build 컴파일·링크 설정으로 `/tmp/respawn_probe`에 빌드. 저장소 파일은 변경하지 않았다. `ROS_DOMAIN_ID=169`의 별도 노드가 `/offline_probe/global_path`에만 발행하여 실제 주행과 분리했다.
- 초기 위치 `(1310.821023, 464.269561)`에서는 `current=401819, intersection=false`, route head=401819, goal=401650.
- `(732.833513, -250.512312)`로 입력 위치를 바꾸면 `current=528793, intersection=true`인데 route head=401819, goal=401650으로 그대로다. 100회 추가 route tick 뒤에도 동일하다.
- 새 위치로부터 남은 최종 목적지까지 **57개 lanelet의 새 경로**가 존재함을 설치된 map/routing Python API로 별도 확인했다. 임시 C++ 검사에서는 첫 체크포인트를 다시 경유하는 더 엄격한 조건에서도 94개 lanelet의 경로가 발견되었다. 따라서 경로 단절 때문에 재계산이 실패한 것이 아니라 재계산 분기를 실행하지 않은 문제다.
- 영향: 글로벌 경로의 `[0]`이 현재 lanelet이라는 계약이 깨진다. `ReferenceBuilder::build()`의 408–412에서 새 current가 예전 경로에 없으면 원하는 successor를 찾지 못해 `following.begin()`을 택한다. 로컬 비용의 목표도 예전 위치의 goal lane에 남는다. 잘못된 분기·무경로 가능성은 소스에서 추론되며 이 probe는 실제 차의 이탈까지 시험하지 않았다.
- 재현 결과: `/tmp/respawn_probe_output.txt`.
- 재실행: 프로젝트에서 `source install/setup.bash` 후 `ROS_DOMAIN_ID=169 ROS_LOG_DIR=/tmp/respawn_probe_roslog /tmp/respawn_probe`.

## 2. 이번 실제 일반 도로 리스타트에서는 글로벌 재생성이 정상적으로 이루어짐

`/tmp/sim_diagnosis_20260911/events.jsonl`과 `actions.jsonl`에서 확인:

- 리스타트 후 첫 ego 수신: Unix time `1789070669.679868`.
- `(859.537537,-1.582902)` → `(508.799683,-168.287659)`로 388.339m 점프. 입력 간격 2.3588초.
- 글로벌 route head 29001 → 57627 재생성은 첫 새 ego로부터 **82.35ms** 뒤.
- 첫 로컬 경로는 **141.31ms** 뒤. 컨트롤러 `reset_detected=1`은 **28.51ms** 뒤.
- 따라서 모든 리스폰에서 재생성이 느린 것은 아니다. 위 1번의 교차로 조건이 별도로 확인된 실패 조건이다.
- 추가 실측 이상: 점프 직후 `/ego_status.speed=164.574539m/s`. 순간 이동 거리를 실제 속도로 계산한 값이다. 속도 추정기의 거리 점프 초기화 누락은 control 진단 담당자가 추적 중이다.

## 3. 마지막 체크포인트의 도착·정지 처리 — 소스에 명시적 처리가 없음

현재 CSV: `/home/stier/HL-FMA2026-sim-stier/src/path_planner/checkpoint/checkpoints.csv`

1. `(1310.818187,464.331725)` → lane 401819, intersection=no.
2. `(1307.840730,579.303653)` → lane 463065, **intersection=yes**.

- 체크포인트 반경 통과는 `checkpoint_index_ + 1 < checkpoints_.size()`인 중간 점에만 적용된다 (planner 218–221). 최종점 반경 검사·도착 상태·정지 명령은 없다. 마지막 좌표는 lanelet 선택에만 쓰이고, 경로의 최종 목적지는 `checkpoints_.back().lane`이다 (193–205, 227).
- 교차로 조기 반환(211–216)이 이 체크포인트 검사보다 앞선다. 따라서 기존 경로를 가진 채 교차로 안의 중간 체크포인트를 통과하면 순번 갱신 자체가 실행되지 않는 조건도 있다.
- 최종 lane에서 오래된 경로가 있으면 교차로 유지 분기 때문에 예전 goals가 유지된다. 별도의 도착 감속은 없다. `VelocityPlanner::plan(stop_at_end=true)`는 충돌 지점에서 후보를 잘랐을 때만 호출된다 (planner 765–768; 끝속도 0 지정 570). 최종 체크포인트를 보고 호출하는 경로는 없다.
- 일반 도로 최종 lane처럼 재탐색 결과가 lane 1개이면 goals가 비어(235–237) 로컬 planner는 아무 메시지도 내지 않고 반환한다(755–756). 이후 컨트롤러가 경로 timeout으로 `STOP_NO_LOCAL_PATH` 정지할 수 있으나 이는 체크포인트 좌표에 대한 계획된 도착 정지와 다르다 (control `mpc_control_node.cpp:326`).
- 지도 routing API 실측: 최종 lane 463065의 successor는 일반 도로 lane 412293이다. 그 successor에서 최종 lane으로 shortestPath를 구하면 31개 lanelet로 돌아오는 경로가 존재한다. 따라서 최종 교차로를 지나면 다음 일반 도로 tick에서 목적지로 순환하는 새 경로를 만들 조건이 성립한다. 실제 목적지 통과·재순환 **주행**은 아직 재현하지 않았으므로, 이는 **소스·지도에서 확인된 조건과 예상 결과**로 구분한다.

## 4. 체크포인트 교체·리스폰 시 주의할 현재 동작

- CSV는 `RouteUpdater` 생성 시 한 번만 읽는다 (189–207). 파일 교체 후에는 path_planner 프로세스를 재시작해야 반영된다. 실행 파일은 install share의 CSV를 참조한다.
- CSV의 `seq` 필드를 읽지만 정렬하거나 검증하지 않고 파일 행 순서대로 push한다 (198–205). 현재 파일은 순서가 맞으므로 이번 주행 결함의 원인은 아니다.
- 리스폰 시 `checkpoint_index_`는 남는다. 이미 지난 체크포인트를 새 미션으로 다시 시작할지, 진행도를 유지할지는 정책에 따라 결정되어야 한다. 현재 구현은 유지다.
- route 10Hz / planner 20Hz의 독립 timer를 사용한다 (938–944). 새 ego 수신과 다음 route tick 사이에 planner가 새 ego + 이전 route snapshot으로 돌 수 있다. 소스상 가능성이며 실제 잘못된 path 발행은 이번 자료에서 확인하지 않았다. README의 단일 tick 설명(28)은 현재 소스와 다르다.
- 컨트롤러는 점프 시 `resetState()`를 부르지만(192–196), 이는 planner에 전달되지 않는다. 컨트롤러 reset에는 `ego_history_` clear가 없어 과거 path 재수락 가능성도 control 담당자가 검토 중이다.

## 5. 선택적 실제 재현 위치 (지도 기준)

| 목적 | lane | x | y | z | yaw rad | yaw deg |
|---|---:|---:|---:|---:|---:|---:|
| 첫 checkpoint에 정렬된 일반 도로 시작 | 401819 | 1310.821022782271 | 464.2695607472982 | 55.661850518866 | 1.616383551075 | 92.611955550958 |
| 원본 코드에서 재현된 교차로 리스폰 | 528793 | 732.833512779967 | -250.512312272020 | 46.9 | -2.472698624910 | -141.675195215130 |

두 위치는 lanelet centerline 중간 구간이며 위 yaw로 vehicle footprint 검사에서 해당 lanelet이 선택됨을 원본 코드로 확인했다. 첫 위치는 checkpoint0에서 약 6.22cm로 2m 반경 안이다. z는 지도 centerline의 노면 높이이며 시뮬레이터 API가 차량 중심점을 쓰는 경우 기존 차체 기준점 offset을 적용해야 한다.

권장 관측값: `/ego_status`의 jump 거리/속도, `/global_path`의 head와 새 위치 lanelet 포함 여부, `/local_path`의 stamp 및 map 변환 후 시작·끝점, `/control/status` reset_detected/state. 교차로 재현은 기존 nonempty 글로벌 경로를 유지한 채 차량 위치만 교차로로 옮겨야 하며 path_planner를 함께 재시작하면 guard의 old path가 비어 있어 다른 조건을 시험하게 된다.

## 6. 두 번째 실제 리스타트: 새 pose + 이전 lane의 경로가 생성·수락됨 — 실측 확인

- Unix t=1789070862.266604: 마지막 이전 global route head=398690.
- t=1789070862.302210: 새 ego `(508.799683,-168.287659)`, heading=0.516169429rad, speed=662.383667m/s, stamp=1789070862300254950.
- t=1789070862.324333 및 .380139: 이 **새 ego stamp**를 단 local path 두 개가 발행됨. 첫 점은 base_link `(266.336924,516.034487)`로 원점에서 약 **580.7m** 떨어져 있음. 각 645점, observer capture_error_ms=0.
- t=1789070862.382405에야 새 global route head=57627이 발행됨.
- controller는 t=.359/.410에 MPC_FAILURE fallback을 출력했지만, 새 speed 표본이 0에 가깝게 내려온 뒤 .458729/.508366에는 **ACTIVE**로 위 잘못된 경로를 수락함. 원점에서 시작하는 정상 path는 .546279에 처음 관측됨.

이는 교차로에서 장시간 유지되는 1번과 구별되는 **비동기 갱신으로 인한 혼합 snapshot 문제**다. `PlanningRegistry::onEgo()`는 snapshot의 ego만 교체(56–60), route/current_lane은 다음 route tick에서 교체(83–90), route 10Hz와 planner 20Hz timer가 독립(938–944)이다. `FrenetPlanner::tick()`은 새 ego와 이전 current_lane reference를 함께 소비(755–758,792)하고 `PathBuilder`는 새 ego header로 stamp한다(699–705). 따라서 오래된 path 메시지가 늦게 도착한 경우와 달리 타임스탬프 검사를 통과할 수 있다.

결론: 4번에서 가능성으로 적었던 혼합 snapshot 경로 발행이 실제 자료로 확인되었다. 컨트롤러가 origin과 path의 과도한 거리를 거부하지 않은 부분은 control 진단과 함께 다룬다.

## 7. 교차로 유지 조건의 실제 차량 재현 — 6.426초간 이전 경로 유지

- t=1789071055.996540: 시뮬레이터 Ego 위치를 위 528793 교차로 지점으로 바꾸는 SCP 명령. Traffic 로그가 적용을 확인함.
- t=1789071056.040495: 새 ego 수신. 순간 이동 거리 775.630m / 입력 간격 0.078919s, 잘못 추정된 speed=9772.470703m/s.
- 직전 global head=394852가 새 ego 이후에도 **6.425707초** 유지됨. 차량이 교차로를 지나 t=1789071062.466202에 일반 도로 head=12504로 처음 재탐색됨.
- 리스폰 후 처음 세 local path는 **각 5점**, 길이가 각각 **0.0547m, 0.0836m, 0.1064m**이고 그 다음 7점 경로도 0.0212m다. 새 ego로부터 +0.118초에 `FORWARD_INVALID_PATH:path is too short for minimum MPC horizon`으로 진입한다. 경로가 짧은데도 forward fallback으로 주행하는 동작은 control 진단 대상이다.
- 따라서 1번의 오프라인 교차로 유지 조건이 실제 시뮬레이터에서도 재현되었다. 새로운 위치를 인식하지 못한 것이 아니라 새로운 intersection current와 이전 route/goals를 함께 유지하다 교차로 밖에서만 route를 바꾼다.
- 그 다음 첫 checkpoint 일반 도로로 옮긴 t=1789071096.080358의 ego에서는 route head=401819로 +82.04ms, 첫 정상 local path(47점, first=(0,0))로 +132.25ms에 복구했다. 순간이동 속도는 이때도 23656.457031m/s였다.

## 8. 신호 위상 조작의 검증 범위

초기 run2/3의 강제 적색 SCP 요청은 적용 성공이 확인되지 않았으므로 그 요청만으로 적색 정지 여부를 판단하면 안 된다. 설치 공식 문서상 XML은 유효하지만 Traffic SetPhase/SetCtrl 처리 로그가 없었다. parent의 후속 시험은 controller136의 **자연 적색**에 맞춰 출발을 15초 늦췄으며, 이 자료로 정지선 전 정지·잘못된 경로 후 재출발 실패를 별도로 확인했다. 위상 조작 실패는 차량 제어 실패와 구분한다. 상세 문법·시나리오 주기는 `/tmp/sim_diagnosis_20260911/traffic_override_findings.md`.
