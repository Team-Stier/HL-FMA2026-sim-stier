# 제어·속도·신호 진단 (소스 수정 없음)

근거: `events.jsonl` 라이브 관측, `actions.jsonl` 시뮬레이터 조작 기록, 저장소 소스 및 기존 C++ 테스트. 아래 timestamp는 기록 파일의 Unix epoch seconds이다. 실제 주행에서 확인한 사실과 코드상 위험을 구분한다.

## 1. 실측: 짧은 경로가 최대 조향과 재가속을 유발함 — 최우선

`src/control/src/mpc_control_node.cpp:298-314`는 fallback 목표를 최대 8m/s로 설정한다. `:343-345`의 invalid path는 `maximum_steering=true`를 넘기므로 직전 조향의 부호로 즉시 ±0.48rad가 된다. 정상 MPC 조향 변화 한도는 0.4rad/s×0.05s=0.02rad인데 fallback은 이 한도를 건너뛴다. 곡률 속도 제한과 overspeed override도 실행하지 않는다.

- t1789070712.475: 속도3.867m/s, 조향+0.03237rad, 제동-3.0, local path5점.
- t1789070712.508: `FORWARD_INVALID_PATH:path is too short for minimum MPC horizon`, 조향+0.48rad, 제동-2.85. 이후 제동을 0.15씩 줄이며 전진 목표로 돌아간다.
- t1789070732.458: 같은 오류로 조향-0.48rad. 약1초 뒤 heading 오차24.85°, 현재 path까지 거리0.91m로 벌어지고 stale path 정지로 이어졌다.
- t1789070806.858~1789070807.208: (1121.46,434.31) 부근, 속도0.67~0.72m/s, 동일 오류·조향+0.48rad를 유지하면서 가속명령+0.0667→+1.1167m/s². 이후 경로 단절로 정지했다. 따라서 “문제 경로에서 조향을 끝까지 꺾고 다시 출발”하는 현상은 실제 명령까지 확인되었다. 물리적 보도 진입 여부는 지도/영상 판정과 별개다.

## 2. 실측: MPC 실패 후에도 양의 가속을 계속함

`mpc_control_node.cpp:353-355`는 solver 실패 시 조향을 0으로 풀면서 위 전진 fallback을 적용한다.

- t1789070675.458: `MPC iteration limit reached`, 속도10.326m/s, 조향0, 가속+1.85m/s². t1789070675.808까지 +0.8m/s²이며 속도는10.86m/s. 목표 fallback8보다 빠른데도 직전 가속의 jerk 제한 때문에 한동안 가속한다.
- `controller_core.cpp:375-377`의 반복한도 실패를 /tmp 독립 probe로 재현: 직선·횡오차0.5m·heading오차0·warm start 없음·기본400회에서 속도2,15,20m/s가 실패. 5,8,10은 성공. 시간제한이 아니라 반복한도 문제를 분리하기 위해 probe만 time budget1000ms; 실제 소요<0.1ms였다.
- `/tmp/diag_control_sweep.cpp`, `/tmp/diag_control_sweep` 참고. 기존 control core 테스트는 통과하며 실차 ROS fallback을 검사하지 않는다.

## 3. 실측: 보호구역 8m/s cap은 적용되지만 진입 전 감속은 보장되지 않음

학교 구간 정적 cap8은 지도에 존재하고 `/speed_limit`30→8 전환 및 감속을 관측했다. 한 구간(654,-102.6 부근)을 세 차례 지나며:

| 진입 t | cap8 전환시 속도 | 8.2m/s 이하까지 | 이동 거리 |
|---|---:|---:|---:|
| 1789070692.880 | 9.962m/s | 0.642s | 5.84m |
| 1789070885.621 | 10.427m/s, 이후 최대10.656 | 1.441s | 14.11m |
| 1789070945.523 | 10.050m/s | 0.799s | 7.39m |

`src/speed_annotator/src/speed_annotator_node.cpp:33-47`는 현재 차량 footprint에 걸친 셀의 최솟값만 발행한다. 앞으로의 낮은 제한을 미리 전달하지 않는다. `path_planner_node.cpp:543-584`가 계획한 speed/eta는 `:699-714`의 `/local_path`에 포함되지 않고, controller는 위치/자세와 현재 cap만 받는다. 따라서 경로 내부에 미래 속도계획이 있어도 제어에 연결되어 있지 않다. cap8에서 정상 목표는 margin을 뺀7.5m/s이고 평온한 구간에서는7.2~7.6m/s 추종을 관측했다. 상세 수치는 `school_entry_metrics.json`.

## 4. 실측: 리스폰 직후 속도 spike와 멀리 떨어진 local path 승인

`src/hdmap_dynamic_tracker/src/tracker_core.cpp:74-86`은 위치차/수신시각차만으로 속도를 계산하며 공간 점프를 거르지 않는다. 기존 `tracker_core_test.cpp:70-72`도99m/0.1s=990m/s·history_reset=false를 assert한다. 이로 인해 tracker의 predictor/signal reset(`hdmap_dynamic_tracker_node.cpp:394-400`)도 위치점프에는 실행되지 않는다.

- 1차 재시작 t1789070669.680에164.575m/s, 2차 t1789070862.301에662.384m/s, 3차에348.87m/s가 기록됐다.
- 2차 t1789070862.324/862.380 local path는 새 Ego=(508.7997,-168.2877)와 정확히 같은 capture stamp(capture_error_ms=0)인데 첫점 base_link=(266.3369,516.0345), 580.7m 떨어져 있었다. 즉 단순 observer 시각매칭 오차가 아니다.
- t1789070862.459/862.508 controller 상태는 ACTIVE이고 실제 speed0~0.0044m/s. 경로 가까움 판정 없이 오래된 위치 근처의 경로를 새 stamp로 수용하는 현상이 실제 확인된다.
- `controller_core.cpp:119-162` 최근접 선택은 heading을 검사하지만 최대 횡거리/종거리 검사가 없다. /tmp probe에서 평행 path가50m 떨어져 있어도 valid=true.
- 별도 잔여 위험: controller resetState(`mpc_control_node.cpp:251-260`)가 ego_history를 비우지 않아 지연 도착한 reset 전 path도 capture pose를 찾고 require_new_path를 해제할 수 있다(`:217-238`). 이번580m path는 이 경우와 구분해야 한다(새 stamp로 잘못 만들어진 path).

## 5. 신호 테스트 결과와 한계

ctrl136(lanelet34407, XODR road174/+1, stopline542472, 허용state3/5)의 stopline 중심은(612.114526,-116.186207), 접근방향0.418981rad이다. 세 차례 앞범퍼가 선을 통과한 시각과 raw는:

- t1789070685.920: state2(황색),7.198m/s
- t1789070878.620: state2(황색),7.284m/s
- t1789070938.643: state2(황색),7.378m/s

각 경우 `COMMITTED` 후 통과했다. 1차 황색 결정 당시 앞축거리9.46m·요구정지거리15.74m여서 통과 결정은 현재 설계에 맞는다. SCP red/freeze 요청 후에도 raw는3→2로 진행했으므로 빨간불 강제 테스트는 설정이 적용되지 않았다. 이 세 통과를 적색신호 위반으로 보고하면 안 된다. Ctrl143 raw red는 해당 차선33446(좌회전, 허용4/5)에 접근하는지 함께 봐야 한다.

교차로 relocation 후 ctrl41 raw1에서 STOP_REQUIRED 및 cap0·차량정지는 관측됐지만 정상 접근 정지시험이 아니다. relocation heading=-141.675°는 ctrl41 차선 방향+38.325°의 반대이고, 최종 차량도-100.32°를 향했다. 최종(711.0885,-272.0828) 앞범퍼는 ctrl41 stopline 평면의 정상 진행방향 기준10.49m 전이다. 이 결과만으로 정상 주행 신호정지 성공/위반을 결론내릴 수 없다.

코드상 정지 여유 불일치: `src/hdmap_dynamic_tracker/config/tracker.yaml:15-21` ramp 설계감속11m/s²·latency0, controller 최대제동은3m/s². `tracker_core.cpp:169-201`은 v²/(2a)길이에 속도를 선형으로 떨어뜨려 ramp 진입에서 순간적으로 약2a의 감속이 요구된다. 이 구성은 늦고 급한 속도제한을 만든다. 다만 정상 적색 접근의 실제 정지실패까지는 이번 관측에서 검증하지 못했다.

## 6. 추가 코드상 위험

- `speed_annotator_node.cpp:23-26,30-48`는 ego와 dynamic 최신값을 시각/timeout 검증 없이 섞는다. dynamic 갱신이 끊겨도 ego마다 과거 cap을 새 unstamped Float32로 다시 발행하므로 controller receipt timeout은 데이터 신선도를 보장하지 못한다.
- `mpc_control_node.cpp:320-336`: stale ego와 stale speed cap도 전진 fallback. 재시작 동안 stale ego 상태에서+0.15→+2.0m/s² 발행을 실측했다. SimBridge 자체의 입력 timeout 제동이 별도로 있으므로 이 명령 모두가 VTD에 적용됐다고 단정하지 않는다.
- `controller_core.cpp:409-414`의 path-end 감속은 남은 거리가1초 MPC preview보다 짧아진 뒤에만 시작한다. 예를 들어15m/s·남은20m이면 이 제한이 없지만 최대제동3에서 필요한 정지거리는37.5m. 고정 local endpoint와 결합하면 감속이 늦을 수 있다.
- `control/README.md`의 출력비활성/실패시제동 설명은 실제 현재 설정·fallback 동작과 다르다.

## 검증/산출물

저장소 소스 및 설정을 수정하지 않았다. 기존 독립 `control_core_test`와 `tracker_core_test`를 /tmp에 빌드하여 모두 통과. 진단 probe와 결과는 모두 /tmp이다. `tracking_metrics.json`의 일반구간 median distance가 약1mm인 것은 경로를 매번 Ego 위치에서 시작하기 때문이며 차선 추종 정확도 보증으로 쓰지 않는다. 실제 실패시 stale path에서 거리0.91m·heading24.85°를 확인했고, respawn의580m 항목은 별도 오염 사례다.

## 7. 추가 실측: 자연 적색 정지 시험 — 정지선 전 정지하지만 차선 이탈 및 초록불 재출발 실패

강제 phase 설정 대신 MPC 프로세스만 잠시 suspend하여 신호주기가 먼저 진행되도록 한 뒤 정상 주행을 재개했다(조작은 actions.jsonl). 이번에는 ctrl136 raw1을 실제로 확인했다.

- t1789071194.031: `STOP_REQUIRED`,7.37m/s, 앞축거리49.41m, 계산 정지거리16.05m. `/speed_limit`은7.370→3.880→0.895→0으로 감소했다. 적색 신호 감속 명령 전달 자체는 동작했다.
- t1789071199.711/1199.759: 정지 경로가 짧아지자 `FORWARD_INVALID_PATH`, 속도5.69/5.60m/s, 조향+0.48rad. 아직 제동 중이었으나 full-steering fallback이 실제 정지시험에도 개입했다.
- 이후 path가 stale이 되어 `STOP_NO_LOCAL_PATH` 및-3m/s². 앞축이 원래 접근차선 밖으로 나가자 t1789071200.630 tracker는 `STOP_REQUIRED→UNKNOWN`으로 바뀌고 red인데 cap0→8로 돌아갔다. 원인 코드는 `activeSignalApproach`가 현재 앞축의 제한셀 소속만 판단(`hdmap_dynamic_tracker_node.cpp:626-663`), 범위를 벗어나면 state manager가 stop 상태를 해제(`signal_state_manager.cpp:103-112`)하기 때문이다.
- t1789071201.682에(601.337952,-119.501213),heading0.761122rad에서 정지. stopline542472 기준 앞범퍼 중심은7.606m 전, 가장 앞으로 나온 앞모서리도 약7.289m 전이라 **이번에 적색 정지선을 넘지는 않았다**.
- 다만 앞범퍼 중심은 해당 정지선 중심축의 왼쪽2.634m, 차선 방향과 heading 차이는19.603°. 관측 footprint가 원래34407과 반대차선34191 양쪽을 차지한다. build_report에서34407은road174/+1,34191은동일road174/-1이다. global route도34191을 새 출발차선으로 바꾸었다.
- t1789071205.800에 ctrl136 raw3(해당 차선의 허용 초록불)가 되었으나 `STOP_NO_LOCAL_PATH`,speed0이 계속되어78초 이상 재출발하지 못했다(관측 시점 기준).

판정: 신호 감지와 제동은 실행됨. 올바른 차선에서 안정적으로 정지하고 초록불에 재출발하는 전체 동작은 실패. 주원인은 짧아진 정지 경로→최대조향 fallback→반대차선 침범→신호 접근상태 소실→새 경로 생성 실패의 연결이다. `natural_red_result.json`에 해당 명령·초록불·정지 좌표를 보존했다.
