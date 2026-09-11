# 주행 경로·제어·도로 이탈 진단 — 최종 보완

소스와 프로젝트 설정은 수정하지 않았다. 아래 clean run은 같은 원본 controller binary/config를 **새 프로세스로 시작한 주행**이다. SIGSTOP/CONT 재사용 중 발생한 DDS 수신 중단 사례와 분리했다. 모든 분석 도구와 산출물은 `/tmp`에 있다.

## 확인된 핵심 원인

로컬 경로가 짧아져 유효성 검사를 실패하면, controller는 기존 작은 조향의 부호만 받아 **최대 ±0.48 rad 조향**을 즉시 명령한다. 동시에 정지 대신 최대 8 m/s를 향한 속도 제어를 계속한다. 다음 tick에 MPC가 정상 복귀하더라도 조향은 정상 제한인 0.02 rad/tick으로 천천히 복귀하므로, 잘못된 큰 조향이 약 1.2초 동안 남는다. 도로 이탈 이후 경로가 끊기고 timeout이 나서야 강제 -3 m/s² 제동한다.

직접 근거:

- [mpc_control_node.cpp:298](/home/stier/HL-FMA2026-sim-stier/src/control/src/mpc_control_node.cpp:298): `maximum_steering`이면 rate limit을 건너뛰고 기존 조향 부호의 최대 조향을 즉시 대입한다. :310–314는 최대 8 m/s 목표로 종방향 제어를 계속한다.
- [mpc_control_node.cpp:343](/home/stier/HL-FMA2026-sim-stier/src/control/src/mpc_control_node.cpp:343): reference가 invalid이면 위 최대조향 fallback을 호출한다.
- [controller_core.cpp:282](/home/stier/HL-FMA2026-sim-stier/src/control/src/controller_core.cpp:282): 정상 MPC의 조향 변화 제한. 따라서 fallback에서 만든 급격한 조향을 ACTIVE 복귀가 즉시 제거하지 못한다.
- [mpc_control_node.cpp:326](/home/stier/HL-FMA2026-sim-stier/src/control/src/mpc_control_node.cpp:326): 경로 timeout 이후에야 `publishStop`으로 전환한다. [287](/home/stier/HL-FMA2026-sim-stier/src/control/src/mpc_control_node.cpp:287)에서 -3 m/s² 제동하며 조향은 서서히 0으로 되돌린다.
- [controller_core.cpp:409](/home/stier/HL-FMA2026-sim-stier/src/control/src/controller_core.cpp:409): 경로 끝 감속은 남은 길이가 요청 preview보다 짧을 때만 적용한다. 기본 1초 preview보다 제동거리가 긴 속도에서는 대응이 늦다. 예: 15 m/s, 남은 길이 20 m에서는 끝 감속이 적용되지 않지만 3 m/s² 정지거리만 37.5 m다.

![깨끗한 두 주행의 최대조향과 도로 이탈](/tmp/control_roadscan/clean_departure_evidence.png)

## Clean run 위치별 재현

차체는 후축 기준 앞 3.808 m / 뒤 1.040 m / 폭 1.886 m로 검사했다. Vehicle canPass lanelet polygon 합집합과 비교하고 높이가 2 m 이상 어긋나는 lanelet은 제외했다. 주요 위치는 `lanelet2.geometry.inside`를 차체 내부 60×24 격자에 독립 적용해 검증했다. 이는 **주행 가능 지도 영역 밖으로 차체가 나간 증거**다. 해당 표면이 실제 인도인지, 교통섬인지에 대한 명칭은 OSGB/영상 대조와 구분한다.

| 위치 / clean case | 실제 현상 | 독립 격자 확인 | 제어와 경로 상태 |
|---|---|---:|---|
| (857,-5) → (861.089,0.345), curve_28004_north | 차체 99.83% 도로 밖 | 99.86% 밖 | 짧은 경로 → INVALID_PATH 최대조향 → ACTIVE 복귀 후 도로 이탈 → timeout STOP |
| (857,121) → (859.948,130.402), 같은 case의 다음 커브 | 약 7.12초 모서리 침범, 최대 차체 28.14% 밖 | 28.68% 밖 | ACTIVE 중 계속 진행, 최대 침범 시 path age 0.400초 |
| (1016.378,-1183.723), south_large_curve_road1909 | 작은 앞 모서리 침범, 차체 2.54% 밖 | 2.85% 밖 | ACTIVE, 신선한 path age 0.120초 |
| (1097.483,-1137.100) → (1104.551,-1134.039), 같은 남쪽 case | 가속하며 이탈, 최대 차체 92.31% 밖 | 92.29% 밖 | 반복 MPC 실패와 짧은 경로 → 최대조향 / 양의 가속 → timeout STOP |
| (566.503,-385.839), west_steep_downhill_road1602 | 차체 중심이 지도 비주행 공간을 통과; 최대 71.64% 밖 | 71.53% 밖 | 이 구간에서는 최대조향 fallback 없이 경로 timeout 후 제동 중 통과. 네 모서리가 서로 다른 도로 조각에 걸리는 형태 |
| (566.250,-383.457), 같은 내리막의 최종 정지 부근 | 차체 5.25% 밖 | 5.07% 밖 | STOP_NO_LOCAL_PATH |
| west_eastbound_intersection_road76 | clean run에서 차체 도로 이탈 검출 없음 | — | 정지 여부와 목표 도달 여부는 부모 통합 보고서 참조 |
| (1447.083,865.332), north_uphill_road2819 녹색 재출발 이후 | 차체 68.51% 밖, 중심 밖 | 68.47% 밖 | 짧은 경로 → +0.48 rad 최대조향 → timeout STOP |
| (1454.143,852.033), 같은 북쪽 최종 정지 | 앞 모서리 침범, 차체 17.71% 밖 | 17.71% 밖 | STOP_NO_LOCAL_PATH |

### 첫 번째 clean 최대조향 사례

시각은 기록의 Unix epoch 초다.

- t1789074329.764: ACTIVE_SHORT_PATH, 조향 -0.0354 rad, 가속 -1.196 m/s².
- t4329.814: INVALID_PATH 단 한 tick에 조향 -0.4800 rad. 변화 0.445 rad는 정상 tick 제한 0.02 rad의 약 22배다. 가속은 -1.046으로 제동이 약해진다.
- t4329.864부터 ACTIVE 복귀, 조향은 -0.46, -0.44, -0.42…로 천천히 회복한다.
- t4330.060: (857.310,-4.962), 6.62 m/s, 첫 앞 모서리가 도로 밖으로 나간다. 최신 path age는 0.040초다.
- t4330.764: STOP_NO_LOCAL_PATH / -3 제동으로 전환.
- t4331.301: (861.089,0.345), 3.58 m/s, 차체 99.83% 밖. 마지막 path는 1.281초 전 것이다.

같은 위치는 baseline 두 번(차체 99.49%, 98.85%), 이전 새 checkpoint run(99.58%), 이번 clean run(99.83%)으로 반복했다. 시험 조작에 따른 수신 중단을 제거해도 재현된다.

### 남쪽 clean 가속 이탈 사례

- t4415–4416: `MPC iteration limit reached`와 ACTIVE가 반복된다. 실패 fallback에서도 양의 가속이 유지된다.
- t1789074416.392 / .442: 짧은 path INVALID_PATH, 조향 -0.48 rad, 가속 +1.55~+1.70 m/s².
- 이후 ACTIVE 복귀에서도 조향의 느린 복귀와 +2.0 m/s² 가속이 겹친다. 속도는 3.59 → 5.25 m/s까지 증가한다.
- t4417.181: (1097.483,-1137.100), 4.88 m/s, 첫 모서리 도로 이탈.
- t4417.843: STOP_NO_LOCAL_PATH로 -3 제동. 이미 도로를 벗어나고 있다.
- t4418.867: (1104.551,-1134.039), 2.36 m/s, 차체 92.31% 밖, path age 1.765초.

'인도로 급발진'으로 보이는 현상을 설명하는 가장 직접적인 clean 사례다. 가속과 최대조향 명령이 실제로 함께 발생했으며, 로컬 경로 유효성 실패에 대한 controller 정책에서 나온다.

### 북쪽 녹색 재출발 이후

t1789074630.858에 짧은 path INVALID_PATH, 직전의 약 +0.00012 rad 조향이 +0.48 rad로 바뀐다. t4630.908부터 `no usable path remains ahead`로 같은 큰 조향을 유지한다. t4631.180, (1445.432,868.883), 10.51 m/s에서 첫 모서리가 나가고, t4631.508에야 timeout STOP으로 전환한다. t4631.580 최대 68.51% 도로 밖이다. 이 구간은 양의 가속 급발진은 아니지만, 작은 조향 부호가 최대조향을 결정하는 문제를 세 번째 독립 경로에서 재현한다.

## 경로가 좋아 보여도 추종이 무너지는 이유를 구분

1. **경로 끝에서 제어 실패 정책이 직접 큰 오차를 만든다.** 위 clean 세 사례는 정상처럼 보이는 짧은 path를 지나며 INVALID_PATH가 발생하고 controller가 강한 회전을 명령한 뒤 이탈한다. 단순히 MPC가 모든 경로를 못 따라가는 사건으로 해석하면 핵심을 놓친다.
2. **커브에서는 planner도 도로 밖 차체를 허용한다.** Baseline (857,122) 커브의 모든 50 ms path를 검사했다. t1789070749.565까지 첫 12 m 차체가 도로 안이지만, t0749.614부터 미래 2.95 m 지점에서 차체 1.31%가 밖인 path를 발행한다. 실제 첫 이탈은 t0749.800, 즉 0.186초 뒤다. t0750.463 path는 미래 최대 16.35%, t0751.462 path는 최대 22.76% 밖이다. 따라서 이 사건은 경로 생성과 추종 오차가 함께 악화되는 경우다.
3. **현재 path까지의 아주 작은 오차는 회복 성능을 증명하지 않는다.** 새 path가 매번 현재 Ego에서 시작하기 때문이다. 마지막으로 검증된 안전 path(t0749.466)를 고정하면 첫 실제 모서리 이탈 때 후축 거리 오차는 3.0 cm, heading 오차 -2.09°였고 약 2초 뒤 거리 오차는 0.619 m다. 차체 여유가 작은 경로는 몇 cm / 몇 도 차이에도 모서리가 벗어난다.

Planner 검사 공백은 [path_planner_node.cpp:602](/home/stier/HL-FMA2026-sim-stier/src/path_planner/src/path_planner_node.cpp:602)에서 확인된다. `queryOverlaps`로 만난 cell만 검사하므로 차체 일부가 도로 밖 빈 공간에 놓여도 남은 부분이 정상 cell과 겹치면 통과한다. 겹친 cell이 아예 없어도 반복문을 건너뛴다. 차체 전체가 주행 가능 영역에 포함되는지의 검사가 없다. :618–620은 처음 1초와 intersection의 다른 차선 접촉 판정도 완화한다.

추가 원인 후보는 ReferenceBuilder 최소간격 0.5 m([controller_core.hpp:28](/home/stier/HL-FMA2026-sim-stier/src/control/include/control/controller_core.hpp:28), [controller_core.cpp:196](/home/stier/HL-FMA2026-sim-stier/src/control/src/controller_core.cpp:196))와 MPC 모델의 v×0.05초 거리([296](/home/stier/HL-FMA2026-sim-stier/src/control/src/controller_core.cpp:296))가 일치하지 않는 점이다. 6 m/s에서 곡률은 0.5 m 간격인데 모델은 0.3 m 전진 단계에 대입한다. 조향 시점 영향은 추가 검증 후보이며, 이번 이탈의 유일 원인으로 확정하지 않았다.

## 북쪽 217 적색 정지 / 녹색 재출발

lanelet 446279, road 2819의 직진 stopline 544484다. 선 중심은 (1440.778381,977.241882), 진행 yaw -1.5275315 rad. 허용 녹색 코드는 3,5다.

- 첫 clean 적색 정지: (1440.396118,983.938171). 앞 범퍼 중심은 선 **2.899 m 전**, 최전방 모서리는 **2.884 m 전**이다. 선을 넘지 않았다.
- 두 번째 clean 적색 정지: t1789074605.780, (1440.239502,984.278503). 앞 범퍼 **3.245 m 전**, 최전방 모서리 **3.244 m 전**이다. raw state=1, cap=0, STOP_NO_LOCAL_PATH 상태였다.
- t4616.700: 신호 raw5 녹색. 이후 자력으로 ACTIVE 복귀해 출발했다.
- t4619.062: 앞 범퍼가 선을 통과했다. 속도 3.53 m/s, 신호는 raw5, cap=30이었다.

**이 북쪽 재현에서는 적색 정지와 녹색 재출발이 성공했다.** red 중 STOP_NO_LOCAL_PATH라는 상태가 나온 사실만으로 영구 교착이라고 판단하면 안 된다. Baseline ctrl136은 적색에서 선 7.29 m 전에 섰지만 반대 차선에 걸쳐 녹색 이후 78초 넘게 무경로 정지했으므로, 두 사건을 분리해야 한다.

계획 감속 11 m/s²와 controller 최대 제동 3 m/s² 차이, 학교구역 진입 후 뒤늦은 감속, respawn 속도 스파이크 등은 [앞선 제어 보고서](/tmp/sim_diagnosis_20260911/control_findings.md)에 정리되어 있다. 이번 북쪽 신호에서는 정지선 위반을 확인하지 않았다.

## 분석에서 제외하거나 구분한 것

- SIGSTOP/CONT 재사용 후 `FORWARD_STALE_EGO` +2 m/s²가 장시간 지속한 north/west sweep은 시험 조작과 연관된 DDS 수신 복구 문제로 분리했다. controller의 `/dev/shm/fastrtps_port7415 (deleted)` 매핑과 재생성 후 회복을 확인했다. 자세한 내용은 [input_freeze_evidence.md](/tmp/control_roadscan/input_freeze_evidence.md). 수신 중단 시작 원인을 일반 주행의 결함으로 집계하지 않았다. stale ego에서도 계속 가속하는 fallback 정책 자체는 source와 실제 명령으로 확인된다.
- Boost union 면적만 큰 일부 결과는 오검출이다. 예: 내리막 (558.143,-393.356)은 union 54.9% 밖이지만 native 격자는 0%; 북쪽 (1441.933,942.402)도 union 86.5%, native 0%. 주요 표는 독립 확인된 결과만 포함했다.
- 자연 적색 정지 후 (601.34,-119.50)는 전체 차도로는 안쪽이지만 반대 방향 차선에 걸친 사건이다. 도로 바깥 이탈과 구분한다.
- (1120.41,438.60) 최종 정지 위치도 도로 안이다. 최종 정지 지점만 검사하면 실제 중간 이탈을 놓친다.

## 산출물

- `/tmp/control_roadscan/clean_departure_evidence.png`, `.svg`: clean 두 사례의 실제 차체와 명령 시간축 비교 그림.
- `clean_sweep_combined.jsonl`, `clean_sweep_summary.json`: 첫 clean sweep 3,014 pose의 위치·차체·명령·상태·path age.
- `clean_west_north_combined.jsonl`, `clean_west_north_summary.json`: 후속 west/north 1,234 pose. 앞 파일 일부 시간과 중복되므로 단순 합산하지 않는다.
- `clean_native_grid.json`, `north_native_grid.json`: 주요 침범과 면적 오검출의 독립 격자 검증.
- `north217_stop.json`: stopline 실좌표 및 run 위치 기록. `red_stop_and_green_crossing` 필드에 신호별 최소 정지 여유와 녹색 통과를 따로 기록했다.
- `baseline_combined.jsonl`: 초기 주행 6,972 pose. `extended_combined.jsonl`: 이전 확장 주행 1,914 pose.
- `path_probe_combined.jsonl`, `timeline_probe_combined.jsonl`: 선택 path 및 모든 50 ms path의 미래 첫 12 m 차체 검사.
- `scan.py`, `containment.cpp`, `containment`: 기록 파일을 읽는 `/tmp` 전용 검사 도구.
