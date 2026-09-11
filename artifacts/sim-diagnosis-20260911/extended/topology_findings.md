# 지도 분할, current lane 선택, 경로 길이 진단

2026-09-11. 저장소 소스·설정·원본 체크포인트 수정 없음. 네이티브 `map/hdmap.bin`과 원본 planner를 읽어 `/tmp`에서만 재현했다. 지도 길이는 2D centerline 누적 거리이며, `minimum horizon` 등 런타임 파라미터와 동일한 단위(m)다.

## 1. 짧은 경로의 근본 원인: 지도 조각 수를 전방 거리로 사용

`ReferenceBuilder::build`는 현재 lanelet에 **following lanelet 하나만** 붙인다. 옆 차로 reference는 해당 lanelet 하나로 끝난다. 따라서 `max_path_length_m=50`은 확보할 전방 길이가 아니라 상한으로만 작용한다. 생성기는 reference 끝에 station을 clamp하고 즉시 종료한다. 연결 도로가 수백 m 남아 있어도 두 번째 lanelet 끝이 움직이지 않는 로컬 패스 끝점이 된다.

근거: [ReferenceBuilder](/home/stier/HL-FMA2026-sim-stier/src/path_planner/src/path_planner_node.cpp:393), [projection clamp](/home/stier/HL-FMA2026-sim-stier/src/path_planner/src/path_planner_node.cpp:340), [station clamp/end break](/home/stier/HL-FMA2026-sim-stier/src/path_planner/src/path_planner_node.cpp:814).

이 지도에 매우 짧은 lanelet이 생기는 이유도 확인됐다. [build_map.py:193](/home/stier/HL-FMA2026-sim-stier/map/tools/build_map.py:193)는 lane section 안 **모든 차로의 roadMark 전환 위치를 합집합**한 뒤 모든 driving lane을 동일한 위치에서 자른다. 반대편 차선 표시의 전환점이 10cm 다르면 현재 주행 차로도 10cm 조각으로 나뉜다. 미세 조각 자체는 표시 속성을 보존하는 지도 표현이며 연결 관계는 정상이다. planner가 lanelet 두 개를 충분한 전방 거리로 취급하는 가정과 맞지 않는다.

원본 XODR: `/home/stier/VIRES/VTD.2025.2/Runtime/Tools/ROD/DefaultProject/Odr/HL_FMA_VTD_LivingLab.xodr`.

| 도로와 실제 lanelet | 분할 원인 | XODR 근거 |
|---|---|---|
| road173, 27227, 길이 0.102650m | 중심선 lane0의 138.5m 전환과 반대 lane+1의 138.6m 전환 | [138.5](/home/stier/VIRES/VTD.2025.2/Runtime/Tools/ROD/DefaultProject/Odr/HL_FMA_VTD_LivingLab.xodr:4928), [138.6](/home/stier/VIRES/VTD.2025.2/Runtime/Tools/ROD/DefaultProject/Odr/HL_FMA_VTD_LivingLab.xodr:4908) |
| road173, 28078, 길이 0.101023m | lane0 244.0m와 lane+1 244.1m 전환 | [244.0](/home/stier/VIRES/VTD.2025.2/Runtime/Tools/ROD/DefaultProject/Odr/HL_FMA_VTD_LivingLab.xodr:4932), [244.1](/home/stier/VIRES/VTD.2025.2/Runtime/Tools/ROD/DefaultProject/Odr/HL_FMA_VTD_LivingLab.xodr:4912) |
| road2804, 401650, 길이 0.500000m | lane0/-1/-2의 34.5m와 lane+1 35.0m 전환 | [34.5](/home/stier/VIRES/VTD.2025.2/Runtime/Tools/ROD/DefaultProject/Odr/HL_FMA_VTD_LivingLab.xodr:76321), [35.0](/home/stier/VIRES/VTD.2025.2/Runtime/Tools/ROD/DefaultProject/Odr/HL_FMA_VTD_LivingLab.xodr:76309) |
| road2804, 401618, 길이 1.000000m | lane-2 26.5m와 lane0/+1/-1 27.5m 전환 | [26.5](/home/stier/VIRES/VTD.2025.2/Runtime/Tools/ROD/DefaultProject/Odr/HL_FMA_VTD_LivingLab.xodr:76345), [27.5](/home/stier/VIRES/VTD.2025.2/Runtime/Tools/ROD/DefaultProject/Odr/HL_FMA_VTD_LivingLab.xodr:76319) |
| road1222, 96205, 길이 10.501465m | 60.5–71.0m의 표시 전환 구간 | [60.5/71.0](/home/stier/VIRES/VTD.2025.2/Runtime/Tools/ROD/DefaultProject/Odr/HL_FMA_VTD_LivingLab.xodr:17481) |

위험 지점의 후속 연결은 존재하고 조사한 centerline 접합부 간격은 모두 0m다.

| lanelet 체인 (괄호 안 길이 m) | 현재 코드에서 나타나는 한계 |
|---|---|
| 401819(41.818) → 401650(0.500) → 401647(7.000) → 401618(1.000) → 401613(26.500) → 460898(24.577) → 407353(26.500) | 첫 reference는 42.318m 전체; 401819 중간에서 시작하면 약21.4m. 뒤의 `401650+401647=7.5m`, `401647+401618=8.0m`도 매우 짧음 |
| 27224(12.785) → 27227(0.103) → 28004(98.344) | 98m 곡선이 연결돼 있지만 끝부분에서 reference는 0.1m 조각까지만 포함 |
| 28004(98.344) → 28075(8.594) → 28078(0.101) → 29001(115.589) | 28075 진입 후 reference 전체가8.695m; 긴 다음 구간은 아직 포함되지 않음 |
| 96120(60.501) → 96205(10.501) → 96736(66.025) → 98402(46.539) | 96120 끝에서는 약10.5m만 남지만 96205가 current가 되면 전체76.526m 확보 |
| 34407(53.258) → 69067(47.941) | 올바른 reference 전체101.200m. 이 지점의 짧은 패스는 지도 조각 부족만으로 설명하면 틀림 |

전체 2,845 lanelet(일반1,941, 교차로904)의 길이 중앙값24.746m, 10분위5.100m, 25분위10.095m, 최소0.092m. 1m 미만44개, 2m 미만100개, 차량 길이4.848m 미만271개, 10m 미만692개다. 후속 lanelet이 있는2,738개 중 **89개는 모든 후속이2m 미만**, 773개는 모든 후속이12m 미만이다. 가장 긴 후속을 선택해도 `현재+후속`이16m 미만인 경우254개, 50m 미만인 경우1,240개다. 이 수치는 특정 주행 경로의 실패율이 아니라 구조적 전방 거리 부족 가능성이다.

## 2. 미세 lanelet은 current 선택에서 건너뛰어지고 reference 시작점도 뛴다

`IntersectionMonitor`는 후륜축 기준 앞3.808m/뒤1.040m/폭1.886m footprint와 겹치는 면적이 가장 큰 lanelet 하나를 current로 선택한다. 진행 방향, 이전 current, global route 순서, 높이 조건이 없다. [원본 선택식](/home/stier/HL-FMA2026-sim-stier/src/path_planner/src/path_planner_node.cpp:149).

원본 C++ 함수를 그대로 호출하고 upstream 끝 접선 위에서 후륜축 위치를0.1m씩 이동시킨 결과:

| 경계 | 실제 current 전환 | 의미 |
|---|---|---|
| 401819→401650→401647 | 401819 끝1.1m 전에서 401647로 전환 | 0.5m lanelet401650 건너뜀 |
| 27224→27227→28004 | 27224 끝1.3m 전에서 28004로 전환 | 0.1m lanelet27227 건너뜀 |
| 28075→28078→29001 | 28075 끝1.3m 전에서 29001로 전환 | 0.1m lanelet28078 건너뜀 |
| 96120→96205 | 96120 끝1.3m 전에서 96205로 전환 | 차량 앞부분이 다음 lanelet에 걸쳐 조기 전환 |

전환 직전에는 기존 reference가 약1.4–1.6m만 남는다. 전환 직후에는 새 reference 시작이 후륜축보다 약1.4–1.6m 앞에 있다. `project()`가 선분 station을0으로 clamp하고 횡방향 d만 저장하므로 뒤쪽 종방향 차이가 보존되지 않는다. 그 결과 로컬 첫점이 차량 위치와 일치하지 않고 앞으로 이동할 수 있다. 이는 map 기반 원본 함수 재현이며 실제 비동기 tick 시점의 숫자는 달라질 수 있다.

`RouteUpdater::updateGoals`는 `previous_next == new_current`일 때만 정상 전진으로 본다. 미세 lanelet을 건너뛰면 `route_changed=true`가 되어 정렬 상태가 리셋되고 목표가 `{route[1]}`에서 `{route[0], following}` 형태로 바뀔 수 있다. [소스254](/home/stier/HL-FMA2026-sim-stier/src/path_planner/src/path_planner_node.cpp:254), [소스272](/home/stier/HL-FMA2026-sim-stier/src/path_planner/src/path_planner_node.cpp:272), [소스283](/home/stier/HL-FMA2026-sim-stier/src/path_planner/src/path_planner_node.cpp:283). 따라서 goal과 비용도 current의 불연속에 반응한다.

## 3. 교차로 stale route와 방향 없는 지도 매칭은 별개 문제

교차로에 있고 이전 global path가 비어 있지 않으면 current만 갱신하고 이전 route와 goal을 그대로 유지한다. [소스211](/home/stier/HL-FMA2026-sim-stier/src/path_planner/src/path_planner_node.cpp:211). 이전 세션의 실제 교차로 respawn에서 새 위치 적용 후 global path가6.426초 유지된 증거와 원본 함수100tick 재현은 [respawn_findings.md](/tmp/sim_diagnosis_20260911/respawn_findings.md)에 있다.

현재 지도 **교차로904개 중 longitudinal following이2개 이상인 lanelet은0개**다. 따라서 이 지도에서 '교차로 frozen route 때문에 following.front가 임의 분기를 고른다'는 설명은 입증되지 않는다. 분기가 있는 일반 접근로34407의 후속 순서는 `[531841,69067]`, 401613은 `[519877,460898]`; 정상 global route가 current를 포함하면 원하는69067/460898을 찾는다. current가 route에 없거나 stale인 때는 fallback을 주의해야 하지만, 이 분기 사례를 정상 교차로의 공통 원인으로 쓰면 안 된다.

반면 current가 **교차하는 다른 진행 방향의 lanelet**으로 바뀌는 현상은 실제 기록으로 확인됐다. 서쪽 내리막 주행 t1789073902.619273의 후륜축 `(565.136475,-386.987549,z41.707340, heading0.740064rad, speed7.256465m/s)`에서 원본 monitor는524969를 고른다.

- 원래 경로: `115710 → 116429 → 116670 → 116753 ...`, 목표는 frozen 상태로 `[115710,116429]`.
- 선택된524969는 XODR road3346의 다른 횡단 방향. reference `[524969,74451]`에 **79.912m**가 남지만 차량과 reference 방향이 **92.655도** 어긋나 후보20개 모두 운동학 조건을 통과하지 못함(원본 코드 오프라인 후보 재현).
- 524969와 footprint 겹침 약5.348m², 예정 좌회전116670과 약0.878m². 후륜축은524969 centerline에서0.220m,116670 centerline에서2.005m 떨어짐. 예정 경로 방향과도 약27.37도 어긋나 있어 이 시점에 이미 좌회전 바깥으로 이탈한 상태다.
- 두 도로 표고는41.7m로 같고 ego 차이는 약7mm다. 고가/지하 도로의 높이 혼동이 아니다. 앞선 이탈 뒤에 면적 우선/방향 무시 매칭이 횡단 차로를 선택하면서 회복을 막는 사슬이다. 첫 이탈의 원인은 이전 후보 경로와 제어 기록을 함께 봐야 한다.

34407에서도 같은 취약점을 독립 확인했다. 중앙점의 정방향 heading0.429955를 유지한 채 왼쪽으로 이동하면1.50m까지34407,1.75m부터 반대 방향34191(yaw-2.71058)을 고른다. 이는 실제 횡방향 이탈과 지도 매칭의 증폭 작용을 구분해서 읽어야 한다.

## 4. 다양한 시험 시작점과 예상 경로

`start_poses.json`은 map centerline 상의 xyz와 해당 선분의 정방향 yaw로 생성했다. **다섯 점 모두 원본 IntersectionMonitor expected==actual, intersection=false** 확인. 목표점까지 연속 longitudinal chain과 shortest path가 존재한다. z는 지도 표면 높이이며, 첫 관측 ego와 footprint cells로 실제 배치도 확인해야 한다. 비탈길 시작 시 p/r=0 명령 자체가 장기 경로 오류의 근거는 아니다.

| 시험 | 시작 lane 및 pose (x,y,z,yaw rad) | 목표 lane | 연결 경로 거리 |
|---|---|---|---|
| 북향 곡선 road173 | 28004, (797.245,-86.639,49.526,0.339030) | 29001 | 210.040m |
| 남쪽 큰 곡선 road1909 | 148779, (1002.360,-1181.549,61.376,-0.293704) | 156955 | 287.926m |
| 서쪽 급내리막 road1602 | 110407, (486.437,-449.733,44.572,0.666965) | 22849 | 269.259m |
| 북쪽 오르막 road2819 | 445027, (1435.540,1097.398,47.783,-1.529396) | 448217 | 277.331m |
| 서쪽 동향 교차로 진입 road76 | 8862, (-43.672,34.999,35.550,0.055746) | 7355 | 258.200m |

이 거리들은 지도상 정상 주행 가능한 거리다. 실제 로컬 패스는 속도·시간 horizon·장애물·신호 등에 따라 달라져야 하며 고정50m가 정답은 아니다. 다만 앞에 이어지는 도로가 정상이고 정지 요인이 없는데 약1m로 끝나는 현상을 목적지/도로 단절로 설명할 수는 없다.

단일 최종 checkpoint lane에 들어가면 shortest path가1개가 되고 goals가 빈 배열이 되어 planner가 publish를 멈춘다([235](/home/stier/HL-FMA2026-sim-stier/src/path_planner/src/path_planner_node.cpp:235), [755](/home/stier/HL-FMA2026-sim-stier/src/path_planner/src/path_planner_node.cpp:755)). 그러므로 임시 한 점 checkpoint 시험에서 목적지 lane 진입 후 `STOP_NO_LOCAL_PATH`는 명시적 도착 정지와 구분해야 한다. 최종 checkpoint의 xy는 도착 속도 계획에 전달되지 않는다.

## 증거 파일과 재현 방법

- 전체 census/hotspot/속성: `map_topology.json`, `map_topology_output.txt`, `topology_extra.json`.
- 표시 분할 원문 위치: `xodr_mark_evidence.json`.
- 원본 monitor sweep: `monitor_probe.cpp`, `monitor_probe_output.txt`, `opposing_lane_sweep_output.txt`.
- 교차 방향 면적/높이: `overlap_probe.cpp`, `downhill_overlap.txt`.
- 시작점과 전체 경로: `start_poses.json`; 실행용 최소 schema: `drive_cases.json`.
- 원본 후보 재현: `/tmp/planner_census/west_downhill.results.jsonl` (local_path agent 소유).

모든 C++ probe는 원본 `path_planner_node.cpp`를 include하고 main 이름만 바꾸며 ROS node/시뮬레이터 제어를 만들지 않는다. 지도 geometry만 읽는다. `source install/setup.bash` 후 `/tmp/sim_diagnosis_extended/monitor_probe < /tmp/sim_diagnosis_extended/start_pose_check.txt` 또는 `printf '565.136474609375 -386.987548828125 41.7073402404785 0.740063607692719\n' | /tmp/sim_diagnosis_extended/overlap_probe`로 재현 가능하다.
