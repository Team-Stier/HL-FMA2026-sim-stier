# Data Pipeline — 원시 데이터부터 RViz까지

현재 정한 데이터 경로와 그 사이에 개입하는 연산만 표시한다. 사각형은 데이터·토픽·마커, 원은 연산, 원통은 파일·정적 맵이다. Bridge·TF·HDMap Dynamic Tracker 1차 버전·Visualizer·Python query marker adapter는 구현했다. Planner·Annotator·Control과 Tracker의 전체 신호 접근 상태기는 아직 구현할 계약이다. 실행과 현재 표시 제약은 README Bringup 및 [Tracker README](../src/hdmap_dynamic_tracker/README.md)를 따른다.

RViz Fixed Frame은 `map`이다. 메시지의 좌표 frame과 RViz Fixed Frame은 다르다. Planner는 `map`에서 경로·탐색 트리를 계산한 뒤 발행 직전에 입력 snapshot 시각의 TF로 `base_link`에 변환하고 RViz가 해당 시각의 TF로 `map`에 표시한다. 탐색 트리와 Ego 차체 박스도 `base_link` 기준이며 TF로 표시한다. 나머지 공간 관측과 지도 마커는 `map` 기준이다. `/objects`는 **XY 박스 중심 + 객체별 min Z**, Ego는 **후륜축 기준점**이다. 마커 출력은 `visualization_msgs/msg/MarkerArray`다.

## 1. VTD 패킷 → Bridge 토픽

```mermaid
flowchart TD
    VTD["VTD 내부 상태"] --> PLUGIN(("대회 플러그인<br/>객체: Ego 제외·80m 이내·가까운 순 최대 30<br/>신호: road와 lane 부호 기준 controller 하나 선택"))
    PLUGIN --> PACKET["TCP 레코드 1109 bytes<br/>Ego·객체·선택 신호<br/>sim timestamp 없음"]
    PACKET --> BRIDGE(("Bridge<br/>바이트 누적·레코드 조립·필드 해석<br/>동일 레코드에 수신 ROS 시각 t_rx 부여"))
    BRIDGE --> EGO["/ego_pose<br/>EgoPose·후륜축 XYZ·heading/pitch/roll"]
    BRIDGE --> CONVERT(("Bridge<br/>객체 reference → XY 중심·min Z"))
    CONVERT --> OBJECTS["/objects<br/>Objects·ID·좌표·heading·speed·크기"]
    BRIDGE --> SIGNAL["/traffic_light<br/>TrafficLight·controller ID/state"]
```

세 토픽의 Header는 같은 레코드의 `t_rx`를 공유한다. 이는 시뮬 관측 시각이 아니라 Bridge 수신 시각이다. 주행 토픽의 QoS는 BEST_EFFORT / VOLATILE / KEEP_LAST(1)이다.

## 2. 객체 → footprint·box·자세 마커

```mermaid
flowchart TD
    PACKET["패킷 객체<br/>reference XYZ·heading·speed·L/W/H"] --> BRIDGE(("Bridge<br/>reference XY에 회전한 offset 적용<br/>객체 박스 꼭짓점의 min Z 계산"))
    OFFSET[("객체 모델·기준점 offset 매핑")] --> BRIDGE
    BRIDGE --> OBJECTS["/objects<br/>x/y: XY 중심<br/>z: 객체 박스 하단 min Z<br/>map·t_rx"]
    OBJECTS --> FOOTPRINT(("Visualizer<br/>중심 기준 L/2·W/2 모서리 생성<br/>heading 회전·z는 min Z"))
    FOOTPRINT --> FM["/visualization/objects<br/>objects/footprint: LINE_LIST"]
    OBJECTS --> BOX(("Visualizer<br/>직립 box 중심 = x, y, z + H/2<br/>크기 L/W/H·자세는 yaw"))
    BOX --> BM["/visualization/objects<br/>objects/upright_box: CUBE"]
    OBJECTS --> INFO(("Visualizer<br/>heading을 방향 화살표로 변환<br/>ID·스칼라 속력을 텍스트로 변환"))
    INFO --> IM["/visualization/objects<br/>objects/heading: ARROW<br/>objects/info: TEXT_VIEW_FACING"]
```

```text
center_x = ref_x + cos(heading) * offX - sin(heading) * offY
center_y = ref_y + sin(heading) * offX + cos(heading) * offY
z        = min(world_box_corner[i].z)
marker_center_z = z + size_z / 2
```

Bridge는 기준점을 바꾸고 Visualizer는 하단 Z를 마커 중심 Z로 바꾼다. 같은 offset을 두 번 적용하지 않는다. 객체 box는 pitch/roll 없는 직립 근사이며 heading은 속도 방향이 아니다. 전체 NPC offset 매핑은 아직 미완성이다. 보정 전 reference는 `/objects`에 없으므로 별도 원시 기준점 마커 경로를 그리지 않는다.

## 3. Ego → 기준점·차체·추정 속도 마커

```mermaid
flowchart TD
    PACKET["패킷 Ego XYZ·heading/pitch/roll"] --> BRIDGE(("Bridge<br/>기준점 변경 없이 필드 번역"))
    BRIDGE --> POSE["/ego_pose<br/>map·후륜축 기준점·t_rx"]
    POSE --> REFERENCE(("Visualizer<br/>기준점·heading 표시"))
    REFERENCE --> RM["/visualization/ego<br/>ego/reference: POINTS<br/>ego/heading: ARROW"]
    POSE --> BODY(("Visualizer<br/>base_link 기준 차체 중심 offset을 pose에 설정<br/>크기 = 차량 길이·폭·높이<br/>자세 = identity·source stamp 계승"))
    VEHICLE[("vehicle.yaml<br/>차체 치수·후륜축 대비 위치")] --> BODY
    BODY --> BM["/visualization/ego<br/>ego/body: CUBE·base_link"]
    BM --> RVIZBODY(("RViz<br/>마커 stamp의 TF로 base_link → map"))
    POSE --> BODYTF(("TF Broadcasting<br/>Ego XYZ·RPY로 변환 생성"))
    BODYTF --> TF["/tf·map → base_link"]
    TF --> RVIZBODY
    POSE --> TRACKER(("Tracker<br/>연속 XY 이동 거리 / Header 시간 차<br/>초기화·비정상 dt·과대 점프의 속도는 0 처리"))
    PREVIOUS["이전 pose·Header"] --> TRACKER
    CONFIG[("runtime ego_velocity 설정")] --> TRACKER
    TRACKER --> STATUS["/ego_status<br/>위치·자세 계승 + 미분 추정 speed<br/>source stamp 계승"]
    STATUS --> TEXT(("Visualizer<br/>위치·자세·추정 속력 텍스트"))
    TEXT --> SM["/visualization/ego_status<br/>ego/status: TEXT_VIEW_FACING"]
```

차체 CUBE의 pose에는 차량 제원의 후륜축 대비 중심 offset만 넣는다. Ego의 map 위치·RPY는 TF에서 한 번만 적용한다. `header.frame_id=base_link`, stamp는 `/ego_pose`에서 계승하고 `frame_locked=false`로 해당 관측 시각에 표시한다.

speed는 시뮬레이터가 직접 준 Ego 속도가 아니라 위치 차분의 결과다. `/ego_status`에는 0 처리 이유가 없으므로 그 이유를 표시하는 경로는 없다.

## 4. Ego → TF → RViz view

```mermaid
flowchart TD
    POSE["/ego_pose<br/>후륜축 map 위치·RPY·t_rx"] --> NODE(("TF Broadcasting<br/>RPY → quaternion<br/>map에서 본 base_link pose 구성"))
    NODE --> TF["/tf<br/>map → base_link<br/>source stamp 유지"]
    PATH["/local_path<br/>base_link·계획 기준 시각 t0"] --> DISPLAY(("RViz 기본 Path display<br/>t0의 TF로 base_link → map"))
    TF --> DISPLAY
    DISPLAY --> SCREEN["화면"]
    TF --> AXES(("RViz TF display<br/>base_link 축 표시"))
    TF --> CAMERA(("RViz IdleFollow 카메라<br/>마우스 조작 중 map 기준 자유 이동<br/>10초 유휴 후 Ego 위치 추종·줌 유지<br/>화면 위 +x·왼쪽 +y"))
    MARKERS["map 좌표의 마커"] --> FIXED(("RViz Fixed Frame = map<br/>마커 좌표에 추가 변환 없음"))
    AXES --> SCREEN["화면"]
    CAMERA --> SCREEN
    FIXED --> SCREEN
```

`odom → map` 변환이나 지도 정합은 없다. 카메라 추종은 view 변경이지 데이터 좌표 보정이 아니다. `/clock` 없이 `use_sim_time=false`를 사용한다.

## 5. 정적 지도 → Lanelet·Cell 마커

```mermaid
flowchart TD
    XODR[("대회 XODR")] --> CONVERT(("오프라인 변환<br/>Lanelet2 변환·projector/원점/축 적용"))
    ASSETS[("OSGB·텍스처·시나리오·신호 참조")] --> RULES(("오프라인 맵 보완<br/>정지선·선종류·학교구역·신호 매핑"))
    CONVERT --> BUILD(("Lanelet 경계에서 약 1m Cell 분할<br/>분할점 보간·Cell ID·previous 고정"))
    RULES --> BUILD
    BUILD --> BIN[("hdmap.bin<br/>Lanelet·Cell polygon·TrafficLight")]
    BIN --> PRODUCER(("각 생산자 hdmap_init<br/>독립 Lanelet map·Cell·R-tree·registry 구성"))
    PRODUCER --> COMPUTE["Tracker·Planner·Annotator의 계산 입력"]
    BIN --> VIS(("Visualizer hdmap_init<br/>자기 프로세스의 독립 맵 구성"))
    VIS --> LINES(("Visualizer<br/>경계·중심선·진행 방향 추출<br/>선종류를 실선/점선 스타일로 변환"))
    LINES --> LM["/visualization/map<br/>map/local_reference/*<br/>LINE_LIST·LINE_STRIP·ARROW"]
    VIS --> BOXES(("Visualizer<br/>Cell polygon의 BoundingBox3d 추출"))
    BOXES --> CM["/visualization/cells<br/>cells/local_reference/bounds: LINE_LIST"]
    VIS --> SIGNALS["신호·정지선 표시용 registry"]
```

파일 제작은 오프라인 작업이다. `.bin` 로드에서는 재투영·Cell 재분할·ID 재생성을 하지 않는다. R-tree는 Cell AABB로 bulk-load한다. Visualizer의 맵 마커는 다른 노드 메모리의 복사본이 아니라 **자기 맵을 그린 것**이다. 자동 생성 중심선과 점선 스타일도 원본 경계에서 파생한 표시다.

## 6. 신호 → controller 관측·물리 신호·정지선 마커

```mermaid
flowchart TD
    PACKET["패킷의 선택된 controller ID/state"] --> BRIDGE(("Bridge<br/>ID/state 필드 번역"))
    BRIDGE --> TL["/traffic_light<br/>공간 frame 없음·t_rx"]
    TL --> TEXT(("Visualizer<br/>관측된 controller ID/state 텍스트"))
    TEXT --> TM["/visualization/signals<br/>signals/observed_controller: TEXT_VIEW_FACING"]
    TL --> LOOKUP(("Visualizer<br/>controller ID로 자기 registry 조회<br/>trafficLights·stopLine 형상 추출"))
    MAP[("Visualizer 자신의 signalRegistry")] --> LOOKUP
    LOOKUP --> GEOMETRY(("Visualizer<br/>물리 형상·정지선·대응 관계의 선 생성"))
    GEOMETRY --> GM["/visualization/signals<br/>signals/local_reference/*<br/>LINE_LIST·TEXT_VIEW_FACING"]
```

API는 controller 관측 하나를 준다. 위치는 registry에서 붙인 것이며, controller에 연결된 모든 lamp의 실제 색을 각각 관측한 것은 아니다.

## 7. 객체 → 현재·미래 Cell 점유 → 점유 마커

```mermaid
flowchart TD
    OBJECTS["/objects<br/>XY 중심·min Z·heading·크기·속력"] --> BOX(("Tracker<br/>yaw footprint와 높이 구간 구성<br/>offset 재적용 없음"))
    BOX --> CURRENT(("Tracker CellTree<br/>AABB 후보·높이 범위·polygon 교차"))
    MAP[("Tracker 자신의 Cell map·R-tree")] --> CURRENT
    CURRENT --> NOW["현재 bin 0<br/>교차 Cell 점유 1"]
    OBJECTS --> PREDICT(("motion_predictor / EKF<br/>ID별 위치 이력·운동 모델·오차 모델"))
    HISTORY["과거 객체 관측·Header 시간 차"] --> PREDICT
    PREDICT --> SWEEP(("Tracker<br/>0.5초 구간별 swept footprint 구성"))
    SWEEP --> FUTURE(("Tracker CellTree<br/>각 시간 구간과 Cell 교차"))
    MAP --> FUTURE
    FUTURE --> BINS["미래 bin 1..12"]
    NOW --> ARRAY(("Tracker<br/>cell-major 13-bin 배열 작성"))
    BINS --> ARRAY
    ARRAY --> DS["/dynamic_status.occupancy_probability<br/>Ego 입력 source stamp"]
    DS --> DISPLAY(("Visualizer<br/>선택 bin 추출·자기 Cell geometry 조회<br/>현재에 가깝고 점유확률이 높을수록 진하게<br/>alpha = p × (13 - bin) / 13"))
    VMAP[("Visualizer 자신의 Cell map")] --> DISPLAY
    CONFIG[("visualization.occupancy_bin")] --> DISPLAY
    DISPLAY --> MARKERS["/visualization/occupancy<br/>occupancy/local_reference/bin_N<br/>LINE_LIST·TEXT_VIEW_FACING"]
```

bin 0은 현재, bin 1..12는 `((bin-1)*0.5, bin*0.5]`초다. Visualizer는 예측을 다시 계산하지 않는다. 점유값은 Tracker 출력이고 표시 기하는 Visualizer 맵에서 복원한다. 객체별 예측 footprint는 주행 토픽에 없으므로 별도 예측 궤적 마커를 만들지 않는다.

선택한 bin의 Cell 선 마커에 `color.a = p * (13 - bin) / 13`을 적용한다. 같은 bin에서는 확률이 높을수록, 같은 확률에서는 현재에 가까울수록 불투명하다. `p=0`은 투명하며 unknown 값 `-1`은 이 식에 넣지 않고 별도 회색으로 표시한다. 텍스트에는 원본 확률과 bin을 그대로 표시한다. alpha는 표시 변환일 뿐 점유확률 자체를 변경하지 않는다.

## 8. 신호·정적 제한 → Cell cap → speed limit 마커

```mermaid
flowchart TD
    SIGNAL["/traffic_light"] --> STATE(("Tracker<br/>controller·접근로·movement 상태기"))
    EGO["/ego_status에 담는 Ego 위치·속도"] --> STATE
    REG[("Tracker 자신의 signalRegistry·정지선")] --> STATE
    STATE --> RAMP(("Tracker<br/>정지선에서 previous 방향 상류 순회<br/>거리별 선형 speed cap 기록"))
    CONFIG[("차량 제원·제동거리·v_entry<br/>factor·감속도·지연·정지 여유")] --> RAMP
    RAMP --> MIN(("Tracker<br/>적용 제한의 최솟값"))
    STATIC[("Lanelet 정적 속도 제한<br/>학교구역 운용 cap 8 m/s")] --> MIN
    MIN --> DS["/dynamic_status.speed_cap_mps"]
    DS --> COLOR(("Visualizer<br/>자기 Cell geometry 조회·cap 색상 매핑"))
    VMAP[("Visualizer 자신의 Cell map")] --> COLOR
    COLOR --> CM["/visualization/cell_cap<br/>speed/local_reference/cell_cap<br/>LINE_LIST·TEXT_VIEW_FACING"]
    DS --> ANNO(("Speed Annotator<br/>Ego가 속한 Cell의 cap 선택"))
    EGO --> ANNO
    AMAP[("Annotator 자신의 Cell map")] --> ANNO
    ANNO --> LIMIT["/speed_limit<br/>Float32·m/s·Header 없음"]
    LIMIT --> TEXT(("Visualizer<br/>현재 제한속도 텍스트"))
    TEXT --> LM["/visualization/speed_limit<br/>speed/current_limit: TEXT_VIEW_FACING"]
```

Annotator는 Path를 읽지 않는다. cap은 적용 제한의 최솟값이며 원인별 기여는 메시지에 없다. `/speed_limit`에는 source stamp와 Cell ID가 없다.

## 9. checkpoint → 글로벌 경로 강조·로컬 Path 표시

```mermaid
flowchart TD
    CSV[("checkpoint CSV<br/>순서·map XY")] --> MATCH(("Planner<br/>남은 checkpoint와 Ego의 lanelet 매칭"))
    EGO["/ego_status<br/>map pose·snapshot 시각 t0"] --> MATCH
    PMAP[("Planner 자신의 Lanelet map·RoutingGraph·Cell R-tree<br/>map 좌표")] --> MATCH
    MATCH --> ROUTE(("Planner<br/>shortestPathVia"))
    ROUTE --> GLOBAL["/global_path<br/>lanelet ID 순서·Header 없음"]
    GLOBAL --> RESTORE(("Visualizer<br/>자기 맵에서 해당 lanelet의 centerline 조회<br/>각 중심선을 밝은 색·굵은 선으로 강조"))
    VMAP[("Visualizer 자신의 Lanelet map")] --> RESTORE
    RESTORE --> GM["/visualization/global_path<br/>global_path/local_reference<br/>lanelet별 독립 LINE_STRIP·map"]
    ROUTE --> GOAL(("Planner<br/>다음 lanelet에서 map 목표 pose 선택"))
    GOAL --> PLAN(("Planner Hybrid A*<br/>map에서 primitive·경로·탐색 트리 생성<br/>차선 비용·횡단 제한<br/>속도 cap·가감속 한계로 도착시간 적분"))
    EGO --> PLAN
    PMAP --> PLAN
    PLAN --> QUERY(("Planner<br/>map footprint의 AABB·높이 범위·polygon으로 CellTree 조회"))
    PMAP --> QUERY
    QUERY --> CHECK(("Planner<br/>Cell ID로 cap·시간 구간 점유 조회<br/>시공간 충돌 검사 결과를 탐색에 반영"))
    DS["/dynamic_status<br/>t0·Cell ID별 상태"] --> CHECK
    CHECK --> PLAN
    PLAN --> COMPLETE["완성된 경로·map pose 목록"]
    COMPLETE --> LOCAL(("Planner 발행 직전<br/>t0의 TF 역변환으로 map → base_link<br/>경로 위치·자세 변환"))
    TF --> LOCAL
    LOCAL --> PATH["/local_path<br/>nav_msgs/Path·base_link<br/>후륜축 pose 목록·Header stamp t0"]
    PATH --> RVIZ(("RViz 기본 Path display<br/>t0의 TF로 base_link → map<br/>Fixed Frame map"))
    POSE["/ego_pose<br/>map pose·t0"] --> TFNODE(("TF Broadcasting<br/>XYZ·RPY → map에서 본 base_link pose"))
    TFNODE --> TF["/tf<br/>map → base_link·t0"]
    TF --> RVIZ
    RVIZ --> SCREEN["로컬 경로 화면"]
```

글로벌 경로는 ID 리스트에 든 lanelet 각각의 중심선만 강조한다. lanelet 사이 연결선을 추가하거나 중심선을 평활화하지 않는다. geometry는 Visualizer 자신의 맵에서 가져온 것이며 Planner의 실제 메모리 기하를 대신 나타내지 않는다.

Planner는 **경로 생성·탐색·비용 계산·충돌 검사 모두 map에서 수행**한다. 목표·차선 기하·primitive footprint·Cell R-tree를 같은 map 좌표로 사용하며 탐색 중 로컬 변환은 하지 않는다. 완성된 경로와 발행할 탐색 트리만 마지막에 `t0`의 TF 역변환으로 `base_link`에 옮긴다. `t0`는 입력 snapshot 시각이며 계산 완료 시각이 아니다. TF는 전담 노드가 Ego XYZ·RPY로 발행한 `map → base_link`를 사용한다. Path는 위치와 자세 모두 변환하고 탐색 트리의 배열 순서·부모 인덱스·최종 노드 인덱스는 유지한다.

Path Header와 모든 PoseStamped Header는 `frame_id=base_link`, `stamp=t0`로 통일한다. 도착 예정 시각이나 발행 시각으로 stamp를 바꾸지 않는다. RViz 기본 Path display가 Path Header의 시각으로 TF를 적용하며 Visualizer의 별도 `/visualization/local_path` 마커는 만들지 않는다. Buffer Length=1, Offset=(0,0,0)으로 원본 pose 순서를 그대로 표시한다. TF가 없으면 최신 TF로 대체하지 않는다. 빈 Path는 경로 표시를 비운다. primitive 도착시간은 Path에 없으므로 표시하지 않는다. 탐색 과정은 별도의 `/search_tree`로 표시한다.

### 탐색 트리 → 부모 연결선·선택 분기

```mermaid
flowchart TD
    PLAN(("Planner<br/>map에서 탐색 노드·부모 관계 생성")) --> RESULT["발행할 탐색 트리·map"]
    RESULT --> LOCAL(("Planner 발행 직전<br/>t0의 TF 역변환으로 map → base_link<br/>변환된 위치·자세에서 x/y/yaw 추출<br/>부모·최종 노드 인덱스 유지"))
    TF --> LOCAL
    LOCAL --> TREE["/search_tree<br/>interfaces/msg/SearchTree<br/>base_link·snapshot stamp t0"]
    TREE --> VIS(("Visualizer<br/>x/y를 z=0 점으로 사용<br/>parent_index로 부모-자식 연결<br/>yaw로 방향 화살표 생성<br/>final_node_index에서 부모를 따라 선택 분기 강조"))
    VIS --> MARKERS["/visualization/search_tree<br/>LINE_LIST·ARROW<br/>base_link·t0·frame_locked false"]
    MARKERS --> RVIZ(("RViz<br/>t0의 TF로 base_link → map<br/>Fixed Frame map"))
    POSE["/ego_pose·t0"] --> NODE(("TF Broadcasting<br/>Ego XYZ·RPY로 변환 생성"))
    NODE --> TF["/tf<br/>map → base_link·t0"]
    TF --> RVIZ
    RVIZ --> SCREEN["탐색 트리 화면"]
```

`x/y/yaw/parent_index`는 길이가 같은 배열이며 인덱스 하나가 탐색 노드 하나다. 좌표는 m, yaw는 rad다. 부모 없는 루트의 `parent_index=-1`, 나머지는 같은 메시지의 부모 노드 인덱스다. `final_node_index=-1`은 선택된 최종 노드 없음이고, 그 외에는 같은 배열의 인덱스다. 부모 연결은 순환하지 않는다. Header는 해당 탐색의 `/local_path`와 같은 `base_link`, `t0`를 사용한다.

연결선은 샘플 노드 사이의 직선이지 실제 primitive 곡선이 아니다. SearchTree는 변환 후 x/y/yaw만 저장하므로 z·pitch·roll은 전달하지 않는다. 표시의 z=0은 이 메시지의 2D 근사이며 경사면에서는 원래 map 탐색 기하와 일치하지 않을 수 있다. 지면에 snap해 이를 숨기지 않는다. 원본 노드와 부모 관계를 그대로 표시하고 새로운 연결을 만들지 않는다. 새 메시지로 이전 트리를 교체하며 사라진 마커는 DELETE한다. 빈 배열과 `final_node_index=-1`이면 트리 표시를 비운다. TF는 최신 값이 아니라 t0의 값을 사용한다.

## 10. Control → 명령 마커·VTD 입력

```mermaid
flowchart TD
    EGO["/ego_status"] --> MPC(("Control MPC<br/>차량 모델·목표 경로로 제어 계산"))
    PATH["/local_path<br/>base_link·t0"] --> MPC
    MPC --> LIMITER(("Control<br/>speed cap·과속 제동 override·HOLD 적용"))
    LIMIT["/speed_limit"] --> LIMITER
    CONFIG[("제어·차량·제동 config")] --> MPC
    CONFIG --> LIMITER
    LIMITER --> CMD["/ctrl_cmd<br/>steering·target_accel·turn_signal<br/>명령 생성 시각"]
    CMD --> TEXT(("Visualizer<br/>최종 요청값 텍스트"))
    TEXT --> MARKERS["/visualization/control<br/>control/requested: TEXT_VIEW_FACING"]
    CMD --> BRIDGE(("Bridge<br/>little-endian ffB 9 bytes로 직렬화·송신"))
    BRIDGE --> VTD(("VTD dynamics<br/>명령 적용·차량 상태 갱신"))
    VTD --> FEEDBACK["다음 Ego 패킷<br/>1절과 3절의 관측 경로"]
```

Control은 `/ego_status`, `/local_path`, `/speed_limit`만 구독한다. 발행된 로컬 경로를 입력으로 사용하며 TF 구독·변환은 하지 않는다.

마커는 요청한 바퀴각·가속도이지 실제 차량 응답이 아니다. MPC 원안·override 이유·송신 완료 여부는 `/ctrl_cmd`에 없다.

## 11. CellTree 조회 → 생산자 Cell 마커

```mermaid
flowchart TD
    BOX["호출자의 query bounding box·map"] --> SEARCH(("CellTree.search<br/>AABB 교차 후보 조회"))
    FOOTPRINT["호출자의 footprint·높이 범위·map"] --> OVERLAPS(("CellTree.queryOverlaps<br/>입력 polygon 정규화<br/>AABB 후보·높이 범위·XY polygon 교차"))
    MAP[("생산자 자신의 Cell map·R-tree")] --> SEARCH
    MAP --> CACHE(("CellTree 초기화<br/>Cell polygon XY 복사·winding/closure 정규화"))
    CACHE --> OVERLAPS
    MAP --> OVERLAPS
    SEARCH --> HITS["실제 hit Cell 목록"]
    OVERLAPS --> HITS
    HITS --> SINK(("DebugSink<br/>operation·Cell 포인터 전달"))
    SINK --> COPY(("생산자 marker adapter<br/>그 Cell의 polygon·AABB를 값으로 복사"))
    COPY --> AGG(("기본 집계<br/>0.05초 window 안의 동일 생산자·Cell·연산·기하 병합"))
    AGG --> MARK(("생산자 marker adapter<br/>hit AABB·polygon 선 생성<br/>집계 window·횟수 표시"))
    MARK --> TOPIC["/debug/node/cell_queries<br/>LINE_LIST·LINE_STRIP·TEXT_VIEW_FACING"]
    TOPIC --> RVIZ["RViz"]
    SINK --> RESULT["callback 완료 후 호출자에게 Cell ID 목록 반환"]
```

마커는 Visualizer가 ID로 재조회한 기하가 아니라 **조회한 프로세스의 실제 Cell**을 사용한다. 기본 집계는 호출 순서·개별 시각을 합치므로 raw trace와 다르다. 현재 sink에 query 입력·source stamp는 없으며, 반환 Cell AABB를 query box라고 표시하지 않는다. Python adapter는 `visualization.query_debug.QueryDebugSink`로 구현했다. C++ ROS adapter는 미구현이다.

## 12. 공통 마커 처리 → 화면

이 처리는 MarkerArray 경로에만 적용한다. `/local_path`는 9절의 RViz 기본 Path display로 직접 표시한다.

```mermaid
flowchart TD
    DATA["각 경로의 기하·값"] --> ROI(("Visualizer<br/>데이터 frame에서 Ego 중심 반경 100m ROI 적용"))
    EGO["/ego_status 위치"] --> ROI
    CONFIG[("visualization 설정<br/>반경·색상·선두께·점 크기")] --> ROI
    ROI --> STYLE(("Visualizer<br/>색상·alpha·선두께·텍스트 적용"))
    CONFIG --> STYLE
    EGO --> ANCHOR(("Visualizer<br/>Ego 상태·제한속도·명령 텍스트의 배치 위치"))
    ANCHOR --> STYLE
    STYLE --> MARKERS["각 /visualization/*<br/>원본 reference frame·source stamp 계승<br/>lifetime 0·frame_locked false"]
    MARKERS --> UPDATE(("RViz<br/>ns/id로 ADD·MODIFY·DELETE 적용"))
    UPDATE --> TRANSFORM(("RViz<br/>base_link 마커는 stamp의 TF로 map 변환<br/>map 마커는 추가 변환 없음"))
    TF["/tf·map → base_link"] --> TRANSFORM
    TRANSFORM --> VIEW(("RViz<br/>Fixed Frame map·카메라 view·렌더링"))
    VIEW --> SCREEN["화면"]
```

ROI 거리 비교도 같은 frame에서 수행한다. base_link 데이터는 해당 시각의 원점 기준이며 map의 Ego 좌표와 직접 빼지 않는다. map 좌표의 points에는 identity pose를 사용한다. CUBE처럼 pose에 중심·회전을 담는 마커는 로컬 형상을 사용한다. 같은 변환을 pose와 points에 중복 적용하지 않는다. 텍스트 배치 위치는 표시용이지 값의 관측 위치가 아니다.

Header 없는 `/global_path`·`/speed_limit`의 표시 시각은 source 시각이 아니다. `lifetime=0`이므로 목록에서 빠진 객체·경로 마커는 발행자가 DELETE로 제거한다. 좌표를 띄우거나 box를 부풀려 겹침을 숨기지 않는다.

## 13. Frame 전수 점검

RViz Fixed Frame은 항상 `map`이며 아래 frame은 **데이터 좌표의 기준**이다.

| 데이터 | reference frame | 화면까지 개입하는 변환 |
|---|---|---|
| `/ego_pose`, `/ego_status` | `map` | 후륜축 위치 그대로 사용; 차체 CUBE는 아래 별도 frame 계약 적용 |
| `/objects` | `map` | Bridge에서 reference → XY 중심·min Z; box 마커는 z에 H/2 추가. 좌표 frame 변경은 아님 |
| Lanelet·Cell·R-tree | `map` | 파일 제작 때 투영; 로드·표시 때 재투영 없음 |
| `/dynamic_status` | `map`의 Cell ID 참조 | Visualizer 자신의 Cell geometry로 위치 복원 |
| `/global_path` | `map`의 lanelet ID 참조; Header 없음 | Visualizer 자신의 lanelet 중심선을 강조 |
| `/local_path` | `base_link(t0)` | RViz가 t0의 TF로 map에 변환; Visualizer 미경유 |
| `/traffic_light` | 없음 | 관측은 ID/state; 물리 위치는 map registry에서 조회 |
| `/speed_limit` | 없음 | 스칼라 값을 map의 표시용 텍스트 위치에 배치 |
| `/ctrl_cmd` | 차량 기준 `base_link` | 제어 스칼라이지 변환할 공간 점이 아님; map 위치에 텍스트 배치 |
| `/debug/node/cell_queries` | `map` | 생산자가 실제 조회한 Cell 기하 그대로 표시 |
| `/visualization/ego`의 `ego/body` | `base_link` | 중심 offset·치수만 적용한 CUBE를 source stamp의 TF로 map에 표시 |
| `/search_tree`, `/visualization/search_tree` | `base_link(t0)` | 노드·부모 연결·yaw 표시 후 t0의 TF로 map에 표시 |
| 나머지 `/visualization/*` 마커 | `map` | 추가 TF 변환 없음; 색상·선두께·ROI는 표시 단계 |
| `/tf` | parent `map`, child `base_link` | Ego pose에서 생성; Path에는 원본 stamp의 변환 적용 |

자료형과 메시지 계약: [README](../README.md), [객체 기준점 조사](05-object-reference-resolution.md), [ROS Marker](https://raw.githubusercontent.com/ros2/common_interfaces/jazzy/visualization_msgs/msg/Marker.msg), [ROS Path](https://raw.githubusercontent.com/ros2/common_interfaces/jazzy/nav_msgs/msg/Path.msg).

## 화면 고정 텍스트 출력

기존 절의 TEXT_VIEW_FACING 생성 경로는 이제 위치 없는 HUD 텍스트로 출력한다. Visualizer가 생성한 텍스트 전체가 대상이며 생산자의 query debug 마커는 별도다. 3D 기하 마커는 기존 frame·stamp 계약을 유지한다.

```mermaid
flowchart TD
    SOURCE["수신 토픽의 값·source stamp<br/>Visualizer 자신의 map에서 얻은 ID·선종류"] --> FORMAT(("Visualizer<br/>표시 문자열 작성·토픽별 최신 항목 교체<br/>namespace/id 보존·좌표와 크기는 사용하지 않음"))
    FORMAT --> HUD["/visualization/hud<br/>std_msgs/String·5Hz<br/>BEST_EFFORT·VOLATILE·KEEP_LAST 1"]
    HUD --> DISPLAY(("RViz HUD Display<br/>viewport 왼쪽 위 16px·고정 글자 크기<br/>스크롤 가능·TF/카메라 변환 없음"))
    DISPLAY --> SCREEN["화면 고정 텍스트 패널"]
```
