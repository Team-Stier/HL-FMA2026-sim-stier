# Visualizer 파이프라인 전수 대조 — 2026-09-05

## 새 컨텍스트 재검토

이번에는 대화 이력을 전달하지 않은 독립 서브에이전트 **2개**가 현재 로컬 checkout을 다시 검토했다. Python 데이터 경로와 C++ RViz·TF·launch 경로를 분리했으며 이전 감사 보고서를 읽지 않도록 했다. 검토 후 수정사항을 재검토하고 아래 실측을 별도로 수행했다. 앞선 보고서의 ‘보완’ 판정을 그대로 신뢰하지 않았다.

| 발견 사항 | 정정 / 확인 |
|---|---|
| 글로벌 경로 HUD가 `emit()`에서 덮여 없어짐 | 실제 known/missing ID 입력으로 재현. 기하 발행 후 요청 목록·MISSING 라벨 보존. 빈 경로·맵 미로딩도 검사 |
| 입력을 거부해도 예전 객체·경로·점유 그림이 남음 | 거부한 스트림만 비움. 다른 토픽의 그림·HUD를 지우지 않음 |
| 모든 과거 marker ID를 영구 보관·재삭제 | 전용 Display에 `DELETEALL + 현재 ADD` snapshot. 중간 유실 뒤에도 다음 snapshot으로 복구. 공유 토픽 사용 금지 |
| 서로 다른 확률 수만큼 RViz marker 생성 | 정점별 원본 RGBA + native blending 경계로 최대 2개 배치. 5,000개 서로 다른 확률로 확인. 확률 양자화·선분 삭제 없음 |
| 수신 나이만 변해도 19만 줄 HUD 전체 재배치 | Qt 문서의 변경 구간만 교체. 전체 문자열·스크롤·선택 유지, undo 이력 비활성화 |
| 항공뷰 교체 때 native embedded texture 미해제 | 소유한 텍스처를 renderer 소멸 시 해제하도록 수정 |
| 빈 TRIANGLE_LIST의 null material 접근 | native 오류 표시 유지, null material 접근 방지. 잘못된 marker action/type과 텍스처 예외 처리 |
| RViz reset 후 1회 발행 항공뷰·정적 지도 소실 | 항공뷰와 map/cells 전용 `StaticMarkerArray`가 retained sample 재수신. native 기하 렌더러 유지, PNG·정점의 주기적 재발행 없음 |
| 다른 view에서 복사된 base_link target이 수동 이동에 개입 | view 활성화 시 fixed frame 고정. Scale 최솟값은 마우스 처리 대신 property 자체에 설정 |
| 보조 bringup에서 SimBridge 중복 실행 | aggregate visualization launch 한 번만 실행. 실행 명령 목록 검사, 실제 프로세스는 검사에서 시작하지 않음 |

단일 root SearchTree의 빈 LINE_LIST는 **결함으로 채택하지 않았다.** RViz는 빈 선 갱신으로 이전 선을 지운다. 임의 보간·가짜 선분을 추가하지 않았다.

### 재현과 실측

- 실제 지도 2,847 Lanelet, 94,157 Cell, 722 정지선, 646 물리 신호. Cell polygon/AABB의 batching 전후 **전체 선분 수 동일**, 좌표·frame 검사 통과.
- 격리 ROS domain에서 Ego 입력 없이 Objects/global/traffic_light/speed_limit/dynamic 동작 확인. 객체 min Z→중심 Z에는 H/2만 더하고 관측 stamp=321을 유지.
- 점유 bin 0/1/12, unknown/zero, opacity 경계 양옆과 배열 역순 검사. 정점별 값·alpha·XYZ 유지 및 배치 수 확인.
- `/tmp/review_visualizer_runtime.py`: 실제 맵 기반 Python 회귀·독립 입력·HUD·삭제·점유 검사. `/tmp/check_visualizer.py`: 전체 정적 기하 검사.
- `/tmp/check_hud_cost.cpp`의 19만 줄 전체 교체는 **333–398ms**. `/tmp/check_hud_incremental.cpp`의 약 6.75MB 문서에서 시각 부분만 변경하면 **18–22ms**. Unicode·개행·삽입/삭제·스크롤·선택·undo 이력 검사 통과. 전체 Cell 값 자체가 매번 바뀌는 경우의 비용이라고 오해하면 안 된다.
- 전체 동적 배열 처리의 Python warm callback은 **약 0.45초**, 첫 Cell 기하 캐시 생성 포함 약 3.15초. **10/20Hz 전체 지도 표시를 달성한 것은 아니다.** HUD 전체 6.25MB를 5Hz로 보내는 기존 계약도 유지되어 전송량은 여전히 크다. 이 수치는 DDS 직렬화·RViz 렌더 시간을 제외한다.
- 항공뷰 world bounds·UV·해상도 검사와 학교 crop 비교 상관계수 **0.9927637704**. 원본 이미지 밝기·지도 좌표 변경 없음.
- 격리된 native RViz/Ogre 검사에서 reset·disable·잘못된 입력 후 texture resource 수가 기준값으로 복귀함을 확인. 원래 1회 발행한 항공뷰·정적 MarkerArray가 reset/re-enable 뒤 재수신되며 추가 발행은 없었음. 빈 기하·marker action/type·텍스처 오류, view mimic 뒤 fixed-frame 복원, Scale 0/음수 clamp도 통과. 로그: `/tmp/visualization_native_check/runtime_static_fixed.log`.
- 최종 colcon visualization 빌드 통과. 로컬 checkout의 설치본으로 실제 RViz를 재시작하고 화면 및 늦은 구독을 확인: aerial 1개, map 2,853개, cells 2개 ADD. 원본 항공뷰·정적 지도·투명 HUD 표시 확인. VTD 연결은 여전히 거부되어 실제 주행 데이터 검증과 구분한다.
- 회귀 검사는 임시 디렉터리에 두었으며 프로젝트에 별도 테스트 패키지나 프레임워크를 추가하지 않았다. `docs/DataPipeline.md`는 작업 시작 체크섬과 동일하다.

## 검토 방법과 범위

이 대화 이력을 전달하지 않은 별도 `codex exec --ephemeral --sandbox read-only` 검토자에게 DataPipeline 전체와 Visualizer·TF·Bridge·메시지·RViz·CellTree·설정을 줄 번호와 함께 전달했다. 독립 검토자는 파일을 수정하지 않았다. 원문 보고서는 `/tmp/viz-audit/report.md`에 있다. 이 문서는 그 결과를 실제 지도·bringup 스크립트·실행 화면과 다시 대조한 판정이다. 독립 검토의 추측이나 입력 자료 부족을 확정 결함으로 옮기지 않았다.

`docs/DataPipeline.md`의 내용과 Mermaid는 이 작업에서 변경하지 않았다. 최신 사용자 결정인 **투명 HUD, 무관한 Ego 의존 제거, 전체 지도와 Cell polygon 표시**는 예외로 구분한다. 미구현 상류 계산을 Visualizer가 대신 생성하지 않는다.

## 119행 답변

`/traffic_light`의 ID/state는 `Visualizer.traffic_light()`가 HUD에 쓴다. world text를 HUD로 옮긴 최종 절의 계약 때문에 `/visualization/signals`의 TEXT_VIEW_FACING으로 나오지 않는 것은 누락이 아니다. 다만 지도 라벨 수천 줄 뒤에 묻힐 수 있었다. 지금은 HUD 앞쪽에 `signals/observed_controller/0`으로 표시하며, 수신 전에는 `waiting for /traffic_light`라고 쓴다.

**실제 누락은 121행의 관측 ID → registry 조회였다.** 이제 해당 controller의 regulatory element만 조회하고, 관련 물리 신호와 정지선 관계를 `/visualization/signals`의 `signals/observed/*`로 강조한다. 원본 관측 stamp를 계승한다. 전체 정적 신호 표시는 유지한다. 강조색은 선택 관계이지 개별 lamp의 실제 색이 아니다. 매핑이 없으면 그 사실을 HUD에 표시하며 가까운 신호를 임의로 대신 선택하지 않는다.

## 절별 대조

| DataPipeline | 확인한 실제 경로 | 판정 |
|---|---|---|
| 1 패킷/Bridge | 1109-byte decode → 동일 Header의 Ego·Objects·TrafficLight | 구현. 아래 Bridge 제약은 남아 있음 |
| 2 Objects | XY 중심·minZ 입력 → yaw footprint·직립 box(z+H/2)·heading·ID/speed HUD | 구현. offset 재적용 없음. footprint를 명세의 LINE_LIST로 맞춤 |
| 3 Ego | 후륜축 POINTS·heading, base_link CUBE, EgoStatus HUD | 표시 구현. 속도 추정기는 상류 책임 |
| 4 TF/view | EgoPose Header 계승, RPY quaternion, map→base_link, 10초 유휴 카메라 | 구현. 데이터에 camera transform을 재적용하지 않음 |
| 5 정적 지도 | 자기 hdmap_init, 경계·중심선·방향·도색 subtype·722 stopline·Cell polygon/AABB | 실제 지도 전체 개수 검사 통과 |
| 6 신호 | 관측 HUD + 관측 ID registry 조회 + 정적/관측 관계선 | 누락된 조회 연결 보완 |
| 7 점유 | cell-major 13-bin 중 선택 bin, 원본 p, alpha=p*(13-bin)/13, unknown 회색 | 구현. 예측을 재계산하지 않음 |
| 8 cap/limit | 원본 cap별 polygon 색, cap HUD, Float32 speed_limit HUD | 구현. cap display 기본 활성화 |
| 9 global/local | 전달된 lanelet ID 각각의 중심선, native RViz Path | 구현. 누락 lanelet을 HUD에 명시, 가짜 연결 없음 |
| 9 SearchTree | base_link 원본 Header, 부모 연결·yaw·최종 부모 사슬 | 구현. 선택 사슬도 LINE_LIST로 맞춤 |
| 10 Control | 받은 steer/accel/turn 요청값 HUD | 구현. 실제 적용·전송 완료로 표시하지 않음 |
| 11 query | 생산자 Cell polygon 복사·집계·count/window·직접 RViz | Python adapter 구현. 생산 노드 연결/C++ adapter는 미구현 |
| 12 공통 처리 | 원본 frame/stamp, lifetime=0, frame_locked=false, snapshot DELETE | 구현. Ego ROI는 최신 독립 스트림 요청으로 미적용 |
| 13 frame | map 관측/지도, base_link body/tree/path, scalar HUD | 자체 코드 대조 통과 |
| 최종 HUD 절 | 화면 고정·투명·스크롤·5Hz·직접 입력별 갱신 | 구현. namespace/ID·수치 정밀도 누락 보완 |

RViz Jazzy의 native Path 구현도 추가 확인했다. `PathDisplay::processMessage()`는 Buffer Length=1의 이전 선을 먼저 지우고 `msg->header`로 TF를 조회하며 실패하면 반환한다. 빈 Path는 이전 선을 지운다. 소스: https://raw.githubusercontent.com/ros2/rviz/jazzy/rviz_default_plugins/src/rviz_default_plugins/displays/path/path_display.cpp

## 독립 검토 지적 24개에 대한 판정

| 번호 | 지적 | 재검토/처리 |
|---|---|---|
| 1 | 문서가 Bridge를 미구현으로 표현 | 실제 Bridge는 구현됨. 보호된 DataPipeline 문구는 바꾸지 않음 |
| 2 | 객체 offset 오류가 Ego·신호까지 막음 | **확정, Bridge 잔여 문제.** decode_frame 예외가 publish_frame의 세 발행 이전에 발생함 |
| 3 | TCP 완성 레코드 중 마지막 것만 전달 | 확정. 전체 raw trace가 아님. 중간 프레임·신호 전이를 복원하지 않음 |
| 4 | Z가 실제 회전 box 꼭짓점 min이 아님 | 확정. API에 pitch/roll이 없어 yaw-only `ref_z+off_z-H/2`. Visualizer의 H/2 처리는 맞음 |
| 5 | controller 관측→기하 연결 없음 | 보완. 관측 ID 직접 조회·관련 기하 강조·미매핑 표시 |
| 6 | 공유 physical shape의 첫 controller/reg만 HUD에 남음 | 보완. 형상 중복만 제거하고 관계별 텍스트는 controller/reg/shape/stopline을 유지 |
| 7 | 정적 batching의 ID가 Cell ID가 아님 | 확정된 표시 제약. 모든 선분 보존, README에 배치 ID라고 명시. 생산자 query와 혼동 금지 |
| 8 | batch helper가 모든 메타데이터를 보존하지 않음 | 현재 정적 호출은 동일 frame/stamp/색/pose만 묶음. 서로 다른 관측 합치기에 사용하지 않음 |
| 9 | 일부 가상 subtype/line_thick 미지원 | 실제 지도에는 solid/dashed/solid_solid만 존재하여 현재 입력의 누락 아님. 다른 지도 지원을 주장하지 않음 |
| 10 | 기본 실행에 지도 없음 / cap 꺼짐 | 전자는 오탐: run.sh와 launch.sh가 HDMAP_PATH를 전달함. 후자는 수정: cap 기본 켬 |
| 11 | 프로세스 하나 종료 시 전체 종료 | 기존 bringup의 명시적 수명주기 계약. 토픽 하나의 미수신과 구분. 이번에 변경하지 않음 |
| 12 | previous가 직전 snapshot이 아닌 누적 ID 집합 | 재검토에서 수정. 전용 Display의 전체 교체로 바꾸어 영구 ID 이력을 제거 |
| 13 | BEST_EFFORT는 마지막 삭제 전달 보장 못 함 | 선택한 주행/마커 QoS의 제약. 수신 확인이나 완전 raw trace라고 주장하지 않음 |
| 14 | 거부된 입력과 이전 표시의 시각이 다를 수 있음 | 재검토에서 거부한 스트림의 이전 표시를 비움. HUD age는 여전히 수신 age이며 sensor latency가 아님 |
| 15 | 일부 HUD에서 namespace/id가 빠짐 | 보완. 관측 신호·점유·cap·speed limit·control 라벨에 식별 경로 유지 |
| 16 | g/.2f/.3f 수치 문자열의 반올림 | 확률·cap·속도·요청값을 round-trip 가능한 문자열로 변경. geometry/alpha 입력값은 전후 모두 원본 |
| 17 | 같은 Cell 개수의 다른 map을 검출 못 함 | 메시지의 ID-only 계약상 한계. hash/valid/seq를 임의로 추가하지 않음. 항상 local reference로 표시 |
| 18 | query AABB가 polygon에서 재계산됨 | 현재 Cell 생성자도 같은 polygon으로 bounding_box_를 계산함을 추가 확인. accessor 캐시 자체를 독립 검증하는 화면은 아님 |
| 19 | query 설정/0-hit/window 정보 한계 | 생성자 인자만 실제 적용. 0-hit 호출 수·실제 window 경계·drop count는 없음. count는 hit 집계 횟수 |
| 20 | footprint/신호/선택 사슬 marker 타입 다름 | LINE_LIST로 일치시킴. 실제 신호 shape는 646개 모두 2-point LineString으로 polygon closure 누락 없음 |
| 21 | 객체/global/query marker ID가 source ID가 아님 | 객체는 슬롯, global은 경로 순서, query는 집계 항목 index. 원본 ID는 별도 HUD/info로 구분. 현재 map ID는 int32 범위 |
| 22 | 누락 global lanelet이 로그에만 나타남 | 보완. 요청 ID 리스트와 MISSING 항목을 HUD에 표시 |
| 23 | 공통 ROI/설정 스타일 그림과 실제 다름 | 전체 입력·무관한 Ego 의존 제거는 최신 사용자 결정. 색/선폭 대부분은 현재 상수이며 runtime 초안 키 전부가 동작하는 것은 아님 |
| 24 | 실제 예측/계획/제어까지 구현된 것처럼 오독 | 상류 미구현과 분리. 요청값/관측값/파생값을 구분하고 없는 결과를 생성하지 않음 |

## 항공뷰 실행 중 발견한 결함

- Jazzy TriangleListMarker는 texture_resource에 파일 URI만 넣으면 텍스처를 로드하지 않아 흰 평면이 나옴 → PNG를 embedded texture로 전달.
- native marker가 텍스처에 조명을 적용하고 반복 갱신 때 texture unit을 누적함 → 정적 배경은 RELIABLE/TRANSIENT_LOCAL로 1회 발행, 최소 `visualization/Aerial` wrapper에서 조명/재질 색 곱셈 제거. 원본 PNG 색을 임의 증폭하지 않음. 정적 map/cells도 RELIABLE/TRANSIENT_LOCAL/KEEP_LAST(1)로 1회 발행하여 반복 정점 업로드를 제거했고 동적 스트림의 BEST_EFFORT는 유지함.
- 전체 PNG는 원본 OSGB를 알려진 bounds로 직접 렌더. 학교 구역의 별도 이미지와 같은 좌표 crop의 상관계수 0.9927637704. XY 좌표를 임의로 맞춘 결과가 아님.
- 이미지 해상도·6개 꼭짓점/UV·scale 검사와 실제 RViz 화면 확인. Z=0의 2D 배경이며 3D 지형이라고 표현하지 않음.

## 수행한 확인과 남은 범위

- 실제 지도: 2,847 lanelets, 94,157 Cell polygon과 AABB, 722 stoplines, 646 physical signals. 정적 batching 전후 모든 선분 수 동일.
- Ego 토픽을 전혀 주지 않은 격리 ROS domain에서 Objects/global/traffic_light/speed_limit/dynamic 표시 확인.
- 잘못된 SearchTree parent cycle 거부, base_link Header 보존, 점유 1/0.5의 alpha와 source stamp 확인.
- colcon visualization 빌드, git diff --check, DataPipeline SHA-256 불변 확인.
- 실제 VTD API는 현재 연결 거부 상태. 실시간 신호 수신이나 simulator 주행 검증이 끝났다고 주장하지 않음.
- Bridge의 프레임 결합·yaw-only Z, 상류 미구현, producer integration, ID-only map 일치 검증 한계는 **남아 있다.** 이번 보고서는 이것을 시각화 완료로 덮지 않는다.
