# VTD 배포본 검토 및 설계 근거

검토일: 2026-09-05. 구현 계약은 [README](../README.md), 이 문서는 근거와 검증 한계를 보존한다. 문서 설명, 배포 파일의 정적 분석, 실제 실행 검증은 구별한다. 이번 작업에서 시뮬레이터 실행·주행·재시작 실험·지도 변환은 하지 않았다.

## 1. 자료 범위와 우선순위

| 자료 | 검토 범위 및 의미 |
|---|---|
| [VTD 교육 자료](VTD%20교육%20자료.pdf), 53쪽 | 교육 흐름과 지도·차량·신호·정지선 구성. 특히 33–39쪽의 신호 및 정지선 모델 |
| [VTD User Manual](VTD_UserManual.pdf), 144쪽 | 목차/추출 텍스트를 통해 전체 구조를 조사하고 좌표계 27–29, 구성 30–50, 동기화/재생 91–94, 차량/센서 101–110, 통신 131–135쪽 집중 검토 |
| [Scenario Editor Manual](VTD_Scenario_Editor_User_Manual.pdf), 50쪽 | 경로·차량·traffic·trigger/action·controller 구조. 특히 35–38쪽의 controller phase와 lamp 시각 상태 구분 |
| [인터페이스 API](인터페이스%20API.xlsx) | 참가자에게 노출되는 필드/타입/단위. 일반 RDB API와 동일하지 않음 |
| `/home/stier/vtd 자료/00_HL_VTD.tar.gz` | 대회 플러그인·설정·차량 정의 |
| `/home/stier/vtd 자료/HL_FMA_VTD_LivingLab.xodr`, 같은 이름의 XML | 실제 제공 지도/시나리오 구조 대조 |
| `/home/stier/VIRES/VTD.2025.2` | 설치 플러그인·SDK RDB 구조체·Setup 확인 |

두 영문 매뉴얼은 2024.3이고 설치본은 2025.2다. 모든 그림을 개별 판독하거나 모든 기능을 시험한 검토는 아니다. 구체적인 참가자 API 동작에는 실제 배포 플러그인 분석을 우선하되 정적 분석을 실행 보증으로 취급하지 않는다.

## 2. VTD 전체 구조에서 중요한 구분

- SimServer는 실행을 관리하고 ParamServer는 설정을 제공한다. TaskControl은 동기화와 데이터 교환의 중심이며 Traffic은 내부 교통 참여자를, ModuleManager는 동역학/센서 플러그인을 관리한다. IG는 시각화 계층이다.
- XODR은 논리 도로망, OSGB는 시각 자산, 시나리오 XML은 배치·경로·행동·신호 제어를 정의한다. 화면에 선이 보이는 것과 Lanelet2 regulatory element가 존재하는 것은 다르다.
- VTD file finder는 프로젝트·Setup·배포 리소스를 탐색한다. 파일 이름만 맞추지 말고 실제 resolve한 파일의 절대 경로와 해시를 기록한다.
- RDB는 런타임 상태/제어, SCP는 설정/명령 프로토콜이다. 참가자 TCP 포맷은 별도 축약 프로토콜이다. SDK에 simTime이나 brakePedal이 있다고 참가자 API에도 있다고 판단하면 안 된다.
- Preparation과 Operation은 동역학이 다를 수 있다. 제동·조향 보정은 Operation의 실제 External ego 모델로 한다.
- 내부 주기, 센서 주기, 참가자 송신 주기는 다르다. ModuleManager의 100Hz 설명을 참가자 API 보장 주기로 사용하지 않는다.
- replay는 배속과 프레임 건너뛰기가 가능하다. 현재 수신 시각 기반 속도 추정은 정상 real-time 운용만 지원한다. pause/재생/비실시간 실행을 물리 시간과 동일시하지 않는다.
- Scenario Editor의 path/route, trigger, 강제 lane change, Beam, traffic recycling은 다양한 비연속/비준수 행동을 만들 수 있다. 현재 XML의 PulkTraffic은 비어 있으므로 일반 기능을 이 대회에서 활성화된 기능이라고 단정하지 않는다.
- 다른 차량이 신호와 제한속도를 준수한다고 가정하지 않는다. 녹색 phase와 배타적 통행권도 동일하지 않다. 카메라/IG가 없어도 본 설계는 API 상태로 동작한다.

## 3. 실제 참가자 wire 계약

배포 및 설치 `libHLVTD.so` SHA-256:

`6059d5466def4821a243835c52c116b86f613db83729f113fb8e766aab6eca4e`

이 해시의 x86-64 플러그인을 정적 분석한 결과다. 배포 버전이 바뀌면 다시 검사한다.

| 수신 byte offset | 내용 |
|---|---|
| 0–23 | Ego x/y/z/heading/pitch/roll, float32 6개 |
| 24–1103 | 객체 30개 × 36 bytes: uint32 id + float32 x/y/z/heading/speed/length/width/height |
| 1104–1107 | 선택된 신호 controller ID, int32 |
| 1108 | 신호 상태 uint8 |

수신은 1109 bytes 고정 길이 little-endian, 송신은 `<ffB>` 9 bytes: steering rad, targetAccel m/s², turnSignal. TCP이므로 recv 호출 단위가 frame이 아니다. 누적 버퍼로 완성 frame을 잘라야 한다. 남은 부분 frame을 버리거나 임의 위치에서 최신 1109 bytes를 취하지 않는다.

근거 함수: `sendDataPacket` 0x21550, `buildStudentObjectList` 0x26c50, `isDataTransmitDue` 0x214a0, `ControlApplier` 0x26680, `Module::applyControl` 0x21800. 주소는 이 바이너리 한정이다.

- 송신 판정 주기는 내부 simTime 기준 0.05s. 네트워크/스케줄링까지 wall-time 20Hz가 보장되는 뜻은 아니다. 미송신 최신 frame 교체가 가능하므로 수신 개수 × 0.05를 시뮬 시간으로 만들지 않는다.
- 참가자 frame에는 simTime, frame counter, 유효 객체 개수가 없다. ROS 쪽은 수신 시각을 Header.stamp에 담으며 별도 session/sequence를 추가하지 않는다.
- 객체 선택은 내부 후보 중 ego 제외, 유한 XY, ego와 XY 거리 80m 이내를 거리순으로 정렬해 최대 30개. 내부 후보 용량은 128이고 동거리 tie는 내부 순서다. 전체 시나리오 객체가 후보에 모두 들어온다는 보장은 없다. 남는 슬롯은 zero-fill된다. zero-fill 슬롯 제거 규칙은 유효 ID 범위와 함께 런타임 대조한다.
- 객체가 빠졌다는 것은 소멸 증거가 아니다. 80m 밖, top-30 탈락, 내부 후보 변경, 통신 문제를 구분할 수 없다. 예측 이력을 제한적으로 유지하고 관측 범위 밖을 무조건 free로 쓰지 않는다.
- 객체 speed는 XY 속력이며 heading은 자세다. 횡미끄럼 등에서 heading×speed가 실제 속도 벡터와 다르다. 위치 이력 기반 추정기를 분리한다.
- 일반 제어 경로는 RDB targetSteering/targetAccel을 사용한다. SDK의 별도 brakePedal은 참가자 wire에 없다. 최대제동 요청은 검증된 음의 targetAccel로 표현하며 실제 답력/달성 감속은 보장하지 않는다. 통신이 끊기면 SW가 브레이크 패킷을 전달할 수 없다. 연결 끊김 시 자동 풀브레이크도 가정하지 않는다.

## 4. 좌표와 차량 기준점

VTD inertial 좌표는 오른손 좌표계, 차량 로컬 축은 x 전방/y 좌측/z 상방이다. 차량 기준점은 후륜축 위치의 도로면 기준이고 chassis에 붙는다. 이 정의를 `base_link`에 사용한다. map 원점과 회전은 실제 XODR/VTD 변환 계약으로 고정하며 파일에 없는 지리 원점을 만들지 않는다.

RDB SDK `RDB_GEOMETRY_t`는 dimX/Y/Z 외에 offX/Y/Z를 별도로 제공한다. 객체 위치는 reference point다. 플러그인은 중심 offset을 내부에 보관하지만 참가자 객체 패킷에는 싣지 않는다. **크기를 안다고 reference point에서 box 중심까지의 오프셋을 알 수 있는 것은 아니다.**

Ego 배포 설정은 길이 4.848m, 전방 3.808m, 후방 1.040m이므로 box 중심은 후륜축보다 1.384m 전방이다. 대칭 `length/2`를 reference point 양쪽에 놓으면 틀린다. 이 값은 Ego에 대한 근거이지 모든 객체에 적용할 근거가 아니다. 객체별 모델/offset 검증 전에는 중심을 확인된 것처럼 표시하지 않는다. 불확실 footprint를 보수적으로 처리하고 debug에 가정을 드러낸다.

차량 모델에는 MaxDecel 9.5m/s², WheelBase 2.944m, MaxSteering 0.48rad 등이 있다. 이는 모델 설정이며 실측 제동거리나 참가자 명령의 달성값이 아니다. [vehicle.yaml](../config/vehicle.yaml)에 출처와 미검증 상태를 보존한다.

TF는 별도 `tf_broadcasting` 노드만 발행한다. Bridge `/ego_pose`의 시각과 pose로 map→base_link를 만들며 stale pose에 현재 시각을 붙이지 않는다. 기본 주행 diagram과 분리한다.

## 5. 신호 ID: lamp가 아니라 controller

**이 플러그인의 API ID는 물리 `signal/@id`가 아니라 XODR `controller/@id`를 의도한 값이다.** 같은 숫자가 두 namespace에 존재해도 같은 개체라는 뜻이 아니다.

예: road 76의 음수 lane → API ID 4. XODR controller 4는 signal 32, 33, 35를 제어한다. `API ID=4 → 물리 신호등 signal 4`로 매핑하면 잘못된다.

플러그인 `HlvtdTrafficLightMapping::kEntries`는 0x52da0의 135개 16-byte 고정 엔트리다. `(road_id, sign(lane_id)) → (controller_id, green_output_state)`이며 exact lane 번호·거리·방향각의 최근접 선택이 아니다. 따라서 같은 도로의 같은 부호 차선이라도 movement를 API만으로 구분할 수 없다.

근거: `findManualMapping` 0x236e0, `selectParticipantTrafficLight` 0x23d30, `publishSelectedTrafficLight` 0x238d0, `resetTrafficLightState` 0x23660. 초기화 때 고정 테이블을 복사하고 reset은 선택/상태 캐시를 초기화한다.

- 같은 플러그인·도로 ID·lane 부호·맵 구성이라면 재선택/재시작 후에도 같은 controller를 선택하는 구조다. 실행 중 관측 ID를 새로 발급하는 구조는 아니다. 실제 restart 실험을 수행한 보증은 아니다.
- 물리 위치가 같아도 XODR/controller ID나 플러그인 버전이 바뀌면 동일성이 보장되지 않는다. 좌표만으로 영속 ID를 약속하지 않는다.
- 미선택 또는 캐시 미확보 때 ID=0 또는 양수 ID에 state=0이 가능하다. 재등장 시 이전 색이 이어진다고 가정하지 않는다.
- API가 한 신호만 보여주므로 나머지 controller 색은 현재 관측이 아니다. 마지막 녹색 무기한 유지, 미선택=녹색, 임의 최근접 신호 대체를 금지한다.
- 테이블의 green 출력 플래그는 3 또는 5다. 엑셀의 좌회전 전용 4가 모든 상황에서 실제 생성된다고 가정하지 않는다. 표시 lamp mask와 API phase를 런타임 대조한다.

### 배포 파일 불일치

135개 매핑 중 131개만 제공 XODR의 top-level controller에서 확인된다. 다음 4개는 시나리오 XML에는 있지만 XODR에는 없다.

| road_id | lane 부호 | controller ID |
|---:|---:|---:|
| 2004 | + | 82 |
| 2011 | - | 80 |
| 2012 | + | 86 |
| 2003 | + | 84 |

이는 route에 영향 없다고 무시할 수 없는 배포 정합성 문제다. 최종 실행 시 resolve된 XODR을 확인하고 route 영향 및 실제 신호 동작을 검사해야 한다. Scenario Editor 매뉴얼의 controller/road 연결 설명도 시나리오 정의만으로 실제 도로 연결이 성립하지 않음을 뒷받침한다.

신호 매핑은 `(controller_id, movement) → stopline cell IDs + permitted states`로 저장한다. 사용자는 직진/좌회전 차선을 구분할 수 있어야 한다. UNKNOWN은 permission이 아니다. 황색에는 제동 가능 여부를 기준으로 STOP_REQUIRED 또는 COMMITTED를 latch하고 HOLD/CLEARING까지 접근 episode 상태를 유지한다. COMMITTED는 이미 정지할 수 없는 접근의 제한적 처리이지 미관측 빨간불 통과 허가가 아니다. movement가 같은 cell을 공유하는데 허용 상태가 다르면 하나의 speed cap만으로는 두 행동을 구별할 수 없으므로 지도 분할/보수적 공통 cap 중 명시적으로 결정한다.

## 6. 지도 구조 실측

XODR SHA-256: `5a369c7b0609fc98b0034680473465db4273dd396ffb37980996cea70ec6ca2f`

시나리오 XML SHA-256: `131eaf1a28000918a7ad58817677d7ebb7851dc2df739fc2036e496a10bbac7a`

| 항목 | 결과 |
|---|---:|
| road / junction / laneSection | 651 / 94 / 901 |
| signal / signalReference | 646 / 4 |
| top-level controller | 214 |
| object / roadMark / elevation | 6344 / 3000 / 1261 |
| speed 요소 | 0 |
| 정지선 시각 모델 `Rm_StopLine_300cm_JPN_01.flt` | 710 |

roadMark는 solid 1752, broken 1111, none 125, solid solid 12다. geoReference는 없고 header bounds는 0이므로 실제 지도 extent로 사용할 수 없다. 시나리오 참조명은 `LivingLAB_ver197_260804_final.xodr`로 제공 파일명과 다르다. 이름 차이가 내용 차이의 증거는 아니지만 resolve 경로 검사는 필수다.

710개 시각 모델은 710개 독립 의미 정지선을 뜻하지 않는다. 교육 자료 39쪽도 외부 3D 정지선 모델 삽입을 설명한다. stopline 의미·차선·controller 연결을 별도 검증해야 한다. speed 요소가 없으므로 Lanelet2 변환기가 대회 제한속도를 자동 복원한다고 가정하지 않는다. 속도 노면 표시도 전 도로 제한 계약을 대신하지 않는다.

cell ID는 사전 산출물에서 고정하고 메모리 로드 때 재할당하지 않는다. 프로세스별 copy는 manifest와 실제 메모리 geometry/order hash를 공개한다. C++ 구현을 Python binding으로 공유하고 lanelet은 기존 Lanelet2 laneletLayer에서 직접 조회한다. 별도 get_lanelet wrapper는 추가하지 않는다. 최신 Cell 코어는 부모 경계 Point3d를 자른 ConstPolygon3d와 그 AABB를 사용한다. 회전 box로 cell 형상을 대체하지 않는다. R-tree 후보 조회 뒤 cell polygon과 객체 OBB polygon을 교차 검사한다. 사전 변환은 지도 제작 단계이며 런타임 노드가 아니다. 메모리 로드 때 필요한 프로세스가 역색인을 한 번 생성한다. R-tree/교차/바인딩은 아직 구현하지 않았다.

## 7. 시간·예측·제동 계약의 한계

`/clock` 없이 `use_sim_time=false`, Bridge system ROS 수신 stamp를 계승한다. 여러 호스트에서는 system clock 동기화가 필요하다. RTT/수신 지연은 소스 시각으로 복원할 수 없다. timeout은 monotonic 시계로 측정한다. 큰 time jump·session 변경·비정상 dt는 이력 초기화다.

KeepLast(1)은 DDS 대기 sample 한 개이지 계산 중 작업 취소/결과 만료 보장이 아니다. 콜백에서 최신 완성 snapshot 슬롯을 원자적으로 교체하고 worker는 하나만 실행한다. 오래된 결과는 source stamp로 폐기한다. Volatile은 late joiner에게 과거 sample을 보관해 전달하지 않는다는 의미다. BestEffort는 재전송 보장 없이 최신 데이터를 우선한다.

점유 bin 0은 현재, bin k=1..12는 `((k-1)*0.5, k*0.5]`다. 경계 순간은 중복 저장하지 않는다. 구간 내부를 잠깐이라도 점유하면 해당 bin을 점유 처리한다. endpoint 두 장만 보는 방식은 불충분하며 swept volume 또는 보수적 연속시간 bound가 필요하다. 미래 1.0은 선택한 예측/불확실성 envelope의 점유이지 현실 확률 100% 보증이 아니다. unknown은 free=0과 구별한다.

EKF는 별도 파일/인터페이스로 분리한다. CV는 Constant Velocity이며 카메라가 아니다. Hybrid A*는 각 primitive의 속도 제한과 동역학으로 도달 시간을 적분해 위 bin과 충돌 검사한다. 표준 Path는 실행 도착 시간을 담지 않으므로 실제 추종 오차에 대한 시간 여유와 재계획이 필요하다. Annotator와 별도 시간 profile 공유를 요구하지 않는다.

선형 `v(d)=v_entry*d/D`의 초입 감속은 `v_entry²/D`다. 따라서 `D=v_entry²/(2*a)`로 놓으면 요구 감속이 2a다. 사용자가 원하는 선형장에 factor를 적용하되 물리적으로 가능한 base distance를 먼저 산정해야 한다. README는 `max(실측 제동거리, v_entry²/a_design)`를 base로 제안한다. factor=1이 가장 적극적이며 임의로 원래 제동거리와 동일하다고 속이지 않는다. 저속 점근·1m 이산화·전방 끝 기준·지연·정지 유지까지 실측 검증 전 motion enable을 막는다.

## 8. 디버깅과 검증 계획

각 producer가 보낸 실제 query box/cell box는 Visualizer의 map으로 다시 lookup하여 바꿔 그리지 않는다. producer/session/map hash/source stamp별 layer를 유지하고 mismatch/stale/drop을 표시한다. 합산 view는 raw view와 분리하고 dedup 횟수·window·누락을 공개한다. 단, 후속 사용자 결정인 `/global_path`는 ID-only 리스트이므로 Visualizer map으로 복원한다. 이를 producer geometry가 아닌 복원 표시로 명시하고 map 일치 여부를 별도 확인한다. 상세 계약은 README를 따른다.

### 후속 질문: 탐색 비용, 지도 완성, 플러그인 소스

XODR 재집계 결과 road 651개, laneSection 901개, 좌우 driving lane 구간 2,480개, road 기준 총 길이 약 33.245km다. 전체 driving 태그 3,282개에는 center lane 항목도 포함되므로 실제 주행 그래프 노드 수로 그대로 쓰면 안 된다. 변환 시 정지선/규칙 경계에서 추가 분할되므로 최종 노드 수는 미확정이다. 1m cell 수는 Lanelet2 routing 노드 수와 별개다.

규모 판단용 가정 V=5,000, E=20,000, shortestPath 8회라면 binary-heap Dijkstra의 O(K*(V+E)*log2(V)) 규모항은 약 246만이다. 이는 실제 명령 횟수나 시간 측정이 아니다. C++ 희소 그래프 조회를 10Hz로 시도할 합리적인 크기이며, 초기 구현은 매 계획 tick 탐색하고 실제 10ms 글로벌 예산 초과 여부로 최적화한다. 원본 소스의 shortestPathVia는 구간별 탐색을 연결하며 Python 바인딩 이름은 shortestPathWithVia다.

Lanelet2 완성 작업은 변환 성공과 동의어가 아니다. 좌우 경계/진행 방향/접속/실선 차선변경 금지/정지선/controller/movement/속도 구간/원본 ID provenance까지 보강하고 검증해야 한다. 실행 주행 전에 대부분 정적 구축이 가능하고 실제 주행은 최종 정합성 검증에 사용한다. 알 수 없는 규칙을 차량을 몰아 추정하면 정답이 생기는 것은 아니다.

속도 단서는 roadmark_speed_30.flt 71개, 30.flt 2개이며 BldResSchool01.flt 학교 모델은 road 190에 1개다. 숫자 노면 표시는 제한속도 후보지만 학교 건물이나 모델명만으로 보호구역의 시작/종료·적용 방향을 확정할 수 없다. 원본 XODR/OSGB/시나리오에서 표지와 노면 표시의 위치·방향을 대조하고 대회 규칙의 구간 정의와 결합해야 한다. 확인된 보호구역 운용 cap은 사용자 결정에 따라 config의 school_zone_speed_cap_mps=8.0을 사용한다. 원본 30km/h 제한과 보수적인 운용값은 출처상 구분하고 더 낮은 제한이 있으면 최솟값을 사용한다. 구간 경계에서 lanelet을 나누고 근거·원본 object ID·적용 road/lane/s 구간·검증 상태를 지도 보강 산출물에 남긴다. 아직 이 보강/변환은 수행하지 않았다.

노면 색/문자는 정적 에셋에서 주행 없이 확인할 수 있는 정보다. 설치본 Runtime/Core/IG64/bin/osgconv와 ModelConverter의 osgviewer/osgconv를 발견했다. 후속 작업은 OSGB scene graph의 geometry/material/texture와 FLT 모델을 추출·정적 렌더링하고 XODR object 변환과 대조하는 것이다. 문자열 검색만으로 compressed OSGB의 표면 내용을 판독했다고 할 수 없으며, 이번 작업에서는 보호구역 텍스처의 실제 문구/경계를 아직 시각 확인하지 않았다. 색만으로 구역을 확정하지 않고 표지·문구·범위를 함께 확인한다.

최신 ROS 계약에서는 Tracker가 EgoStatus를 분리 발행한다. EgoStatus는 Header와 float32 x/y/z/heading/pitch/roll/speed만 담는다. DynamicStatus는 Header와 cell 배열만 담고 Control은 이를 구독하지 않는다. Planner/Annotator는 Header 시각을 맞춰 사용하며 Control은 EgoStatus, local_path, speed_limit만 사용한다. 별도 seq/session/valid 필드와 `/speed_info`는 없다. debug 집계는 기본 활성화다. 상세 필드는 README와 msg 파일을 따른다.

배포 archive에서 참가자 플러그인의 C++ 원본 소스는 찾지 못했다. 확인한 것은 symbol이 남은 libHLVTD.so이며 nm/objdump와 고정 데이터 테이블을 사용한 바이너리 정적 분석이다. SDK header를 읽을 수 있다는 사실과 이 플러그인의 원본 C++를 보유했다는 것은 다르다. 현재 소켓을 연결해 API를 실시간 관측한 것도 아니다.

API controller 4의 위치 추적은 XODR control 참조를 따라 lamp 32/33/35로 연결된다. 세 lamp는 road 76, s=44m, t=0/-0.4/-0.8m, zOffset=4.6m에 있다. 이것은 OpenDRIVE 도로 좌표이며 map XYZ로는 planView와 elevation을 평가해야 한다. 이처럼 controller에서 물리 lamp 집합까지 추적할 수 있지만 API 하나가 개별 lamp 하나의 관측이라는 뜻은 아니다. 누락된 controller 80/82/84/86은 이 참조 체인이 끊겨 있으므로 실제 resolve 지도와 시나리오를 더 대조하기 전 위치를 임의로 확정하지 않는다.

Marker lifetime=0, producer 전용 namespace/topic에서 명시적으로 교체/삭제한다. topic을 공유하며 DELETEALL로 다른 producer 표시까지 지우지 않는다. ROI100m와 occupancy bin 선택은 표시 설정이며 알고리즘 입력을 변경하지 않는다. 점선/실선·중앙선·진행 방향·정지선·lamp geometry 및 관측/Tracker 해석을 구분한다.

| 실행 검증 | 통과 조건 |
|---|---|
| packet framing/네트워크 부하 | 분할·병합·지연·drop에도 frame 정렬 유지; 조용한 stale command 재사용 없음 |
| 신호 ID 재등장/재시작 | 동일 배포 hash에서 road/lane 키와 controller 대조; unknown 전환 기록 |
| 신호 135개 테이블 | 최종 resolve 지도와 전 항목 연결, 누락 4개 해결 또는 해당 경로 명시 차단 |
| 좌표/box | Ego 네 모서리·축, 객체 reference offset을 실제 모델/관측과 대조 |
| velocity | 시작·같은 stamp·역행·respawn·300km/h 초과에서 속도 0 처리; 별도 플래그 없음 |
| 제동/조향 | Operation에서 명령-실제 반응, 지연, 속도별 제동거리 측정; 선형장 정지 오차 검증 |
| 예측/Planner | 구간 사이 빠른 통과, 경계 시각, unknown, full footprint, 차선변경 중 충돌 검사 |
| TF/시각화 | 유일 broadcaster, source stamp 보존, jump/stale/hash 차이를 숨기지 않음 |
| 10Hz 성능 | 최대 cell/object와 debug on/off에서 deadline 및 DDS Python 역직렬화 비용 측정 |

## 9. 외부 기술 기준

- [ROS 2 Jazzy QoS](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [ROS 2 시간 설계](https://design.ros2.org/articles/clock_and_time.html)
- [REP-103 좌표와 단위](https://www.ros.org/reps/rep-0103.html)
- [nav_msgs/Path](https://docs.ros.org/en/jazzy/p/nav_msgs/msg/Path.html)

이 문서는 API에 없는 관측 정보를 만들어내거나 전체 지도 점유를 완전 관측이라고 보장하지 않는다. 변환 도구 버전 고정 및 실제 Lanelet2/cell 산출물 검증은 후속 작업이다.
