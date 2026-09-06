# HD map 실제 VTD 주행 감사

2026-09-06, LivingLab / VTD 2025.2. 이 문서는 지도와 독립적인 VTD native 차량의 RDB 관성 좌표를 HD map에 대조한 **분산 단거리 주행 샘플 검사**를 기록한다. 한 차량이 모든 도로를 연속 완주한 시험, 전체 cell을 통과한 시험, 신호·경로계획·충돌 회피의 무결점 인증은 아니다.

## 시험 방법과 증거

`map/tools/drive_map_audit.py`는 기존 `config/tools/measure_object_offsets.py`의 SCP/RDB 형식을 재사용한다. 원본 지도 `xodr_road_id / xodr_lane_id / xodr_s_begin / xodr_s_end`에서 대상을 고르되, 실제 배치는 VTD `TrackPos`가 수행하고 VTD 내부 운전자가 `SmartForTwo_14_WhiteBlack` 차량을 운전한다. 지도 중심선 좌표를 차량 위치로 재생하지 않는다.

- SCP `127.0.0.1:48179`, RDB `127.0.0.1:48190`. 임시 NPC 이름만 `MapAudit*`를 사용한다. 원본 시나리오 및 OpenDRIVE 파일은 수정하지 않는다. 복구 중 재초기화된 Ego 위치·자세·정지 속도는 종료 전에 원래 snapshot과 비교한다.
- 차량 생성 후 3초, 배치 후 0.7초를 기다린다. 주행 목표 속도는 6 m/s이며 신호와 교통표지 준수를 켠다. 주행 구간은 대상마다 벽시계 6초다.
- `placement`와 `drive`를 별도 기록한다. 거리 합계는 `drive` 중 연속 RDB XY의 차이만 사용한다. 대상 재배치의 순간 이동은 제외한다. 0 < Δt ≤ 0.5초이고 이동이 max(0.5m, 20m/s × Δt) 이내인 쌍만 적산한다.
- lanelet은 실제 2D polygon 거리 0.02m 이내, cell은 기준점 주변 ±0.02m 사각형과 z ±0.5m의 겹침으로 검사한다. 차량 전체 외형이나 바퀴 접지면 검사가 아니다.
- 교차로에서 겹쳐 보이는 다른 경로를 잘못 커버했다고 세지 않도록, **RDB road/lane 번호가 해당 cell의 부모 lanelet 원본 road/lane과 일치하고 실제 이동한 경우**만 엄격 커버리지에 포함한다. RDB road 상태는 마지막 수신값이며 도로를 벗어날 때 갱신이 멈춘 사례가 있었다. XY와 source 도로 투영을 함께 봐야 한다.
- 초기 시험은 실시간 RDB 약 25Hz에서 검사하고, 재현 로그에는 일반 좌표 약 8.3Hz(0.099초 이상 간격; 25Hz 프레임에서는 0.12초 간격)와 모든 containment miss 좌표를 보존했다. 초기 시험의 지도 재검사는 이 **저장된 좌표**만 사용하므로 실시간 검사보다 cell 커버리지가 작을 수 있다. 이후 명시적 t 전역 재시험은 `--all-samples`로 모든 수신 좌표를 보존했다.
- 내부 driver의 신호 준수 옵션은 적색 정지의 성공을 보증하는 측정이 아니다. controller별 적색/황색/녹색 통과 행태는 이 시험에서 전수 검증하지 않았다.

## 기준 지도 전역 시험

기준 지도 SHA-256:
`7759de6a84ba9ef6fd7807792c642d5e479a981c37cbc38f570fc4912420bc78`

원본 643 driving road, 2,847 lanelet, 94,157 cell을 대상으로 lanelet별 1개 배치를 정해 64대씩 분산 주행했다. 총 2,847개 stint에서 473,977개 RDB 상태를 처리했다.

| 항목 | 결과 |
| --- | ---: |
| 실제 XY 이동 거리 합계 | 74,792.241m |
| 엄격 매칭으로 이동이 관측된 road | 591 / 643 |
| 엄격 매칭으로 이동이 관측된 lanelet | 2,760 / 2,847 |
| 엄격 매칭으로 이동이 관측된 cell | 45,788 / 94,157 |
| 목표 lanelet에서 이동 확인, 일반 containment miss 없음 | 2,667 stint |
| 차량은 움직였으나 목표 lanelet의 엄격 이동거리 미달 | 166 stint |
| 일반 lanelet 또는 cell containment miss 있음 | 14 stint |
| RDB 없음 / 이동 없음 | 0 / 0 stint |

**74.792km는 여러 차량과 여러 재배치 구간의 합계**다. 동일 구간의 반복 이동도 포함하며, 지도 고유 도로 길이 또는 연속 완주 거리가 아니다. 45,788개 cell에서 이동이 관측됐다는 사실은 나머지 48,369개 cell의 주행을 검증하지 못했다는 뜻이다. `target_not_driven`은 실제 도로 누락 확정값이 아니다. 교차로 중첩·native 경로 선택·RDB road/lane 투영 차이도 포함한다.

## 보강 지도에서 명시적 t 전역 재시험

이 단계의 지도 SHA-256은 `758f2990e28374036725bc88be2938e5857e1f9a6035e3017a83692d78e6c718`이다. 당시 2,846개 lanelet의 진입 s에서 원본 laneOffset와 누적 width의 절반으로 차로 중심 t를 계산해 SCP에 명시했다. 이 계산은 HD map의 `width_start_overrides`를 적용하지 않은 VTD 원본 기준이다.

| 항목 | 결과 |
| --- | ---: |
| 대상 / RDB와 실제 이동 관측 | 2,846 / 2,846 stint |
| 처리한 RDB 상태 | 473,840 |
| 실제 XY 이동 거리 합계 | 74,743.460m |
| source road/lane·polygon·원본 중심 t가 일치한 배치 | 2,658 / 2,846 |
| 엄격 매칭 이동 road / lanelet / cell | 586 / 2,744 / 45,562 |
| 목표 lanelet 이동, 일반 containment miss 없음 | 2,636 stint |
| 목표 lanelet 엄격 이동거리 미달 | 193 stint |
| 일반 containment miss | 17 stint |
| RDB 없음 / 이동 없음 / 시계 중단 | 0 / 0 / 0 |

배치 검증과 실제 주행 커버리지는 서로 다른 지표다. 교차로의 겹친 도로에서 native가 요청과 다른 RDB road/lane을 선택한 배치는 검증 실패로 기록한다. 이후 실제 이동으로 관측된 다른 도로의 cell은 그 도로의 관측으로만 계산한다. 명시적 t도 native의 연결도로 선택을 강제하지 않는다. 예를 들어 road6024의 s=1m 배치는 RDB road791/-1로 분류돼 27.972m를 움직였지만 road6024 목표 이동은 0m였다.

원본 폭 시작점에 결함이 있는 road1927 section3 lane+4는 이 시험에서 진행 방향에 따라 section 끝 s=158.136m에 배치됐고, 그 지점의 원본 폭3.3m·t11.55m가 실제 상태와 일치했다. 이것을 원본 width 첫 record 이전 구간까지 검증한 결과로 확대 해석해서는 안 된다.

17개 miss는 최대 약2.500m의 border/sidewalk/parking/none 진입 및 일부 도로 전환 시점으로 구성됐다. 최근접 기록된 다른 MapAudit 차량은 최소15.46m였고 4m 이내 근접은 없었다. 원본 reported driving만 남은 짧은 전환 사례는 별도 좌표 투영 검토 대상으로 보존했다. 기준점 이탈을 근거로 driving polygon을 확대하지 않았다.

## 단독 재현과 native 이탈

전역 시험에서 미커버였던 52개 road는 원본 s 구간 중앙에서 새 NPC 1대씩 재배치해 다시 6초간 운전을 요청했다. 51개에서 실제 주행했고, 1개는 아래의 Traffic 종료 사례로 남았다. 원래 containment miss가 있던 14개 target도 새 NPC 단독으로 다시 시험했다. 14개 모두 실제 이동했으며 7개에서 containment miss가 다시 나타나고 7개에서는 나타나지 않았다. 이 재시험의 합산 이동은 388.982m다. 결과와 최종 수정 지도 커버리지는 아래 산출물의 summary 및 stint manifest에 기록한다.

전역 14개 miss의 원본 차로 유형과 거리 대조에서는 border, sidewalk, parking, none 구역에 실제 진입한 사례가 확인됐다. 가장 큰 이탈은 driving polygon에서 약 2.514m다. 기록된 다른 MapAudit 차량과 4m 이내 근접한 miss는 없으며, 최소 거리도 18.88m였다. 따라서 64대 임시 차량끼리의 근접·충돌을 설명으로 뒷받침하는 증거는 없다. 로그에 없는 Ego와 원래 시나리오의 다른 traffic은 이 근접 계산에 포함되지 않는다.

road 211 / target 36924의 6프레임은 예외처럼 보였다. RDB road s/t는 동일한 driving 위치를 반복했지만, raw XY를 원본 planView에 투영하면 s가 -0.084m에서 -0.893m로 변해 이미 도로 시작점 밖이었다. 원본 junction 12에서 road 211의 +1차로는 후속 연결이 없고 +2/+3/+4만 road 1174로 연결된다. 이 사례는 지도 polygon 누락의 근거가 아니라 고정된 RDB road 메타데이터와 native 진입 행동의 문제로 분류한다.

중앙 배치 target 537427 / road 6024 / lane -1 / s=15.9884m에서 VTD Traffic(`ghostdriver`) 프로세스가 종료됐다. 마지막 위치는 (524.3313, -154.8539, 42.0)이며 TaskControl은 Traffic 연결 부재를 기록했다. 해당 단독 시험의 정지는 신호 정지로 판정하지 않는다. 이후 RDB가 없던 시도는 미검증으로 남기고, Traffic만 공식 설정으로 복구해 나머지를 재시도했다. 원인을 확정하지 않은 시뮬레이터 실패 사례다. 이 첫 배치의 XY는 원본 road6024 reference t=0에 해당했다. 공식 SCP의 lane 대안을 사용했지만 특정 중첩 도로에서 요청한 차로 중심이 실현되지 않았다. 다른 2,685개 전역 배치는 RDB road/lane이 요청과 일치하고 laneOffset 절댓값이 모두 0.2m 이내였으므로 전역적인 좌표 명령 오사용으로 일반화할 수 없다.

추가로 원본 laneOffset와 누적 width에서 계산한 t=-1.46755를 명시했다. 이번에는 원본 차로 중심 (525.758214, -155.196813)에 정확히 배치됐지만 RDB가 중첩 road207/-1을 선택했고, 약 0.28초의 정지 상태 뒤 Traffic가 다시 종료됐다. 명시적 t로도 재현되는 native junction/driver 실패다. 커널 로그에서 두 번 모두 vendor `ghostdriver`의 같은 offset `0x27994c`에서 null 읽기 segfault가 확인됐다. `addr2line`은 `DT_OdrPath::getNeighbourIfToNarrow(OpenDrive::Path::Leg::LaneList::LaneInfo const&) const`를 가리키며 fault instruction은 `mov (%rsi),%rsi`, rsi=0이었다. 차로 폭·인접 차로 선택 중의 native null 참조로 좁혀졌으며, 어떤 입력 조합이 해당 null 상태를 만드는지는 미확정이다. 바이너리는 수정하지 않았다. 근거: [`vtd-traffic-kernel-segfault.txt`](../map/audit/vtd-traffic-kernel-segfault.txt), Build ID `786df1c0b7a919438438c6ea6bf3dafaa7f92ae9`.

이러한 native 이탈이나 임의 배치 후 실패를 보상하기 위해 보도·경계·주차 구역에 driving lanelet을 확장하지 않았다.


## 최종 지도에서 저장된 전체 좌표 재검사

현재 배포 지도 SHA-256은 `e7a426f900813748747a39233f3678237fd7f849c335bae3bc3014b31ae678b0`이며, 643개 source driving road, 2,845개 lanelet, 94,154개 cell이다. 기존의 모든 분산·단독·추가 주행과 부분중단에서 보존한 실제 RDB XY를 이 지도에 다시 질의했다. 과거 지도 cell ID를 직접 합산하지 않았다.

| 최종 raw 좌표 재검사 항목 | 결과 |
| --- | ---: |
| 저장된 drive 상태 | 1,124,807 |
| 기준점 polygon/cell + RDB 부모 일치 이동 | 639 roads / 2,829 lanelets / 82,493 cells |
| 위 조건과 차체 헤딩·실제 XY 이동방향 모두 일치 | 639 / 643 roads, 2,829 / 2,845 lanelets, 82,381 / 94,154 cells (87.496%) |
| 방향 일치 이동 미검증 cell | 11,773 |
| 전혀 엄격 이동이 관측되지 않은 source road | 2752, 2898, 3382, 6024 |
| 저장 좌표의 실제 XY 거리 합계 | 251,196.387m |

거리251.196km는 다차량·다회 재배치·중복 구간의 합계다. 방향 검사는 cell 중간점의 주행 접선과 차량 헤딩, 실제 XY 이동벡터의 내적이 둘 다 양수인 표본만 받는다. 반대 방향 성분이 있는 표본은 해당 cell의 정상 방향 커버로 세지 않는다. 이 기준은 정확한 횡방향 추종, 차량 외형 전체의 차로 내 존재, 신호 준수 또는 연속 경로 완주를 보증하지 않는다.

같은 저장 표본 전체에 별도의 [RDB 차체 박스 감사](HDMapFootprintAudit.md)도 완료했다. 기준점 검사에 통과한 이동 표본 중 16,743개에서 실측 검증된 bbox의 driving 밖 면적이 0.05㎡를 넘었다. 이것은 반복 프레임을 포함한 이탈 후보 집계이며 실제 mesh·타이어 충돌 확정값은 아니다.

같은 교차로에서 다른 도로로의 RDB 재선택이 실제 주행 방향과 충돌하는 사례도 보존했다. source와 일치하는 모든 cell의 방향이 반대인 5,888개 표본을 제외하면 고유 커버 cell이112개 줄어든다. 나머지는 같은 cell에서 다른 시점에 올바른 방향 이동이 있었던 경우도 포함한다. 일반 containment miss는2,436개 lanelet 표본 및2,397개 cell 표본이며 서로 겹친다. 이것은 서로 다른 지도 오류 개수가 아니며 source 투영으로 native 이탈·stale RDB 상태를 별도 분류했다.

![VTD 실제 주행과 최종 지도 방향 일치 커버리지](assets/hdmap-audit/driving/drive-final-coverage.png)

추가1,595개 타깃의 분산 실행은257.0초 벽시계 후 정상 종료했다. 1,426개 sampled_motion,148개 배치 차단,17개 containment miss,3개 실제 정지,1개 목표 lanelet 이동 미달이었다. 이 실행의 합산 이동은41,451.21m다. 정상 종료가 해당17개 native 이탈까지 없었다는 뜻은 아니다.17개 miss의 최대 driving 경계 거리는2.810m, 기록된 다른 임시차량 최근접은9.224m이며4m 이내는 없었다.

`driving-cleanup-final.json`은 분산 실행 뒤 임시 MapAudit 이름0개, Ego 원 snapshot과 마지막3개 RDB XYZ/HPR/speed 오차 모두0을 기록한다. 종료 시 Stop을 전송했다. 과거 주행의 RDB에는 center offset을 저장하지 않았으므로 차체 footprint 재구성은 동형 모델의 별도 측정 근거를 필요로 한다. 도구 최종본은 향후 수신 `center_offset=STATE[8:11]`도 보존한다.

## 재현 명령

VTD에서 LivingLab을 로드하고 RUN 상태로 만든 뒤 실행한다. 동시 감사 실행은 금지한다. 실행은 임시 NPC를 생성하며 종료 시 삭제와 `Stop`을 전송한다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/install/hdmap_core/lib/python3.12/site-packages:$PYTHONPATH"
python3 map/tools/drive_map_audit.py --self-check
python3 map/tools/drive_map_audit.py \
  --map map/hdmap.bin --output /tmp/hdmap-drive-new \
  --xodr "/home/stier/vtd 자료/HL_FMA_VTD_LivingLab.xodr" \
  --fleet 64 --seconds 6 --all-samples
```

JSON 배열에 원하는 lanelet ID를 넣으면 단독 중앙 배치 재현을 할 수 있다. 재현마다 native 내부 운전자 이력을 새로 만들기 위해 `--fresh`를 쓴다.

```bash
python3 map/tools/drive_map_audit.py \
  --map map/hdmap.bin --output /tmp/hdmap-drive-isolated \
  --xodr "/home/stier/vtd 자료/HL_FMA_VTD_LivingLab.xodr" \
  --fleet 1 --seconds 6 --fresh --placement-fraction .5 \
  --lanelet-ids /tmp/target-lanelet-ids.json
```

저장된 RDB 로그를 수정된 지도에 다시 대조하는 작업은 VTD를 조작하지 않는다.

```bash
python3 map/tools/drive_map_audit.py \
  --map map/hdmap.bin --output /tmp/hdmap-drive-replay \
  --replay docs/assets/hdmap-audit/driving/drive-global
```

lanelet ID는 지도 버전에 따라 달라지므로 대상 배열은 같은 버전의 지도에서 생성해야 한다. 최종 지도에서 제거된 극소 폭 lanelet은 새로운 target 생성에는 포함하지 않지만, 과거 시험의 미검증/실패 기록은 보존한다. 다른 버전의 cell ID를 그대로 비교하지 말고, 항상 RDB XYZ를 새 지도에 재쿼리한다. 지도 SHA-256은 모든 실행 summary에 기록된다.


## 미커버 구간 추가 주행과 배치 검증

명시적 t 전역 시험까지 합친 저장 좌표의 재검사에서 44,478개 cell, 약43.213 lane-km가 아직 미커버였다. 이 길이의 89.2%는 이미 일부를 주행한 lanelet의 후반부였다. 이를 실제로 더 검사하기 위해 각 미커버 구간에 약20m 간격의 source s 출발점을 만들었다. 총4,049개 타깃이며, 같은 lanelet을 여러 s에서 출발하는 경우가 있다. 앞선 실패로 단독 두 번 종료가 입증된 road6024의 35개 cell은 동일 조건을 반복하지 않고 known failure로 유지했다. 계획의 cell 수는 실제 커버 수와 다르다.

첫 추가 실행은 1,792개 타깃에서 실제 표본을 얻은 뒤 다음64개 배치에서 native Traffic가 종료됐다. 원본 후속 실행의 세 번째64대 배치는 4.56초만 관측된 부분 주행이며, 요청한6초 완주가 아니다. 그 실제 좌표와 거리는 유효한 관측으로 보존한다. 원래 status는 당시 기준으로 일부 sampled_motion이었지만 `interrupted-batch.json`이 64개 전부의 4.56초 중단 문맥을 보존한다. 이후64개 시도는 Traffic 종료 뒤 RDB가 없었으므로 다시 미검증 대상으로 넣었다. 재시작 직후 scenario가 초기화되기 전에 시작했던 별도 실행은 **0개 주행**으로 보존했다. 공식 Init과 POST-INIT 확인, Start/RUN, 실제 RDB 수신 및 Ego 원위치 확인을 순서대로 마친 후 나머지2,001개 타깃을 재개했다.

마지막 실행의 `--require-placement`는 `--fresh`를 자동 적용한다. 각 타깃에서 수신 road/lane, 대상 polygon, 요청 s와 ±1m, **수신 s에서 계산한** 원본 차로중심 t와 ±0.2m 일치를 확인한다. 0.7초 안정화 중 수십 cm의 이동이 있으므로 요청 s의 정확한 정지점 배치를 증명하는 조건은 아니다. 원본 폭이 실제 RDB 차량폭 +0.2m보다 좁거나 배치가 검증되지 않은 NPC는 주행 전에 삭제한다. 이 타깃은 `placement_mismatch` 및 미검증으로 유지하며 커버리지의 분모에서 빼지 않는다. 실제 관측된 차량폭은 약1.652m다. 이후 source 도로 선택이나 주행 방향까지 이 배치 검증이 보증하지는 않는다.

`sim_duration_s`와 native 종료 문맥이 실제 관측 길이를 나타낸다. 도구 최종본은 요청 벽시계의90% 이상 simulation 시간 표본이 있는지를 `requested_duration_completed`로 별도 기록한다. 이 값도 경로 끝까지의 완주를 뜻하지 않는다. 기존 실행은 당시 status를 변경하지 않고 부분중단 manifest를 추가했다.

### Native 실패의 최소 재현 정보

원본 XODR SHA-256은 `5a369c7b0609fc98b0034680473465db4273dd396ffb37980996cea70ec6ca2f`다. VTD 2025.2 내부 운전자와 `SmartForTwo_14_WhiteBlack`을 사용했다. 다음 명령의 연결은 SCP TCP48179이며, 생성 후3초 및 배치 후0.7초를 관측한다. 실제 조작은 실패 증거를 이해하기 위한 예시이며, 이미 두 번 종료된 road6024 조건은 추가 전역 계획에서 제외했다.

```xml
<Player name="MapAudit000"><Create category="vehicle" vehicle="SmartForTwo_14_WhiteBlack" control="internal"/></Player>
<Player name="MapAudit000"><DriverBehavior desiredSpeed="0" obeyTrafficSigns="true" obeyTrafficLights="true"/></Player>
<Set entity="player" name="MapAudit000"><Speed value="0"/></Set>
<Set entity="player" name="MapAudit000"><TrackPos track="6024" lane="-1" s="15.9884041176" t="-1.46755" dhDeg="0"/></Set>
<Player name="MapAudit000"><DriverBehavior desiredSpeed="6" obeyTrafficSigns="true" obeyTrafficLights="true"/></Player>
```

6024의 명시적 t 배치는 raw XY가 원본 차로중심과 일치했으나 실제 RDB road/lane은207/-1이었다. t를 생략했던 최초 단독 배치는 원본 reference t0 및207/-2로 관측됐다. **두 단독 실행의 같은 함수 종료는 재현됨**으로 기록한다. 후속64대 배치의845→688 및2789→2783은 마지막 상태·native 로그·원본 겹침이 지목하는 후보이며, 개별 actor stack으로 원인이 확정된 것은 아니다. 845 배치의 원본 좌표 오차는2.84e-14m였고 같은 XY의688/-4 경로와101.35°로 교차했다. 해당 다른 차로의 원본 폭은3.2336m로, 요청 도로 자체가 극소 폭이라는 설명을 뒷받침하지 않는다. 최근접 다른 임시차량은119.3m였다.

vendor fault offset, Build ID, null 참조 명령은 앞 절 및 kernel 증거 파일에 보존한다. 원본 도로의 기하 연속성이 검증됐다는 사실과 vendor가 모든 중첩 경로를 안전하게 선택한다는 주장은 구분한다. 외부 보고 발송 또는 vendor 바이너리 수정은 하지 않았다.


배치 후 가드를 추가한 guarded2에서도256개 주행 뒤 다음64개 배치에서 같은 native 함수가 종료됐다. 당시55개는 이동하지 않은0.28초 RDB 상태만 남았고9개는 주행 전 삭제됐다. 그 배치에는 원본 road418/-1 폭5.93mm,420/+1 폭3.47mm,2816/-2 폭2.19mm가 있었다. 이미 TrackPos를 보낸 뒤의 삭제는 native 경로 초기화를 피하지 못한다. 이에 다음 계획에서는 미완료1,681개 중 원본 폭1.852m 미만86개를 **TrackPos 이전**에 보류했다. 나머지1,595개만 재개하되, 보류86개 및 두64개 격리 배치의 배치·정지 좌표 자체는 정상 커버리지로 계산하지 않는다. 이후 단독 주행으로 새 증거가 있는 cell은 그 새 관측으로 계산한다. 이 폭 조건은 해당 임시 차량의 배치 가능 조건이며, 모든 lanelet의 합법성 판정 기준이 아니다. 폭이 충분했던6024나845 사례가 있으므로 극소 폭이 모든 crash의 원인이라고 결론 내리지 않는다.

resumed의6개 containment miss,90프레임을 원본 source에 독립 투영한 결과 모두 driving 영역 밖으로 확인됐다. 최대 지도 경계 거리는1.5065m이고 기록된 다른 임시차량은 최소12.467m였다. 마지막 crash 후보2789→2783은 별도 상태다. 최초 배치는 원본2789의 요청 XY와 정확히 같으며, 동시에2783/-2와101.45°로 교차하는 중첩점이었다. 마지막 raw 위치는2783/-3 driving 경계 **안쪽15.76mm**, 원본 폭3.00095m였으나 차량 헤딩은 해당 주행 방향과138.82° 어긋났다. 최근접 다른 임시차량은32.253m였다. 마지막 native path 재계산 로그와 일치하는 문맥이지만 이 actor가 null 참조를 직접 발생시켰다고 확정하지는 않는다.


현재 합집합 재검사는 `docs/assets/hdmap-audit/driving/drive-final-combined-replay/summary.json`과 `misses.json`에 보존한다. 같은 결과를 하나의 오프라인 실행으로 재현하려면 아래 저장 디렉터리를 모두 전달한다. 여러 실행에서 같은 cell이 반복 관측돼도 커버 수는 집합으로 한 번만 센다.

```bash
python3 map/tools/drive_map_audit.py --map map/hdmap.bin \
  --output /tmp/hdmap-all-replay --replay \
  docs/assets/hdmap-audit/driving/drive-global \
  docs/assets/hdmap-audit/driving/drive-midpoint \
  docs/assets/hdmap-audit/driving/drive-midpoint-resumed \
  docs/assets/hdmap-audit/driving/drive-isolated \
  docs/assets/hdmap-audit/driving/drive-explicit-pilot \
  docs/assets/hdmap-audit/driving/drive-explicit-global \
  docs/assets/hdmap-audit/driving/drive-uncovered \
  docs/assets/hdmap-audit/driving/drive-uncovered-resumed \
  docs/assets/hdmap-audit/driving/drive-uncovered-guarded \
  docs/assets/hdmap-audit/driving/drive-uncovered-guarded2 \
  docs/assets/hdmap-audit/driving/drive-uncovered-guarded3 \
  docs/assets/hdmap-audit/driving/drive-quarantined-isolated
```


## 격리 구간의 마지막 단독 검증

격리·부분중단 타깃 중 원본 폭이1.852m 이상이고 계획한 cell에서 정상 방향 주행이 전혀 확인되지 않았던38개를 추렸다. 이미 원본 중첩 재선택과 종료가 지목된845 동일 조건1개를 제외한37개를 새 NPC 한 대씩 검사했다. 37개 모두 처리했고32개 sampled_motion,4개 배치 불일치로 주행 전 삭제,1개 일반 containment miss였다. 실제 합산 이동1,004.363m, native crash는0이었다. 최종 정상 방향 cell이155개 추가돼82,381개가 됐다. 이 시험의 raw에는 각 차량의 실제 center_offset도 보존했다.

마지막 cleanup은 `driving-cleanup-isolated-final.json`이다. MapAudit NPC0개를 관측했고 Ego 원 snapshot과 마지막3개 RDB XYZ/HPR/속도 오차가 모두0이었다. 마지막 검증 후 시뮬레이터는 정지 상태로 반환했다.

방향필터의 독립 검토에서는 최종cell 중 polygon entry→exit와 대조 가능한93,806개에서180° 부호반전이 없었다(음의source차로60,998개, 양의차로32,808개). 곡선 cell chord와 중간점 접선 차이는 최대10.77°였으며, odd346개와 tiny2개는 이 독립 chord 검사에서 제외했다. 기존9개 방향 충돌 타깃 중 source road/lane이 같은 실제 연속661구간은 원본 s 진행부호로도 모두 역방향이었다. 양의차로 경계는 builder가 이미 진행방향으로 뒤집으므로 tangent를 다시 반전하지 않는 구현이 맞다.


최종 미검증11,773개 cell의 남은 구간 분류는 다음과 같다. cell 개수와 원본 주행선 길이를 함께 기록하며, 이 길이는 도로 양방향·여러 차로를 합친 lane 길이다.

| 남은 구간 | cell | 길이 |
| --- | ---: | ---: |
| 부분 주행 lanelet의 앞 구간 | 485 | 0.482 lane-km |
| 부분 주행 lanelet의 중간 구간 | 6,499 | 6.476 lane-km |
| 부분 주행 lanelet의 뒤 구간 | 4,485 | 3.997 lane-km |
| 전혀 정상 방향 이동을 확인하지 못한16개 lanelet | 304 | 0.296 lane-km |


마지막 단독 miss4프레임은 실제 보고 도로1940/1946의 원본 driving 바깥0.02358~0.17478m로 독립 확인했다. 또한 이37개 실행의 전체5,596개 저장 기록에서 실제 RDB center_offset이 모두 `(1.284500002861023, 0, 0)`이며 치수도 별도 모델 측정과 정확히 일치했다. 이전 raw에 없던 offset을 이전 기록에서 직접 관측했다고 바꾸지는 않으며, 동형 모델 및 후속 직접 관측의 근거를 구분한다.

첫 추가 실행의 큰 이탈 또는 driving 보고가 있던24타깃도 원본 driving 밖 또는 source s 끝밖이었다. 최악6.0106m의 이탈을 포함한다. guarded3에서 driving만 보고된 target95633의5프레임은 횡방향 경계 밖, target119704의128프레임은 source s 끝밖이었다. 이 원본 투영 사례에서 원본 driving 내부에 HD map polygon이 빠진 표본은 발견하지 못했다. native의 원본 경계 이탈을 보상하려고 보도·주차 영역까지 lanelet을 늘리지 않았다.
