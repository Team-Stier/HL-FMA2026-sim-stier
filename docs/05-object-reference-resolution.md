# 객체 ID별 기준점 → Bounding Box 변환

## 적용 계약

Bridge는 `config/runtime.yaml`의 `sim_bridge.object_offsets_file`이 가리키는
`config/object_offsets.yaml`을 시작할 때 읽는다. 파일 경로는 runtime.yaml의 디렉터리 기준이다.
**API의 object ID → 측정된 reference-to-center 오프셋**으로 조회하며, 슬롯 번호·차종 ID·크기로 오프셋을 추측하지 않는다. Ego 제원과 무관하다.

```yaml
3:
    size_m: [4.240, 1.796, 1.302]
    center_offset_m: [1.232, 0.0, 0.0]
```

이 예시는 측정 세션에서 ID 3이었던 BMW Z4다. `center_offset_m`은 객체 로컬 XYZ 변위다.
`size_m`은 같은 ID가 다른 크기의 객체로 재사용됐는지 확인하는 값이지 오프셋 선택 키가 아니다.
API 크기와 축별 0.001m를 초과해 다르거나 ID가 없으면 **그 패킷의 세 토픽을 발행하지 않고 오류를 기록**한다.
미지원 객체만 빼서 빈 공간처럼 보이게 하거나 공통값·0으로 대체하지 않는다. 기존 입력 만료 처리는 그대로 적용된다.

현재 탑재한 ID는 실험에서 직접 확인한 **2(Ioniq6), 3(BMW Z4), 4(Smart), 5(재생성 Ioniq6)**다.
이는 아래 재현 시나리오의 매핑이지 모든 대회 시나리오의 공통 ID표가 아니다.
ID는 생성 순서·시나리오에 따라 달라질 수 있으므로 실제 실행 시나리오가 바뀌면 새로 수집해 교체하고 Bridge를 재시작한다.
같은 ID·같은 크기지만 오프셋이 다른 모델로 교체되는 경우는 축약 API만으로 탐지할 수 없다.

## 실제 시나리오 매핑 생성

대상 시나리오를 실행한 개발용 VTD에서 **Bridge와 다른 참가자 API 클라이언트를 중지한 뒤** 다음 명령을 사용한다. 실측 중 두 클라이언트의 `9910` 동시 연결은 기존 연결의 종료/재연결을 일으켰으므로 동시에 실행하지 않는다. `--observe`는 NPC를 추가하거나 시나리오를 시작/정지하지 않고, **이름에 관계없이** 현재 RDB와 참가자 API를 읽는다.

```bash
python3 config/tools/measure_object_offsets.py --observe --seconds 30 \
    --output /tmp/scenario-objects.json \
    --export-offsets /tmp/scenario-object-offsets.yaml
```

관측한 API ID와 동일 ID의 RDB reference/heading/크기를 대조한 뒤 `geo.off`를 내보낸다.
위 출력 파일을 검토하고 `config/object_offsets.yaml`을 교체하거나 `object_offsets_file`로 지정한다.
API에서 관측했는데 RDB 대응을 찾지 못한 ID, 수집 중 geometry가 바뀐 ID, 서로 충돌하는 ID 매핑은 오류로 끝내며 기존 출력 파일을 덮어쓰지 않는다.
한 프레임 최대 30개이고 아직 등장하지 않은 객체는 수집되지 않는다. 필요한 구간을 실행해서 관측 범위를 확보한다.
여러 구간을 모으되 **동일 시나리오의 동일 ID 할당**에 속하는 보고서만 합친다.

```bash
python3 config/tools/measure_object_offsets.py \
    --from-report /tmp/section-a.json /tmp/section-b.json \
    --export-offsets /tmp/scenario-object-offsets.yaml
```

RDB는 이 개발용 매핑 생성 도구만 사용한다. 대회 Bridge는 저장된 매핑과 기존 참가자 API만 사용한다.

## 2026-09-05 실측과 반영

로컬 VTD 2025.2 / `00_HL_VTD`에서 LivingLab 시나리오에 임시 NPC 세 대를 넣고 실행했다. 임시 시나리오 초기 실험 후, VTD 프로세스를 다시 시작하고 SCP로 NPC를 생성해 재측정했다. 원본 RDB TCP `48190`과 참가자 API TCP `9910`을 동시에 읽었다. 설치본·플러그인·원본 시나리오 파일은 수정하지 않았다. 이 RDB 접속은 **개발용 측정만** 사용하며 대회 런타임 Bridge에 추가하지 않는다.

| NPC 모델 | API 길이 / 폭 / 높이 [m] | RDB reference → geometry center [m] |
|---|---|---|
| `HyundaiIoniq6_23_White` | 4.848 / 1.886 / 1.507 | **(1.384, 0, 0)** |
| `BMW_Z4_2010_grey` | 4.240 / 1.796 / 1.302 | **(1.232, 0, 0)** |
| `SmartForTwo_14_WhiteBlack` | 3.519 / 1.652 / 1.543 | **(1.2845, 0, 0)** |

처음 넣었던 Ioniq6 공통값은 제거했다. 현재는 위 세 차종과 재생성 객체에 각각 **ID별로 다른 측정값을 적용**한다. 임의 대회 시나리오 전체를 검증한 것은 아니므로 `config/vehicle.yaml`의 `object_reference_offsets_verified`와 주행·제동 보정 플래그는 false를 유지한다.

세 축 모두 SDK `RDB_GEOMETRY_t.offX/offY/offZ`를 읽은 값이다. 특히 **offZ=0은 임시값이 아니라 관측값**이며 높이 절반으로 덮어쓰지 않았다. 현재 `/objects` 계약은 yaw-only RDB box의 XY 중심·Z 하단이므로 Ioniq6의 발행 Z는 `API.z - 0.7535`다. 이것이 그래픽 모델 차체/타이어의 실제 최저점을 측정했다는 뜻은 아니다. 경사로의 물리 OBB와 full 3D min-Z도 검증하지 않았다.

API의 XYZ와 heading을 float32로 변환한 RDB reference/heading과 대조했다. 값이 일치하는 관측을 찾았으며 API가 이미 center를 보내고 있다는 가정은 배제했다. API에 시뮬 frame/time이 없으므로 이를 exact timestamp 동기화라고 부르지 않는다. 일반 실행·Ego 대비 90도 회전·삭제 후 재생성(180도 회전)에서 Ioniq6 오프셋은 같았다. 같은 이름/모델을 다시 생성했지만 객체 ID는 **2 → 5**로 바뀌었다. 이전 ID를 모델의 영구 식별자로 쓰지 않는다.

재시작 후 실측 증거와 소스 해시는 [object-offset-measurement.json](object-offset-measurement.json)에 저장한다. 각 3초의 일반·회전·재생성 측정에서 API 패킷은 각각 61/60/61개였고 각 NPC의 모든 XYZ/heading 관측에 대응하는 RDB 값이 있었다. 마지막 재현 실험은 NPC를 Ego 상대 위치에 고정한 정지 상태에서 수행했다. 주행/제동 실험은 아니며 제어용 9바이트 참가자 패킷은 송신하지 않았다. 임시 NPC를 삭제하고 원래 LivingLab 시나리오를 Stop 상태로 남겼다.

```bash
# 로컬 VTD에서 LivingLab 초기화를 완료한 뒤 실행한다.
# OffsetNPC0/1/2를 임시 생성하고 종료 시 삭제·Stop한다.
python3 config/tools/measure_object_offsets.py --seconds 3 --rotate \
    --output /tmp/vtd-offset.json --export-offsets /tmp/reproduced-object-offsets.yaml

# VTD 없이 저장된 실측 데이터와 현재 config/Bridge 계산을 검사한다.
python3 config/tools/measure_object_offsets.py --check docs/object-offset-measurement.json
```

재현 스크립트는 VTD 프로세스를 설치/시작/초기화하지 않는다. `00_HL_VTD`의 LivingLab 초기화 완료와 `Ego` 존재가 전제이며 `OffsetNPC0/1/2`라는 기존 actor가 없는 개발 세션에서만 실행한다. SCP `48179`, RDB `48190`, 참가자 API `9910`에 접속한다. `--check`는 시뮬레이터에 연결하거나 시나리오를 변경하지 않는다.

`--check`는 현재 설치 전 설정과 실제 Bridge 변환 함수를 사용한다. 세 차종 × 세 단계의 9개 관측에 대해 중심/하단 좌표와 총 72개 bbox 꼭짓점을 RDB yaw-only box와 비교한다(오차 0.0001m 미만). 객체 슬롯 순서를 뒤집어도 ID로 선택하고, 미등록 ID·크기가 바뀐 ID는 거부하며, 비어 있는 패킷·ID 0·XYZ 모두 0이 아닌 오프셋도 검사한다. 이는 물리 그래픽 모델이나 경사로의 full 3D OBB를 검증하는 검사는 아니다.

ID별 구현 후 `interfaces`·`sim_bridge` 빌드를 통과했고, 설치된 Bridge를 실제 VTD에 연결해 `/objects` 126개를 구독했다. 정상·90도 회전·삭제/재생성 단계의 세 차종 9개 관측 및 72개 꼭짓점이 RDB 기반 기대값과 0.0001m 미만 오차로 일치했다. 배열 30개·`length=3`·`frame_id=map`도 확인했다. API 동시 접속 충돌을 피하려고 각 단계에서 **RDB/API 참조 수집 → 연결 종료 → 동일한 정지 NPC 상태에서 Bridge 실행** 순서로 검사했다. 원시 패킷 재생이 아니라 실제 VTD 연결 검사이지만 exact-time 동기화 검사는 아니다. 새로 내보낸 ID표도 탑재된 YAML과 일치했다. 검사 후 NPC를 제거하고 Stop했다.

## 확인한 사실

- SDK `viRDBIcd.h`의 RDB_OBJECT_STATE_BASE_t.pos는 객체 reference point다. RDB_GEOMETRY_t는 dimX/Y/Z와 offX/Y/Z를 따로 제공한다.
- 배포 플러그인은 RDB offX/Y/Z를 내부에 보관하지만 참가자 객체 패킷에는 담지 않는다. 현재 API의 XYZ/LWH만으로 일반 객체의 중심을 유일하게 복원할 수 없다.
- 설치 Vehicles 디렉터리와 별도 배포 Vehicles 디렉터리에서 DistFront/DistRear/DistLeft/DistRight를 갖춘 VehicleDef 55건을 확인했다. 중복 파일/모델을 포함한 정의 수이며 55종의 대회 객체라는 뜻은 아니다.
- 이 정의로 모델의 로컬 XY 중심을 계산할 수 있다. offX=(DistFront-DistRear)/2, offY=(DistLeft-DistRight)/2. 길이는 front+rear, 폭은 left+right다. DistHeight만으로 일반적인 offZ를 확정하지 않으며 실제 RDB geometry와 대조해야 한다.
- 예: BMWZ4_10.xml은 front=3.352/rear=0.888, 따라서 L=4.240m/offX=1.232m. Ioniq6는 front=3.808/rear=1.040, 따라서 L=4.848m/offX=1.384m.
- 제공 HL_FMA_VTD_LivingLab.xml의 Player는 Ego 하나이고 ObjectList/PulkTraffic 등에는 일반 참가자 객체 모델 매핑에 쓸 배치가 없다. 따라서 현재 파일만으로 실제 런타임 30개 객체의 ID→모델을 모두 확정할 수 없다.

## 권장 해결 순서

1. 최종 대회 시나리오와 resolve된 모델 정의를 받으면, actor 식별자→모델→reference offset을 정적 매핑한다. API 런타임 ID와 시나리오 이름의 동일성을 가정하지 않고 확인한다. 기준점/스케일 변경이 없는지 포함해 검증하고 파일 해시로 버전을 고정한다.
2. 개발 환경에서 원본 RDB 대조가 허용되는 경우 id/name/geo.dim/geo.off와 참가자 API를 대조한다. 위 세 모델에 대해서는 수행했으며 대회 런타임에 별도 RDB 구독을 추가하는 설계는 아니다. 다른 모델·스케일링까지 검증 범위를 확대할 때도 초기화/respawn의 ID 의미를 함께 검사한다.
3. 위 매핑을 고정할 수 없는 동적 모델까지 허용한다면 참가자 API 제공처가 오프셋을 노출하거나, 이미 플러그인 내부에 있는 offset을 적용해 객체 XYZ를 box 중심으로 송신하도록 계약을 바꾸는 것이 정확한 해결이다. 후자는 패킷 크기는 유지할 수 있지만 XYZ 의미가 바뀌므로 버전 구분과 양측 동의가 필수다. 승인 없이 배포 바이너리를 패치하지 않는다.
4. L/W/H 비교는 후보 축소에만 쓴다. 동일 크기 모델/스케일링/보행자/트레일러 때문에 모델 확정 근거가 되지 않는다. 검증된 유한 후보 집합이 있는 경우에만 각 후보 box의 합집합을 보수적 footprint로 사용할 수 있다. 후보 집합의 완전성도 모르면 임의 중심/오프셋을 정답으로 만들지 않고 미지원 관측으로 처리한다.

## 좌표 변환

NPC 검증용 시나리오는 최종 대회 시나리오와 별도로 만든다. 이번에는 확인된 세 모델의 NPC를 추가하고 참가자 API와 원본 RDB id/name/geo.dim/geo.off를 대조했다. geometry offset 확인 자체에는 차량 주행이 필수가 아니지만 시뮬레이션에서 RDB 상태가 생성되어야 한다. 참가자 축약 API만 켜서 주행하는 것으로 누락된 offX/Y/Z가 새로 관측되는 것은 아니다.

정확한 자세가 있으면 center_map=reference_map+R*offset_local이다. 참가자 객체 패킷은 heading만 주므로 일반 3D pitch/roll까지 복원하지는 못한다. 지면 평면 가정이 검증된 구간에서는 다음 XY 변환을 쓴다.

```text
center_x = ref_x + cos(heading)*offX - sin(heading)*offY
center_y = ref_y + sin(heading)*offX + cos(heading)*offY
```

그 중심 기준 로컬 (+/-L/2, +/-W/2) 네 모서리를 heading으로 회전시켜 cell polygon과 교차한다. 경사/다층 도로에서 z를 height/2로 임의 이동하지 않는다.

## 소비자의 Bounding Box

발행된 `(x, y)`는 중심, `z`는 하단이다. 로컬 XY 네 모서리 `(±size_x/2, ±size_y/2)`를 heading으로 회전해 `(x, y)`에 더하고 높이는 `[z, z + size_z]`를 사용한다. 소비자는 ID 매핑이나 reference 오프셋을 다시 적용하지 않는다. RViz `CUBE`의 pose는 하단이 아니라 중심이므로 `pose.position.z = z + size_z/2`, scale은 원본 size, orientation은 heading의 quaternion으로 설정한다.
