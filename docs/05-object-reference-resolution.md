# 객체 API 기준점: 조사 결과와 해결 절차

## 확인한 사실

- SDK `viRDBIcd.h`의 RDB_OBJECT_STATE_BASE_t.pos는 객체 reference point다. RDB_GEOMETRY_t는 dimX/Y/Z와 offX/Y/Z를 따로 제공한다.
- 배포 플러그인은 RDB offX/Y/Z를 내부에 보관하지만 참가자 객체 패킷에는 담지 않는다. 현재 API의 XYZ/LWH만으로 일반 객체의 중심을 유일하게 복원할 수 없다.
- 설치 Vehicles 디렉터리와 별도 배포 Vehicles 디렉터리에서 DistFront/DistRear/DistLeft/DistRight를 갖춘 VehicleDef 55건을 확인했다. 중복 파일/모델을 포함한 정의 수이며 55종의 대회 객체라는 뜻은 아니다.
- 이 정의로 모델의 로컬 XY 중심을 계산할 수 있다. offX=(DistFront-DistRear)/2, offY=(DistLeft-DistRight)/2. 길이는 front+rear, 폭은 left+right다. DistHeight만으로 일반적인 offZ를 확정하지 않으며 실제 RDB geometry와 대조해야 한다.
- 예: BMWZ4_10.xml은 front=3.352/rear=0.888, 따라서 L=4.240m/offX=1.232m. Ioniq6는 front=3.808/rear=1.040, 따라서 L=4.848m/offX=1.384m.
- 제공 HL_FMA_VTD_LivingLab.xml의 Player는 Ego 하나이고 ObjectList/PulkTraffic 등에는 일반 참가자 객체 모델 매핑에 쓸 배치가 없다. 따라서 현재 파일만으로 실제 런타임 30개 객체의 ID→모델을 모두 확정할 수 없다.

## 권장 해결 순서

1. 최종 대회 시나리오와 resolve된 모델 정의를 받으면, actor 식별자→모델→reference offset을 정적 매핑한다. API 런타임 ID와 시나리오 이름의 동일성을 가정하지 않고 확인한다. 기준점/스케일 변경이 없는지 포함해 검증하고 파일 해시로 버전을 고정한다.
2. 개발 환경에서 원본 RDB 대조가 허용되는 경우 한 번의 캘리브레이션에서 id/name/geo.dim/geo.off와 참가자 API를 대조한다. 이 작업은 현재 수행하지 않았고 대회 런타임에 별도 RDB 구독을 추가하는 설계도 아니다. 초기화/respawn 때 ID 의미가 유지되는지 함께 검사한다.
3. 위 매핑을 고정할 수 없는 동적 모델까지 허용한다면 참가자 API 제공처가 오프셋을 노출하거나, 이미 플러그인 내부에 있는 offset을 적용해 객체 XYZ를 box 중심으로 송신하도록 계약을 바꾸는 것이 정확한 해결이다. 후자는 패킷 크기는 유지할 수 있지만 XYZ 의미가 바뀌므로 버전 구분과 양측 동의가 필수다. 승인 없이 배포 바이너리를 패치하지 않는다.
4. L/W/H 비교는 후보 축소에만 쓴다. 동일 크기 모델/스케일링/보행자/트레일러 때문에 모델 확정 근거가 되지 않는다. 검증된 유한 후보 집합이 있는 경우에만 각 후보 box의 합집합을 보수적 footprint로 사용할 수 있다. 후보 집합의 완전성도 모르면 임의 중심/오프셋을 정답으로 만들지 않고 미지원 관측으로 처리한다.

## 좌표 변환

NPC 검증용 시나리오는 최종 대회 시나리오와 별도로 만들 수 있다. 기존 시나리오를 복사하고 확인된 모델의 NPC 한 대를 정지 상태로 배치한 뒤 참가자 API와 원본 RDB id/name/geo.dim/geo.off를 대조한다. 그 다음 회전·주행·재시작·respawn·모델 교체 순으로 확인 범위를 넓힌다. geometry offset 확인 자체에는 차량 주행이 필수가 아니며 정지 상태의 RDB만으로도 확인할 수 있다. 단, 참가자 축약 API만 켜서 주행하는 것으로 누락된 offX/Y/Z가 새로 관측되지는 않는다. 이 개발용 검증이 허용되는 환경에서 수행하며 현재 NPC 시나리오 생성/실행은 아직 하지 않았다.

정확한 자세가 있으면 center_map=reference_map+R*offset_local이다. 참가자 객체 패킷은 heading만 주므로 일반 3D pitch/roll까지 복원하지는 못한다. 지면 평면 가정이 검증된 구간에서는 다음 XY 변환을 쓴다.

```text
center_x = ref_x + cos(heading)*offX - sin(heading)*offY
center_y = ref_y + sin(heading)*offX + cos(heading)*offY
```

그 중심 기준 로컬 (+/-L/2, +/-W/2) 네 모서리를 heading으로 회전시켜 cell polygon과 교차한다. 경사/다층 도로에서 z를 height/2로 임의 이동하지 않는다.

## 결론

현재 선택은 정확한 Ego 오프셋을 사용하고 일반 객체는 검증된 모델 매핑을 확장하는 것이다. 모든 대회 객체에 대한 매핑 완료를 선언할 자료는 아직 없다. 완전한 동적 객체 지원이 필요하면 원본 geometry offset을 보존하는 API 계약이 최선이며, 단순히 size를 reference point에 더하는 것으로 해결되지 않는다.
