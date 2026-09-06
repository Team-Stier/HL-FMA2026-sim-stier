# LivingLab 전 도로 이미지 감사 — 2026-09-06

원본 XODR의 **651개 도로 전체**를 306개 구역으로 나누어 정밀 OSGB 원본과 현재 lanelet·cell·정지선·신호 관계 영상을 직접 비교했다. 도로 밖에 떨어진 원본 정지선 3개도 별도 1개 구역에서 확인했다. 이번 전수 검토에서 추가로 확정된 변환 오류는 없었다. 영상만으로 판단이 애매한 객체와 차로는 사용자의 지시에 따라 XODR의 존재·유형·좌표를 보존했다.

[전체 영상과 구역별 판정 열기](assets/hdmap-audit/full-vision/index.html) · [검토 목록 JSON](assets/hdmap-audit/full-vision/review-summary.json)

## 실제 검토 범위

| 항목 | 범위 |
|---|---:|
| XODR road | 651 / 651 |
| 도로 기준선 길이 합계 | 33,244.6005m |
| driving이 있는 road / 없는 road | 643 / 8 |
| 원본 표본 | 39,242개, 최대 간격 1m 및 geometry·laneSection·width 변경점 |
| 도로 영상 | 306개 구역, 원본·중첩 612장, 비교 페이지 153장 |
| 도로 밖 보충 영상 | 1개 구역, 원본·중첩 2장, 비교 페이지 1장 |
| 영상 포함 lanelet / cell | 2,845 / 94,154, 전체 |
| 영상 포함 정지선 | 751 / 751 (도로 영상 748 + 보충 3) |
| 영상 범위의 물리 등화 | 646 / 646 |
| 고유 물리 신호–정지선 관계 | 1,811 / 1,811 |

33.245km는 XODR 기준선 길이 합계로 교차로 연결 road를 포함한다. 실차 주행 거리나 서로 중복되지 않는 도로 길이가 아니다. 1,817개 규칙별 물리 관계 발생 건수에는 동일 관계 중복이 있으므로 고유 관계 1,811개와 구별했다.

한 구역은 80m 코어와 사방 10m 여유를 포함한 100m 영상이다. 정밀 OSGB를 0.078125m/px로 렌더링하고 각 원본/중첩 그림을 1280×1280으로 보존했다. 네 검토자가 서로 겹치지 않는 페이지를 직접 읽고 관찰·판정·XODR 키·확대 열람 파일을 기록했다. 축소 화면에서 불명확한 지점은 원본 크기로 재확인했다. 완료 집계는 이미지 생성 수가 아니라 실제 검토 기록과 페이지 목록을 대조한 결과다.

원본 도로 목록에서 영상 구역을 선정했으므로 변환된 lanelet이 있는 곳만 검사하는 방식이 아니다. 비주행 road 1251, 1253, 1260, 1272, 4741, 4744, 6028, 6029도 포함했다. 1260, 6028, 6029는 laneSection이 없는 reference-only road다. 차도가 없는 공간에 임의로 lanelet/cell을 만들지 않았다. 남쪽 t-1_18/t-1_19는 road1909 주변 여유 영역을 담은 구역이다.

[원본 전체 목록](assets/hdmap-audit/full-vision/coverage-source.json)과 [native 전체 면적·객체 범위 검사](assets/hdmap-audit/full-vision/native-coverage-check.json)를 함께 보존했다. 각 cell이 구역에 한 번이라도 나타나는지만 세지 않고, 모든 lanelet·cell의 전체 polygon 면적이 영상 구역 합집합 밖으로 나가는지도 검사했다. 외부 면적은 수치 허용치 1e-9m²를 넘는 사례가 없었다.

## 판정과 유지한 항목

- **road146 objects409/410:** 사용자가 지적한 정지선은 XODR에 존재하므로 위치와 객체를 유지한다. 기존 ‘현장 배치 미확정’은 이제 편집 대기 사유가 아니다. 이는 원본 보존 결정이며 법적 시설 배치 적합성을 새로 증명한 것은 아니다. 정밀 배경에서 약 15.3m 앞의 crosswalk403이 나타나는 [LOD 수정 근거](HDMapSignalAudit.md#2026-09-06-우상단-정지선-배경-lod-누락과-미확정-배치)는 그대로 유효하다.
- **도로 밖 빨간 정지선:** 예를 들어 road2077 objects2913/2914, road2195 objects3204/3205, road1043 objects1215/1216은 XODR의 s=0과 큰 횡방향 t 값에 대응한다. 원본 객체를 보존하되 관계없는 차량 cell에 붙이지 않는다. 미할당은 별도 상태로 남는다.
- **도로 영상 밖의 객체:** road2819 objects5960/5961/5962는 t≈171.7m에 거의 겹쳐 있다. 보충 구역 t29_26에서 건물/장면 가장자리 부지의 도색 부재와 native 붉은 객체를 확인했다. XODR에 존재하므로 세 객체 모두 유지한다.
- **영상에 흰 정지 도색이 불분명한 곳:** road76 objects114–117 등은 원본 존재와 좌표를 직접 대조해 유지한다. 전체 710개 XODR 정지선이 각각 정확히 한 번 보존되어 있으며 누락·source metadata 불일치가 없다.
- **중앙 사선지대와 차로 축소부:** t01_19/t04_13 등은 확대하면 사선지대 내부에 cell이 없고 양쪽 차로 경계가 둘러간다. 축소 중첩만으로 판단했던 침범 의심을 해소했다.
- **긴 보라 신호 관계:** 개별 등화의 실제 위치 및 controller 소속과 비교한다. 선 길이만으로 인접 정지선에 재연결하지 않는다. controller1의 원본 positionRoad/등화 소속 모순은 원본 보존으로 처리하며, 존재하지 않는 controller80/82/84/86을 생성하지 않는다.

[710개 정지선별 XODR 보존 검사와 정책](assets/hdmap-audit/full-vision/xodr-policy-review.json)에는 미할당 원본 26개, controller 없는 원본 318개, 기존 근거로 수정한 5개 및 OSGB 도색에서 보강한 41개가 구분되어 있다. 이번 지시는 애매한 대상을 임의 편집하지 않는 기준이며, 이미 근거가 확정된 변환기 수정과 보강을 되돌리는 기준으로 적용하지 않았다. 추가 지도 바이너리 변경은 없다.

## 영상 생성기의 전역 확인

이전 화면에서 횡단보도·정지 도색이 가려졌던 원인은 고공 카메라의 거친 LOD 선택이었다. 수정된 렌더러의 LODScale=0.001을 모든 이번 영상에 사용했다. OSGB의 LOD 노드 1,000개를 조사해 근거리 모델 선택 누락이 없음을 확인했다. 기본 설정과 비교해 870개 노드의 선택이 달라졌다.

선택되지 않은 56개는 먼 거리용 주택 52개, 유효하지 않은 거리 범위의 office 3개, XY 면적이 0인 연석 조각 1개였다. 이 검사에서 주행 노면·도색의 추가 선택 누락은 발견하지 못했다. [전체 LOD 조사](assets/hdmap-audit/full-vision/lod-scope.json)

지붕·나무·입체 구조물의 실제 가림은 항공 영상만으로 해소되지 않는다. 그런 곳도 원본 XODR 차로 유형·객체·연결 정보를 기준으로 유지했으며, 영상에서 가려진 면을 직접 보았다고 집계하지 않는다.

## 재현 및 근거

- XODR SHA256: 5a369c7b0609fc98b0034680473465db4273dd396ffb37980996cea70ec6ca2f
- OSGB SHA256: 2d8f83fdc26cb7f8373e71cf850c70f7d90981362789fb474eb31b0d5b253296
- HDMap SHA256: e7a426f900813748747a39233f3678237fd7f849c335bae3bc3014b31ae678b0
- 운영 항공 배경 SHA256: 94a2873b95b0418ca3edd1dadc13d2f8885aad09a66c1e093e40c71309171662

[영상 생성 도구](../map/tools/prepare_vision_audit.py), [구역 목록 생성](assets/hdmap-audit/full-vision/inventory_source.py), [검토 누락 검사·색인 생성](assets/hdmap-audit/full-vision/finalize_review.py), [보존 이미지 체크섬](assets/hdmap-audit/full-vision/image-checksums.json)을 제공한다. 큰 중간 블록 PNG는 재생성 가능한 작업 산출물이다. 직접 열람한 비교 페이지 154장과 이에 대응하는 1280px 원본·중첩 그림 614장, 총 768개를 저장했다.

최종 집계 검사에서 306개 구역의 검토자·페이지 대응과 651개 road 포함 여부가 모두 통과했다. 페이지 오기·새 오류 판정·검토 누락을 넣은 세 부정 사례는 결과 저장 전에 거부됐다. [집계 검증](assets/hdmap-audit/full-vision/finalizer-negative-checks.json) · [도로/구역 검색 검증](assets/hdmap-audit/full-vision/index-checks.json). 코드의 AST 지식 그래프 갱신과 git diff --check도 통과했다.

```bash
source /opt/ros/jazzy/setup.bash
/tmp/hdmap-build-env/bin/python docs/assets/hdmap-audit/full-vision/inventory_source.py --output /tmp/new-vision-coverage.json
/tmp/hdmap-build-env/bin/python map/tools/prepare_vision_audit.py --coverage /tmp/new-vision-coverage.json --xodr '/home/stier/vtd 자료/HL_FMA_VTD_LivingLab.xodr' --osgb '/home/stier/vtd 자료/HL_FMA_VTD_LivingLab.osgb' --renderer /tmp/hdmap-render --output /tmp/new-vision-atlas
```

생성 도구는 새 영상을 pending으로 기록한다. 실제 영상 재검토 없이 이전 판정을 새 지도에 자동 이월하지 않는다. 현재 보존된 검토 기록의 원래 /tmp 경로는 열람 당시의 경로이며, 같은 이름의 이미지가 영구 보존 atlas/supplemental 디렉터리에 있고 체크섬으로 식별된다.

이 후속 작업은 전 도로 정적 이미지 감사다. 새 주행·제동·전체 신호 주기 검증으로 집계하지 않는다. 기존 [주행 감사](HDMapDrivingAudit.md)와 [신호 API·상태 계약의 남은 문제](HDMapSignalAudit.md)는 유지되며, 원본 기준 보존 결정으로 release_ready=false를 해제하지 않았다.
