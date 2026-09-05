# Sim Bridge (Python)

VTD 참가자 TCP 연결을 `/ego_pose`, `/objects`, `/traffic_light`로 발행하고
`/ctrl_cmd`를 `<ffB>`로 송신한다. TF와 `/clock`은 발행하지 않는다.

프로젝트 루트에서 전체 bringup:

```bash
./run.sh --check
./run.sh
```

`run.sh` → `scripts/bringup.py`가 `src/sim_bridge/launch.sh`를 자동으로 찾아
첫 번째로 실행한다. launch는 ROS 환경을 불러오고 루트의 `config/runtime.yaml`,
`config/vehicle.yaml`을 노드에 전달한다. 단독 실행 전에는 한 번 빌드한다:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select interfaces sim_bridge
./src/sim_bridge/launch.sh
```

기본 접속은 `127.0.0.1:9910`이다. 다른 VTD 호스트를 사용하려면
`VTD_HOST=192.168.0.10 VTD_PORT=9910 ./run.sh`처럼 지정한다.
launch에 추가 ROS 인자도 전달할 수 있다(`./src/sim_bridge/launch.sh -p host:=192.168.0.10`).
경로는 launch 파일 위치 기준이므로 다른 작업 디렉터리에서도 절대 경로로 실행할 수 있다.
브리지는 TCP 클라이언트이며 VTD 자체를 시작하지 않는다. VTD 설치·라이선스는
시뮬레이터 호스트에서 준비하고 시나리오를 실행한다.

`/objects`의 XY는 객체 box 중심, Z는 객체 하단(min Z)이다.
`config/runtime.yaml`의 `sim_bridge.object_offsets_file`로 지정한 파일을 읽고
**API object ID로 `center_offset_m`을 조회**한다. 기본은 같은 디렉터리의
`object_offsets.yaml`이다. 오프셋은 세 축 모두 reference → center의 로컬 변위다.
단위는 m이고 +x 전방, +y 좌측, +z 위다. `size_m`은 ID 재사용/크기 변경 검사에 쓴다.
Bridge는 XY 오프셋을 heading으로 회전해 더하고,
`z = ref_z + offset_z - size_z / 2`로 하단 좌표를 발행한다.
API에 pitch/roll이 없어 yaw만 사용한다. 탑재된 측정 세션의 ID 2/5는 Ioniq6의
`(1.384, 0, 0)`, ID 3은 BMW Z4의 `(1.232, 0, 0)`, ID 4는 Smart의
`(1.2845, 0, 0)`을 적용한다. 공통 오프셋은 없다. **다른 시나리오에서는 ID표를 새로 생성해야 한다.**
미등록 ID 또는 측정 크기와 축별 0.001m 초과 차이가 있으면 기존 오류 처리로
패킷 전체를 거부한다. 일부 객체만 누락한 `/objects`나 빈 목록으로 발행하지 않는다.
실측 `offZ=0`을 높이 절반으로 바꾸지 않는다. 현재 계약의 발행 Z는
`ref_z-size_z/2`이며, 그래픽 차체/타이어의 실제 최저점 측정값은 아니다.
실험·재생성 및 적용 범위는 [객체 기준점 조사](../../docs/05-object-reference-resolution.md)를 따른다.
Tracker 등 소비자는 오프셋을 다시 적용하지 않는다. 설정 변경 후 노드를 재시작한다.
RViz `CUBE`는 발행된 `(x, y, z + size_z/2)`를 pose 중심으로 쓰고,
heading quaternion과 `(size_x, size_y, size_z)` scale을 적용하면 된다.

실제 시나리오 실행 중 ID 매핑 수집(Bridge 등 다른 `9910` 클라이언트는 먼저 종료, 시나리오 변경 없이 관측만):

```bash
python3 config/tools/measure_object_offsets.py --observe --seconds 30 \
    --output /tmp/scenario-objects.json --export-offsets /tmp/scenario-object-offsets.yaml
```

출력을 해당 시나리오용 설정으로 지정한다. 개발용 수집 도구만 RDB를 읽으며 Bridge에는 RDB 연결이 없다.

현재 설정은 `allow_motion=false`, 제동 보정값 `null`이므로 **수신 전용**이다.
주행 허용에는 `allow_motion=true`, `calibration_verified=true`, 검증된 음수
`speed.emergency_target_acceleration_mps2`가 모두 필요하다. 가속도 0을 정지
명령으로 대신하지 않는다. 제한값·조향 비율 보정은 Controller 책임이다.

- 같은 완성 패킷의 세 토픽은 동일한 ROS 수신 stamp를 사용한다.
  QoS는 BEST_EFFORT / VOLATILE / KEEP_LAST(1)이다.
- 한 번에 수신한 완성 패킷 중 최신 것을 사용하고 뒤의 미완성 바이트는 보존한다.
  객체 슬롯 전체가 0인 경우만 제거하고 앞쪽으로 압축한다. ID 0만으로 제거하지
  않는다. 이 빈 슬롯 규칙은 실측 대조가 필요하다. NaN/Inf가 있는 패킷은 버린다.
- 제어 송신은 wall time 기준 최대 20Hz다. 입력 0.25초 또는 명령 0.20초 만료 시
  설정된 제동 요청을 보낸다. 명령 생성 stamp와 monotonic 수신 나이를 모두 검사한다.
  `base_link`가 아닌 제어 명령과 비유한 명령도 차단한다.
- 연결 실패/EOF/송신 오류 시 버퍼·명령을 폐기하고 1초 뒤 재접속한다.
  종료 시 연결과 제동 설정이 있으면 정지를 요청한다. 연결 단절 시 전달은 불가능하다.
- 런타임 설정 변경은 재시작으로 적용한다. 루트 `run.sh`의 프로세스 감시·종료 규칙을 따른다.

2026-09-05 로컬 VTD 2025.2 / `00_HL_VTD` / `HL_FMA_VTD_LivingLab.xml`에서
두 번 연결해 각각 5초 동안 101개 패킷을 받았다(각각 19.9996Hz, 20.0001Hz).
Ego XYZ=(508.7997, -168.2877, 42.0), controller ID=27, state=1→5를 관측했다.
객체는 0개였고 잔여 바이트·디코딩 오류는 없었다. 제어 패킷은 보내지 않았다.
플러그인 SHA-256은 설계 문서의 `6059d546…eca4e`와 일치했다.
검증 후 시나리오는 Stop으로 정지했다.

같은 날 Ubuntu 24.04에 ROS 2 Jazzy ROS Base와 colcon을 설치하고 위 두
패키지 빌드를 완료했다. 실제 VTD에 연결한 브리지의 ROS 토픽을 10초간 구독해
`/ego_pose` 200개, `/objects` 199개, `/traffic_light` 199개를 받았다.
동일 stamp의 세 메시지 묶음 199개, frame ID, 고정 배열 크기와 빈 슬롯,
BEST_EFFORT / VOLATILE QoS를 검사했다. KEEP_LAST(1)은 노드에서 설정하지만
Fast DDS discovery는 history/depth를 UNKNOWN/0으로 보고하므로 원격 조회로
그 두 값까지 검증했다고 보지 않는다.

검증용 브리지는 정상 종료했고 VTD 시나리오는 Stop 상태로 돌려놓았다.
이후 임시 NPC 세 대의 참가자 API와 원본 RDB를 동시 측정해 reference point와
geometry offset을 대조했다. 회전·삭제/재생성 결과와 Bridge 변환 검사는
[`docs/05-object-reference-resolution.md`](../../docs/05-object-reference-resolution.md)에 기록한다.
ID별 매핑 적용 후 실제 VTD에 연결해 `/objects` 126개와 세 차종의 72개 bbox
꼭짓점을 검사했다. 미등록 ID·크기 변경 거부와 매핑 내보내기 검사도 통과했다.
제어 송신·주행·제동 검증과 최종 대회 시나리오의 모든 ID 수집은 남아 있다.
