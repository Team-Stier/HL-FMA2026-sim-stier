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
`config/runtime.yaml`의 `sim_bridge.object_center_offset_m`은 **세 축 모두
API reference point → box center로 이동시키는 오프셋**이다. 객체 로컬 축
기준 m 단위(+x 전방, +y 좌측, +z 위)이며, z 설정값도 중심까지의 변위다.
Bridge는 XY 오프셋을 heading으로 회전해 더하고,
`z = ref_z + offset_z - size_z / 2`로 하단 좌표를 발행한다.
API에 pitch/roll이 없어 yaw만 사용한다. 현재 공통 임시 오프셋 `(0, 0, 0)`은
미보정값이며, XY는 원본 좌표, Z는 원본에서 높이 절반을 뺀 값이다.
Tracker 등 소비자는 오프셋을 다시 적용하지 않는다. 설정 변경 후 노드를 재시작한다.

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
비어 있지 않은 객체 목록과 제어 송신·주행·제동의 실제 검증은 남아 있다.
