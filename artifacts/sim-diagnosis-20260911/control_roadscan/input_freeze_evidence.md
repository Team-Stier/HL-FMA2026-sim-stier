# 지속 stale-ego: 시험 중 프로세스 정지와 연관된 수신 단절

관측 시각은 `/tmp/sim_diagnosis_extended/actions.jsonl`, `/tmp/sim_diagnosis_extended/events.jsonl` 기준이다. 저장소 소스나 DDS 설정은 수정하지 않았다.

- MPC PID142382를 재사용하던 sweep에서 `north_uphill_road2819`의 SIGCONT가 t1789073918.354에 기록되었고, t3918.356부터 `FORWARD_STALE_EGO`가 지속됐다. 다음 `west_eastbound_intersection_road76`에서도 같은 상태였다.
- 다른 subscriber인 observer는 현재 Ego를 source stamp 이후 약1ms 안에 받았다. `/ego_status` publisher는 tracker1개이고 controller subscription과 QoS가 BEST_EFFORT/VOLATILE로 일치했다. Controller는 현재시각의 status/command를 계속20Hz 발행했지만, 다른 노드가 보는 cap0·속도40~69m/s와 무관하게 조향0·가속+2m/s²를 계속 보냈다.
- `/mpc_control`의 parameter get 서비스 요청도 수십 초 응답하지 않았다. 단순 Ego 값의 finite 검증 문제보다 수신경로 전체 문제를 의심할 근거다.
- 수신 단절 중 `/proc/142382/maps`를 읽었을 때 다음 mapping을 확인했다: `... 00:1d 770047 /dev/shm/fastrtps_port7415 (deleted)`.
- 다음 case를 resume한 t1789074228.420에 `/dev/shm/fastrtps_port7415`가 inode805936으로 재생성되었다. stat mtime은2026-09-11 06:03:48.420114121+0900이었다. Controller와 tracker `/proc/maps`가 모두 새 inode805936을 가리켰다. PID142382는 바뀌지 않았다.
- 그 직후 상태가 `STOP_NO_LOCAL_PATH`(t4228.508), `ACTIVE`(t4230.758)로 복구됐다.
- 설치된 FastDDS 공식 헤더 `/opt/ros/jazzy/include/fastrtps/fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h:49`는 기본 port health-check timeout을1000ms로 선언한다. 이것만으로 현재 프로세스가 기본값을 그대로 사용했다고 단정하지는 않는다.

판정: SIGSTOP/CONT를 통한 시험 제어와 공유메모리 port 삭제/재생성에 연관된 수신 단절을 직접 확인했다. 특정 DDS 내부 호출 경로까지 증명한 것은 아니다. 이 두 case의 장시간 직진 폭주는 정상적인 경로 추종 결과와 분리해야 한다. 재시험은 동일 원본 binary/config의 새 controller 프로세스로 수행한다.

이와 별개로 입력이 끊겨도 계속 가속하는 fallback 자체는 실제 명령 및 `src/control/src/mpc_control_node.cpp:298-314,320-324`에서 확정된다. 입력 중단을 일으킨 계측 조작과 이를 차량 가속으로 바꾼 제어 정책을 구분한다.
