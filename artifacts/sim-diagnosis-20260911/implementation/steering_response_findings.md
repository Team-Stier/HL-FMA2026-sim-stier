# 조향 전달과 실제 yaw 응답: 읽기 전용 진단

검사 기록: `/tmp/sim_fix_20260911/events.jsonl`. 북쪽은 재배치 이후 실제 주행한 1789079834.0–1789079842.69, 남쪽은 1789079994.0–1789079999.94. 빈 Path로 인한 제동 전 구간만 사용했다. 소스·설정·노드·시뮬레이터를 변경하지 않았다.

## 확인한 결과

**상수 조향 단위 오류보다, 속도가 높아질수록 실제 yaw 응답이 MPC의 자전거 모델 예측보다 작아지는 현상이 뚜렷하다.** 비교식은 `yaw_rate / (v*tan(command.steering)/2.944)`다. 명령 생성 stamp와 Ego capture stamp를 사용하고 명령은 zero-order hold로 정렬했다. yaw는 unwrap 후 0.2초 중심 차분, 동일 시간 창으로 예측 yaw도 평균했다. v>2m/s, |예측yaw|>.02rad/s에서 0–.5초 명령 지연을 탐색했다.

- 최적 관측 지연은 두 구간 모두 약 70ms. 이는 전체 command→packet→sim→pose 경로의 관측 지연이며 actuator 지연만을 분리한 값은 아니다.
- 전체 LS gain: 북 .8481, 남 .8320. 보정 후 yaw RMSE 각각 .00484/.00684rad/s.
- 미분 창을 .4초로 바꾸어도 gain .8507/.8344, 최적 지연 70/60ms로 유지된다.

| 속도 m/s | 북쪽 gain | 남쪽 gain |
|---|---:|---:|
| 2–4 | 0.998 | 0.990 |
| 4–6 | 0.930 | 0.916 |
| 6–7 | 0.869 | 0.822 |
| 7–8 | 0.829 | 0.832 |
| 8–9 | — | 0.806 |

저속은 거의 1이며 7–8m/s는 약 .83, 8–9m/s는 약 .81이다. rad↔degree 또는 16.8 steering-ratio 누락이라면 이런 크기와 속도 의존성을 설명하기 어렵다. 두 구간을 설명하는 단순 관측식 `gain≈1/(1+K*v²)`의 K는 .00367/.00362, yaw RMSE .00241/.00315rad/s다. **이 수치는 관측 적합치이며 검증된 모델 파라미터나 즉시 적용할 상수로 제안하지 않는다.**

현재 MPC `src/control/src/controller_core.cpp:333,344`는 `b1=v*dt/wheelbase`로 yaw를 계산한다. 7–8m/s에서 같은 조향 입력으로 생기는 yaw를 약 20% 크게 예측하는 셈이다. 이때 경로가 매번 현재 차체 heading에서 새로 시작하여 추종 오차를 지우면, 조향 부족이 누적되어 바깥쪽으로 흐를 수 있다. 경계 접촉의 단독 원인이라고 확정하지는 않는다; planner의 현재 자세/곡률 연결과 복귀 후보 비용을 함께 검토해야 한다.

## 단위 및 기준점 확인

- `src/interfaces/msg/ControlCommand.msg:4`: steering은 road-wheel radians.
- `src/sim_bridge/sim_bridge_node.py:73–79,194`: `<ffB>`에 steering 값을 그대로 전달. gain, sign, degree 변환 없음. `:258–266`에서 약 .05초 간격으로 최신 명령 송신.
- 설치 `libHLVTD.so.1.0.0`의 공개 함수 `ControlApplier::buildDriverControl`(0x26680–0x2678d)은 steering을 그대로 또는 범위 clamp만 적용해 반환한다. `Module::iVHEvalPlugin::applyControl`(0x21947–0x21959)은 이를 RDB driver-control offset0x1c에 복사한다. multiplication/division이나 ratio 변환 없음. 근거 disassembly는 `hlvtd_control_applier_disassembly.txt`.
- 설치 공식 API `.../Develop/Communication/VtdFramework/VtdToolkit/include/VtdToolkit/viRDBIcd.h:2028`: offset0x1c의 `steeringTgt`는 desired wheel angle in rad. `:1168` target-steering validity는0x40이며 플러그인 mask0x40e0에 포함. 따라서 전달 계약은 일치한다.
- 설치 `.../Data/Distros/Distro/Config/Players/Vehicles/HyundaiIoniq6_23.xml:167`: WheelBase2.944, DistFront3.808, DistRear1.04, halfwidth.943, MaxSteering.48. `config/vehicle.yaml:9–24` 및 `src/control/config/control.yaml:29–31`과 일치. SteeringRatio16.8은 steering-wheel/road-wheel 구분에 쓰일 설정이지만 현재 steeringTgt 경로에 다시 곱할 근거가 없다.
- 설치 `.../Data/Setups/00_HL_VTD/Config/HLVTD/hl_vtd_config.json:10–21`: rear_axle_center, 동일 bodygeometry. Bridge `:209–215`는 Ego x/y/heading을 변환 없이 발행한다.
- Ego 좌표의 차체 횡방향 속도/관측 yaw로 고정 기준점 offset을 추정하면 저속 약 .21m에서 7–8m/s 약 .45–.47m로 증가한다. 일정한 box center offset1.384m와 맞지 않는다. 이 지표에는 실제 tire sideslip과 상태 시각 정렬 오차가 섞이므로 .4m 기준점 오류라고 해석하면 안 된다. 설정 불일치는 확인되지 않았다.
- 같은 창으로 계산한 XY 속도와 발행 speed의 중앙 비율은 북1.0048/남1.0106으로 큰 speed-unit 오류도 없다.

## 남은 불확실성

로그에는 실제 front-wheel angle, yaw-rate 센서, simulator origin stamp가 없다. 따라서 VTD steering actuator 응답, tire slip/understeer, 각 피드백 지연을 완전히 분리할 수 없다. 설정에 WheelSkewStiffness12와 무게2095kg 등이 있으나 VTD 내부 모델 공식을 확인하지 못했으므로 그것을 직접 원인으로 단정하지 않는다. 다음 검증은 단위/gain을 임의로 바꾸기보다, 기존 RDB vehicle-systems의 실제 front-wheel steering(`viRDBIcd.h:1756`)을 별도 관찰하고 안정된 곡률·여러 속도에서 commanded/actual wheel angle과 yaw를 나누어 비교하는 것이다. 현재 기록만으로도 **속도 의존 kinematic-model mismatch**는 재현된다.

재현 스크립트: `analyze_steering_response.py`. 수치: `steering_response.json`, 미분 창 민감도: `steering_response_window400ms.json`.
