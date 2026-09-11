# VTD 신호 강제 변경 진단 (읽기 전용)

## 확인된 계약

설치된 VTD 공식 SCP 문서 `/home/stier/VIRES/VTD.2025.2/Doc/SCP_HTML/docu.html:406`–412는 다음 XML을 그대로 지원한다. SetPhase/SetCtrl의 처리 모듈은 TR(Traffic)이며 phase=stop, syncJunction=false, fadeTime=0, freeze=true가 유효하다.

```xml
<TrafficLight><SetPhase ctrlId="136" phase="stop" syncJunction="false" fadeTime="0"/><SetCtrl ctrlId="136" freeze="true" syncJunction="false"/></TrafficLight>
```

SCP header receiver=`any`는 설치된 공식 예제 `/home/stier/VIRES/VTD.2025.2/Develop/Communication/SCPClientSample/ExampleConsole.cpp:249`에서 사용하는 값이다. payload는 뒤에 NUL을 붙일 필요가 없다. 따라서 XML 철자나 `any`가 문서에 없는 값이라서 실패한 것은 아니다.

원상복구 문법은 `<TrafficLight><SetCtrl ctrlId="136" freeze="false" syncJunction="false"/></TrafficLight>`이다. 이 진단에서는 어느 명령도 전송하지 않았다.

## 실제 프로그램과 플러그인

실행 중인 시나리오 `/home/stier/VIRES/VTD.2025.2/Data/Projects/SampleProject/Scenarios/HL_FMA_VTD_LivingLab.xml`의 SignalController Id=136은 Delay=0, go=15초, attention=3초, stop=18초다. 주기 36초로, 자연 적색은 시뮬레이션 시간 [18,36), [54,72), ... 이다.

`/tmp/taskRec_ModuleManager.txt`의 `[TL_RDB] controllerId=136`은 rawState=3 at0/36/72, rawState=5 at15/51/87, rawState=1 at18/54/90을 보여 이 프로그램과 일치한다. HLVTD는 raw RDB 신호를 읽고 green 코드를 3/5 중 테이블 값으로 변환하는 구조다. 플러그인이 별도 시간 주기로 적색 명령을 덮어쓰는 근거는 찾지 못했다. 설치 hl_vtd_config.json에도 신호 주기 설정은 없다.

## 실패 지점에 대한 추가 근거

- actions.jsonl의 두 적색 변경 시도는 각각 Start 명령에서 0.116ms, 0.117ms 뒤에 전송됐다. 정상 주행·RDB 수신이 확인된 후 별도로 보낸 시험은 아직 없다.
- 실제 ghostdriver 바이너리의 `SCP_Interpreter::handleGroupTrafficLight` (symbol 0x1de5ac)를 읽었다. ctrlId로 controller를 찾은 후에는 `+++ SCP: <TrafficLight><SetPhase> ctrl=...`/SetCtrl 로그를 출력하고 flush한다. SetPhase는 `OT_LightSignController::jumpToPhase`, SetCtrl은 `modify`를 호출한다.
- 유효 ctrlId 파싱 후 controller lookup이 nullptr이면 **로그 없이 다음 command로 넘어간다** (0x1de6d8–0x1de6ed 및0x1dea75–0x1dea8a). 따라서 로그가 없다는 것만으로 전달 자체가 없었다고 단정할 수 없다. 전달/초기화 시점/lookup 실패를 구분해야 한다.
- `/tmp/taskRec_Traffic.txt` 전체에서 SetPhase/SetCtrl 로그는 없고 이후 차량 Set/Speed 로그는 정상적으로 있다.

## 다음으로 검증할 최소 시험

이미 동작 중이고 ego 및 raw RDB 신호 수신이 확인된 시점에 위 **같은 공식 XML**을 별도 SCP 메시지로 보내고, Traffic의 SetPhase/SetCtrl 로그와 ModuleManager `[TL_RDB] controllerId=136 rawState=1`을 함께 확인한다. 명령 적용을 입증하기 전에는 ROS 제동 동작 성공/실패 판정용 강제 적색으로 간주하지 않는다.

이것은 문법 검증을 마친 시험안이며, 적용 성공을 검증한 수정 명령이라고 주장할 수 없다. 초기화 시점이 실패 원인인지는 위 시험으로 결정된다. override 없이도 자연 cycle의 red 구간에 접근시켜 실제 적색 정지를 시험할 수 있다.

TaskControl 직접 Mask(문서413)는 물리 lamp별 표시를 바꾸는 별도 명령이다. controller raw RDB 상태와 HLVTD API 적색까지 바뀐다는 보장이 없어 이 주행 진단에는 대체안으로 권하지 않는다.

## 후속 시험 상태

parent는 추가 override 시험 대신 출발을15초 늦춰 controller136 자연 적색 정지를 시험했다. 정지선 전 정지는 관측됐으나 invalid-path 및 최대 조향으로 lane34407에서34191로 바뀐 뒤 녹색 재출발이 되지 않았다. 이 결과는 phase override 성공 여부와 무관한 실제 관측 신호 시험이며 control 진단에서 상세 판단한다. 추가 simulator 명령은 수행하지 않았다.
