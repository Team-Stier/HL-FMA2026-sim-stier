# VTD 교육 영상 문서 인덱스

이 디렉터리는 2026 HL FMA Cadence & iVH 시뮬레이션 대회 교육 녹화 2편을 화면과 음성 전체 구간에 걸쳐 정리한 문서다. 본문은 실제 작업 절차와 운영 주의점을 재구성하고, 전사 부록은 자동 전사의 모든 큐와 타임스탬프를 보존한다.

## 문서 바로가기

- [원시 데이터 → ROS 토픽 → RViz 마커 파이프라인](DataPipeline.md)
- [VTD 배포본 검토 및 ROS 설계 근거](04-vtd-design-review.md)
- [객체 API 기준점 해결 조사](05-object-reference-resolution.md)
- [VTD 시뮬레이터 설치·실행·사용 가이드](03-vtd-simulator-install-run-guide.md)

| 구분 | 본문 | 전체 자동 전사 |
|---|---|---|
| 세션 1 · VTD/ROD Road Designer | [01-vtd-road-designer.md](01-vtd-road-designer.md) | [transcripts/session-1.md](transcripts/session-1.md) |
| 세션 2 · Scenario Editor/제어기 연동 | [02-vtd-scenario-editor-integration.md](02-vtd-scenario-editor-integration.md) | [transcripts/session-2.md](transcripts/session-2.md) |

## 원본과 검증 범위

| 항목 | 세션 1 | 세션 2 |
|---|---:|---:|
| 원본 길이 | 01:05:18.8 | 02:09:11.75 |
| 영상 | H.264, 1,920×1,080, 약 16 fps | H.264, 1,920×1,080, 16 fps |
| 음성 | AAC mono, 16 kHz | AAC mono, 16 kHz |
| MP4 크기 | 181,063,841 bytes | 757,897,788 bytes |
| 전사 큐 | 1,768개 | 3,309개 |
| 전사 마지막 시각 | 01:05:01.040 | 02:09:06.930 |

- 배포 ZIP 크기는 910,420,346 bytes이며, 다운로드 뒤 전체 압축 무결성 검사를 통과했다.
- 자동 전사는 Whisper `large-v3-turbo-q5_0`을 CPU와 VAD로 처리했다. 고유명사·메뉴명·숫자·문장 경계에는 오류가 있을 수 있어 본문 작성 때 영상 화면과 전사 맥락을 함께 대조했다.
- 본문에 사용한 그림은 모두 녹화 영상에서 추출한 실제 프레임이다. AI 생성 시각자료는 없으며, 기술 UI만 남기도록 단순 crop했다.
- 참가자 패널, 실명·팀명·연락처, 내부 URL, 계정·비밀번호 등 기술 이해에 불필요한 정보는 이미지 crop 또는 최소 텍스트 치환으로 비식별화했다. 전화번호가 포함된 일정표 프레임은 수록하지 않았다.

## 문서 읽는 법

1. 실습·대회 준비에는 각 세션의 본문을 먼저 본다.
2. 특정 발언의 원문과 정확한 시각이 필요하면 같은 세션의 전사 부록에서 타임스탬프를 찾는다.
3. 본문의 **확인 필요** 표시는 화면·음성만으로 정확한 철자나 최종 배포값을 확정할 수 없음을 뜻한다. 실제 설치본, 배포 매뉴얼, 대회 공지를 우선한다.
4. 화면에 나온 편집 중 값과 시나리오 버전은 교육 과정의 중간 상태일 수 있다. 저장·제출·최종 평가 경로 완료를 뜻하지 않는다.

## 영상에 나온 설치 환경

교육 영상의 제어기 연동 부분은 **Ubuntu 24.04 LTS, x86-64, NVIDIA GPU/드라이버, VTD 2025.2** 환경을 전제로 설명한다.

## 아직 배포본에서 확인할 항목

- VTD 실행·Setup 선택 명령의 정확한 철자와 옵션
- 설치 경로와 디렉터리 대소문자, 최종 XML/XODR/OSGB 파일명
- 제어기 연동 Setup과 JSON 파일의 최종 이름·경로
- 대회장에서 사용할 고정 IP와 실제 연결 방향/방화벽 정책
- ROD Track 연결 UI의 `Outer Side` 판독과 3D Viewer 스케일 키
- IONIQ 리소스 항목 수와 대회 배포 차량 모델
- 카메라·LiDAR의 정량 사양과 최종 송신 주기
