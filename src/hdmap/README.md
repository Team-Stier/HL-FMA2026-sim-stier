# Cell 코어

`include/hdmap/cell.hpp` 상단에 선언, 하단에 inline 정의를 둔다. 별도 cpp는 없다. C++17과 lanelet2_core를 사용한다.

Cell은 ID, 부모 lanelet ID와 0부터의 순서, ConstPolygon3d, 계산된 BoundingBox3d, 정지선 ID 목록, 이전 cell의 비소유 포인터를 담는다. geometry 점은 부모 lanelet 경계를 잘라 만든 Point3d다. 로드 후 공유 점을 수정하지 않는다.

소비자용 메서드는 id/parent/geometry/bounding_box/stopline_ids/is_stopline/previous다. 초기화 때 set_previous로 이전 cell을 연결한다. 생성자는 받은 데이터를 저장하고 bounding box만 계산한다. 카탈로그·형상·ID 검증 및 예외 처리는 현재 넣지 않는다.

previous는 주행 방향 기준 바로 앞서 통과하는 상류 cell이며 기본값은 nullptr다. 모든 cell의 메모리 배치를 마친 다음 각 프로세스가 자기 메모리의 주소로 연결한다. 연결 후 vector 재할당·삽입·삭제·정렬·Cell 교체를 하지 않으며, 컨테이너를 복사했다면 포인터를 다시 연결한다. 원본 cell 저장소가 포인터 사용 기간 내내 살아 있어야 한다. 주소를 파일/ROS 메시지로 저장하지 않는다.

기본 로드 순서는 cell 저장소·R-tree 로드 → previous 연결 → 소비자 조회 시작이다. previous는 R-tree 내부 엔트리가 아니라 Cell 저장소의 주소를 가리킨다. R-tree를 먼저 만드는 것보다 중요한 조건은 연결 시점부터 Cell 주소가 더 이상 바뀌지 않는다는 점이다.

같은 lanelet에서는 index-1 cell을 연결한다. lanelet 경계를 넘는 이전 cell은 로더가 도로 연결 관계로 명시적으로 선택한다. 이전 가지가 여러 개인 합류점은 포인터 하나로 모든 상류를 표현하지 못하므로 임의로 하나를 고르지 않는다. 그런 경우 이전 연결을 비워 두고 후속 분기 처리 설계에서 다룬다. 신호 감속장 순회는 필요한 거리까지만 수행하며 loop 도로 전체를 끝없이 순회하지 않는다. 동적 speed cap은 계속 별도 배열에 기록한다.

`make_stopline_cell_index(cells)`는 시작 시 정지선 ID → cell ID 목록을 생성한다. 중복 제거·정렬·ID 재할당은 하지 않는다. 이 순서와 형상은 사전 제작한 데이터 그대로 사용한다.

신호 매핑 및 허용 판단은 Tracker의 향후 구현 책임이다. hdmap에 신호 정책 코드는 두지 않는다. R-tree/교차 함수/Python binding은 아직 미구현이다. C++와 Python 호출 예시는 루트 README를 따른다.

## 빌드

```bash
cmake -S src/hdmap -B /tmp/hdmap-core-build
cmake --build /tmp/hdmap-core-build
```

header-only이므로 소비자 translation unit을 컴파일해야 코드가 검사된다. CMake 소비자는 hdmap::core를 사용한다. 개발용 Lanelet2 1.2.2 소스와 Boost/Eigen은 임시 경로 `/tmp/vtd-lanelet2-deps`에 있으며 이 환경에서는 `-Dlanelet2_core_DIR=/tmp/vtd-lanelet2-deps/build`를 사용할 수 있다. 배포 의존성은 향후 run.sh에서 설치한다.
