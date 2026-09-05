# HDMap 코어

C++17 header-only 라이브러리다. 각 헤더 상단에 선언, 하단에 inline 정의를 둔다.

Cell은 작은 Lanelet처럼 사용한다. `id()`, `polygon2d()`, `polygon3d()`와 camelCase 명명으로 맞춘다. polygon2d는 원본 Point3d 데이터를 공유하는 읽기 전용 XY view이며 polygon3d는 저장된 읽기 전용 polygon 참조다. Lanelet의 CompoundPolygon과 Cell의 ConstPolygon은 구성 방식만 다르고 Lanelet2 기하 함수에 그대로 전달할 수 있다. 경계선/중심선은 polygon만으로 임의 생성하지 않는다.

Cell 고유 API는 `parent()`, `stoplineIds()`, `isStopline()`, `previous()`, 초기화용 `setPrevious()`다. `boundingBox3d()`는 Lanelet2 함수로 계산한 캐시를 반환한다. box 조회는 LaneletLayer처럼 `search(box)`로 호출하며 Cell ID 목록을 반환한다. BoundingBox2d 입력은 높이를 구분하지 않고, BoundingBox3d 입력은 높이 범위까지 사용한다. `queryOverlaps()`도 Cell ID 목록을 반환한다.

- `cell.hpp`: 사전 ID·부모 lanelet·polygon·bounding box·정지선 목록·이전 Cell 포인터.
- `cell_tree.hpp`: Boost R-tree bulk-load, box 후보 조회, 실제 polygon 교차, 동기 debug callback.
- `hdmap.hpp`: Lanelet2 IO와 `hdmap_init`, 프로세스별 지도·Cell·트리·정지선 역색인·신호 레지스트리 소유.

`signalRegistry()`는 API controller ID를 key로 Lanelet2 TrafficLight의 const shared_ptr 목록을 반환한다. 지도에서 같은 controller_id를 가진 신호 규칙들을 묶으며 허용 신호 판단은 하지 않는다. 물리 신호와 정지선은 TrafficLight의 trafficLights()/stopLine()을 그대로 사용한다. 데이터 속성과 ID 계약은 루트 README를 따른다.

```cpp
#include "hdmap/hdmap.hpp"

auto map = hdmap::hdmap_init("/path/to/hdmap.bin");
const auto& cell = map->cells().at(cell_id);
const auto candidates = map->cellTree().search(cell.boundingBox3d());
const auto previous = cell.previous();
```

`hdmap.bin`은 Lanelet2 바이너리 지도이며 polygonLayer에 사전 제작한 Cell들을 포함한다. 속성 계약은 루트 README의 정적 지도 항목을 따른다. 런치 때 분할/ID 재생성은 하지 않는다. R-tree의 내부 노드는 Cell bounding box에서 메모리에 bulk-load한다. 별도 R-tree 바이너리를 읽는 것은 아니다.

HdMap은 복사/이동할 수 없다. 반환된 unique_ptr을 노드 수명 동안 보관한다. Cell 저장소 배치와 트리 구성이 끝난 다음 previous를 연결하므로 이후 포인터가 바뀌지 않는다. same-lane previous는 index-1이며 다른 lanelet 연결은 사전 previous_cell_id가 있을 때만 사용한다. 합류점 분기는 아직 단일 previous로 표현하지 않는다.

Cell ID로 `cells().at(id)`와 dynamic 배열을 함께 조회한다. lanelet은 `laneletMap().laneletLayer.get(id)`로 직접 사용한다. HdMap 소유 데이터를 외부에서 변경하지 않는다. 신호 규칙은 Tracker의 향후 책임이며 여기에 포함하지 않는다.

## 빌드 및 소비

Lanelet2 core/io, Boost, Eigen 개발 환경이 필요하다. Python API는 기본 빌드하며 Python 개발 헤더와 같은 Python 버전의 Boost.Python이 추가로 필요하다. 실행 시 공식 `lanelet2` Python 패키지가 필요하다. C++만 빌드하려면 `-DHDMAP_BUILD_PYTHON=OFF`를 지정한다.

```bash
cmake -S src/hdmap -B /tmp/hdmap-build -DCMAKE_INSTALL_PREFIX=/path/to/install
cmake --build /tmp/hdmap-build
cmake --install /tmp/hdmap-build
```

```cmake
find_package(hdmap_core REQUIRED)
target_link_libraries(my_node PRIVATE hdmap::core)
```

header-only 코어의 실제 코드 검사는 소비자 translation unit이나 Python 확장을 컴파일해야 한다. TF·Visualizer 노드와 Python ROS marker adapter는 각각 `src/tf_broadcasting`, `src/visualization`에 구현했다. C++ ROS marker adapter는 미구현이다.

## Python

```bash
export PYTHONPATH="/path/to/install/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
python3 src/hdmap/python/check_api.py
```

설치 경로의 Python 버전은 빌드에 사용한 버전과 맞춘다. ROS 환경과 Lanelet2 Python 환경도 먼저 불러온다. Boost.Python과 Lanelet2는 같은 Python ABI를 사용해야 한다.

```python
from hdmap import hdmap_init
from lanelet2.core import BasicPoint2d, BoundingBox2d

static_map = hdmap_init("/path/to/hdmap.bin")
cells = static_map.cells()
cell_tree = static_map.cellTree()
box = BoundingBox2d(BasicPoint2d(0, 0), BasicPoint2d(10, 5))
cell_ids = cell_tree.search(box)
cell = cells[cell_ids[0]] if cell_ids else None
if cell is not None:
    print(cell.id, cell.parent().lanelet_id, cell.polygon2d())
```

- `cell.id`는 공식 Lanelet2 Python과 같은 읽기 전용 property다. 다른 Cell 조회 메서드와 HdMap 메서드는 C++ 이름을 유지한다.
- `hdmap_init(path, projector=None, debug_sink=None)`를 제공한다. `.osm`은 공식 Lanelet2 projector를 전달한다.
- `search`는 native BoundingBox2d/3d를 받는다. `queryOverlaps(footprint, min_z, max_z)`는 native polygon 또는 x/y property가 있는 Lanelet2 점들의 iterable을 받는다. 결과는 Cell ID의 list다.
- `laneletMap()`은 native LaneletMap, `signalRegistry()`는 controller ID → native TrafficLight list의 dict다. 신호의 `stopLine`·`trafficLights`는 공식 Python API대로 property다.
- `cells()`는 Cell view의 list를 만든다. 초기화 때 한 번 받아 재사용한다. `stoplineCells()`와 `signalRegistry()`도 Python 컨테이너 복사본이므로 로드 후 보관한다.
- Cell/트리 view가 HdMap의 수명을 유지하므로 Python의 지도 변수만 삭제해도 포인터가 무효화되지 않는다. `previous()`의 끝은 None이다. 로드 후 구조 변경용 생성자나 setPrevious는 노출하지 않는다.
- native Lanelet2 지도·신호 객체는 원본을 공유한다. Python의 native API 자체는 변경 메서드를 제공하므로 수정하면 Cell/R-tree와 어긋날 수 있다. 로드한 정적 지도·신호·좌표는 읽기 전용으로 취급한다.
- debug_sink는 `(operation, cells)`로 동기 호출된다. GIL은 유지하며 콜백 예외는 호출자에게 전달한다. callback 안에서 다시 같은 트리를 조회하거나 HdMap/Cell을 callback 자신에게 영구 보관하지 않는다. 전자는 재귀, 후자는 소유권 순환을 만들 수 있다. 로그를 저장할 때 ID·좌표 값만 복사한다.

```python
def trace(operation, cells):
    print(operation, [cell.id for cell in cells])

static_map = hdmap_init("/path/to/hdmap.bin", debug_sink=trace)
```

check_api.py는 임시 지도를 직접 생성해 바이너리/OSM 로드, native 자료형, 교차·높이 필터, 신호/정지선, previous 수명과 callback 예외 전달을 검사한다. 마커 발행 adapter는 포함하지 않는다.
