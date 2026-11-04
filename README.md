# SolidMesh（static2）

C++20 静态体网格数据结构库，支持 TET、HEX、PYRAMID 混合单元。
一次性构建拓扑，随后只读访问坐标、元素和邻接关系。没有局部增删或坐标修改接口。
此前动态网格的设计讨论保存在 [docs/dynamic-design.md](docs/dynamic-design.md)，
该文档仅作背景资料，当前静态版本以本 README 和公开头文件为准。

## 构建与使用

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
```

在父项目中使用：

```cmake
add_subdirectory(SolidMesh)
target_link_libraries(my_app PRIVATE SolidMesh::core)
```

```cpp
#include <solidmesh/mesh/mesh.h>

using namespace SolidMesh;

std::vector<Vector3> vertices = {
    {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}
};
std::vector<std::vector<IndexType>> cells = {{0, 1, 2, 3}};
std::vector<CellType> types = {CellType::TET};

Mesh mesh = Mesh::from_data(vertices, cells, types);
const Vertex& v = mesh.vertex(0);
auto neighbors = mesh.vertex_vertices(0);
auto outgoing = mesh.vertex_dedges(0);
for (IndexType dei : outgoing)
{
    const DirectedEdge& de = mesh.dedge(dei);
    auto ordered_cells = mesh.dedge_cells_ccw(dei);
    // de.vertices[0] == 0，de.vertices[1] 是相邻顶点。
}
```

`Mesh{}` 是合法空网格；`std::vector<Mesh> meshes(n)` 创建 n 个空网格。
可以复制、移动或整体赋值 Mesh。复制深拷贝存储并保留 ID；移动转移存储。
不承诺移动后源对象的具体内容，它可以销毁或重新赋值。
元素引用借用 Mesh 内部存储，不应跨越所属对象的赋值、移动或销毁使用。
邻接查询返回独立的 `std::vector<IndexType>`。

## 输入和编号

`Mesh::from_data(vertices, cells, celltypes)` 不修改输入，成功返回完整 Mesh。
顶点和单元 ID 保持输入数组顺序；不同种类的元素分别使用从零开始的连续编号。
`SolidMesh::IndexType` 为 `std::uint32_t`，其最大值保留为 `invalid_id`。
调用方负责使用正确 Mesh、正确元素种类的 ID。

- `cells.size()` 必须等于 `celltypes.size()`。
- 每个单元的顶点数必须匹配类型，顶点 ID 必须有效且互不重复。
- 保留孤立顶点和不相交的连通分量；空输入合法。
- 通过相同顶点 ID 表达连接；坐标相同不会自动焊接。
- 单元的局部顶点编号遵循 `CellOrderTemplate`，面环从单元外侧看为逆时针。

参考坐标：

- TET：`(0,0,0), (1,0,0), (0,1,0), (0,0,1)`。
- HEX：底层 `0,1,2,3` 为 `(0,0,0), (1,0,0), (1,1,0), (0,1,0)`；
  上层 `4,5,6,7` 为对应点加 `(0,0,1)`。
- PYRAMID：底面 `0,1,2,3` 同 HEX，顶点 `4` 可取 `(.5,.5,1)`。

构建检查输入数组长度、单元类型、顶点数和索引范围，并在面配对或边邻域排序
无法完成时抛出带说明的 `std::runtime_error`。分配失败正常传播标准库异常。
不创建完整的网格验证流程；调用方负责输入的共形性、流形性和正确编号。
不验证顶点处的完整流形性、重复单元、坐标有限性、体积/Jacobian、退化、
四边面平面性、几何自交或单元重叠，也不自动修复方向。
构建成功不代表通过了这些检查。

## 元素和查询

元素访问 `vertex/dedge/edge/halfface/face/cell` 返回 `const &`，要求 ID 有效。
调试构建使用断言；发布构建中违反此要求属于未定义行为。
所有邻接查询对无效 ID 返回空 vector。

| 查询 | 含义 |
| --- | --- |
| `vertex_dedges(vi)` | 以 vi 为起点的所有出向有向边 |
| `vertex_vertices(vi)` | 通过实际网格边相连的顶点，去重且不含自身 |
| `cell_cells(ci)` | 共享面的相邻单元，去重且不含自身和边界占位项 |
| `dedge_cells_ccw(dei)` | 围绕有向边的单元序列，从终点望向起点时为 CCW |

前三种查询不承诺几何排序。同属一个 HEX 但没有边相连的两个顶点不算顶点邻居；
仅共享边或顶点的两个单元不算 `cell_cells` 邻居。

`dedge_cells_ccw` 在构建时按面连接排序。内部边返回闭环的一次遍历，不重复首项，
以最小 Cell ID 为首项；边界边返回从一个边界端到另一个边界端的完整开链。
反转 dedge 会反转顺序，闭环仍以最小 Cell ID 开始。
每条边的关联单元必须能够组成单一环或链，否则构建报错。
CCW 的几何含义以合法的正向单元编号为前提，不按坐标角度重新排序。

存储约定：

- 每个有向顶点对只有一条 DirectedEdge；每条 Edge 对应两个相反方向的 dedge。
- `DirectedEdge::vertices[0]` 为起点，`vertices[1]` 为终点，`opposite` 指向反向 dedge。
- 每个 HalfFace 只属于一个 Cell。内部 Face 有两个相反的半面；边界 Face 只有一个，
  其 `halffaces[1]` 和半面的 `opposite` 为 `invalid_id`。不创建虚拟外部单元。
- `Cell::vertices/halffaces` 保持输入局部顶点序和模板面序；
  `HalfFace::dedges` 保存有序的有向边环。有效项在前，剩余位置为 `invalid_id`。
- 顶点到出向有向边、定向边到有序单元使用 CSR；offsets 使用 `std::size_t`。
  顶点邻居由出向边终点生成，不额外保存顶点到顶点 CSR。

## VTK IO

MeshIO 保留原始数据接口，支持 VTK Legacy ASCII Unstructured Grid。
`VtkRawData` 的坐标为 `vector<vector<double>>`，单元类型保留 VTK 整数编码。
读写均只保留 10（TET）、12（HEX）、14（PYRAMID），其他类型（包括 WEDGE）跳过。
附带的点/单元属性不读写。坐标写出使用 double 的往返精度。
IO 失败返回 `false`；读取失败时输出数据不变。

当前在调用方转换坐标及类型后，交给 Mesh 构建：

```cpp
#include <solidmesh/mesh/mesh_io.h>
#include <stdexcept>

using namespace SolidMesh;
MeshIO::VtkRawData raw;
if (!MeshIO::read_vtk("input.vtk", raw))
    throw std::runtime_error("Cannot read input.vtk");

std::vector<Vector3> points;
points.reserve(raw.vertices.size());
for (const auto& p : raw.vertices)
    points.push_back({p[0], p[1], p[2]});

std::vector<CellType> types;
types.reserve(raw.cell_types.size());
for (int type : raw.cell_types)
{
    switch (type)
    {
    case 10: types.push_back(CellType::TET); break;
    case 12: types.push_back(CellType::HEX); break;
    case 14: types.push_back(CellType::PYRAMID); break;
    default: throw std::runtime_error("Unsupported VTK cell type");
    }
}
Mesh mesh = Mesh::from_data(points, raw.cells, types);
```

`write_vtk(filename, raw)` 仍接受 `VtkRawData`，不增加 Mesh 写出重载。
过滤不支持的体单元可能使网格不完整，读取成功不保证剩余网格满足构建前提。
`assets/bpgc.vtk` 本身在顶点 144、148 之间存在两个不连通单元扇，
该边不涉及 WEDGE；因此读取可成功，但拓扑构建会报错。

## 回归测试

```sh
cmake -S . -B build -DSOLIDMESH_BUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
```

覆盖单种和混合单元、共享三角面/四边面、内部环和边界开链的方向、孤立顶点、
空网格、不自动焊接、非法 ID 查询、复制移动和 VTK 读写。
资产测试构建并检查五份合法网格，验证 WEDGE 过滤及 bpgc 的已知非流形边报错。
