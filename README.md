# SolidMesh

C++17 混合体网格库。元素结构、Mesh 方法和强类型值句柄遵循 DynamicSolidMesh；底层继续使用原有 `EntityPool`，以便在统一的接口和拓扑表示下比较存储容器。

## 使用

```cmake
add_subdirectory(SolidMesh)
add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE SolidMesh::SolidMesh)
```

原有 `SolidMesh::core` 链接目标仍可使用。目标传递头文件目录与 C++17 要求，启用 PIC，不需要 DynamicSolidMesh、Eigen 或其他外部依赖。默认仅构建库。

```cpp
#include <solidmesh/mesh/mesh.h>

int main() {
    SolidMesh::Mesh mesh;
    auto a = mesh.add_isolated_vertex(0, 0, 0);
    auto b = mesh.add_isolated_vertex(1, 0, 0);
    auto c = mesh.add_isolated_vertex(0, 1, 0);
    auto d = mesh.add_isolated_vertex(0, 0, 1);
    auto cell = mesh.add_tet({a, b, c, d});
    if (cell.empty()) return 1;

    const auto* vertex = mesh.vertex(a);
    if (!vertex) return 2;
    return mesh.set_vertex_position(a, vertex->pos[0] + 1, 0, 0) ? 0 : 3;
}
```

独立构建、示例及测试：

```powershell
cmake -S SolidMesh -B build-solid -DCMAKE_BUILD_TYPE=Release -DSOLIDMESH_BUILD_EXAMPLE=ON -DSOLIDMESH_BUILD_TESTS=ON
cmake --build build-solid --parallel 8
ctest --test-dir build-solid --output-on-failure
```

多配置生成器使用 `--config Release`。示例无参数时创建两个共享面的 TET；可传入 `input.vtk [output.vtk]` 执行读写。

## 元素与身份

```cpp
struct Vertex { Vector3 pos; std::vector<DedgeHandle> outgoing; };
struct Dedge { VertexHandle v0, v1; std::vector<HalffaceHandle> incident_halffaces; };
struct Halfface {
    FaceType type;
    std::array<DedgeHandle, 4> dedges;
    CellHandle cell;
    HalffaceHandle opposite;
};
struct Cell { CellType type; std::array<HalffaceHandle, 6> halffaces; };
```

正式定义在 `mesh_element.h`，默认枚举值为 UNSET，默认句柄为空。坐标保留 SolidMesh 原有的 double `Vector3` 数学类型，`Real` 为 double；统一使用 `pos[0..2]` 访问，和 DynamicSolidMesh 的默认精度及调用方式一致。

四种句柄均为 `Handle<Tag, HandleSize>`，只保存一个 64 位整数，支持默认构造、`empty()`、`==`、`!=`。不同元素的句柄不能互相转换，不保存 Mesh 指针，也不提供元素方法或裸 ID 构造接口。Mesh 通过 `vertex/dedge/halfface/cell(handle)` 查找元素，无效或失效时返回 nullptr；返回的元素指针始终为 const。

`empty()` 只检查空值，删除前保存的句柄通常仍非空，但查找会失败。句柄的身份作用域是对应 Mesh；独立网格可能产生同值句柄，跨 Mesh 使用由调用方负责。这与 DynamicSolidMesh 的约定一致，并非全局唯一 ID。

`storage.h` 只适配 EntityPool 的协议：把内部 slot/generation 编码为整数并预留 0 为空值，保留其 dense vector、空闲槽复用、generation 检查和 swap-with-last 删除策略。`entity_pool.h` 未修改；没有暴露两套可混用的 Mesh ID 或 dense index。仍继承 EntityPool 的 32 位 slot/generation 范围。

## Mesh API

公共基准接口与 DynamicSolidMesh 使用相同名称和参数形式：

- 计数：`num_vertices()`、`num_dedges()`、`num_halffaces()`、`num_cells()`。
- 插入：`add_isolated_vertex(x,y,z)`、`add_tet(array<...,4>, order)`、`add_hex(array<...,8>, order)`、`add_pyramid(array<...,5>, order)`。
- 删除：`delete_cell(handle)`、`delete_isolated_vertex(handle)`。
- 遍历：`vertices()`、`dedges()`、`halffaces()`、`cells()`，返回句柄 range。
- 元素访问：`vertex(handle)`、`dedge(handle)`、`halfface(handle)`、`cell(handle)`。
- 坐标：`set_vertex_position(handle,x,y,z)`，返回 bool。
- 复制：拷贝构造、拷贝赋值、`copy()`；移动后源 Mesh 为空且可继续使用。
- 邻接：`vertex_vertices(VertexHandle)`、`edge_cells(DedgeHandle)`、`edge_cells_ccw(DedgeHandle)`、`cell_cells(CellHandle)`。

SolidMesh 额外保留 `add_prism(array<VertexHandle,6>, order)` 和 `CellType::PRISM`，沿用相同元素布局、句柄和编辑约定。公共三种单元的模板与 DynamicSolidMesh 的行顺序、环方向和顶点编号完全一致。

## 拓扑、模板与编辑约定

- `Dedge::v0` 为 source，v1 为 target。每个有向顶点对只有一条 dedge。一个物理边通常对应两条反向 dedge，单 TET 有 12 条 dedge。
- `Vertex::outgoing` 存以该顶点为 source 的 dedge；`incident_halffaces` 存使用该有向边的半面。
- 半面只属于一个单元。共形内部面的两个半面互为 opposite；边界 opposite 为空，不创建外部单元，也不单独存储无向 Face。
- `Cell::halffaces` 有效长度为 TET 4、HEX 6、PYRAMID/PRISM 5；三角面的 dedges 有效长度为 3，四边面为 4。其余位置为空句柄，遍历时必须跳过。
- `edge_cells_ccw` 在正向编号下，从 target 看向 source 为逆时针。内部边返回闭环的一次遍历、不重复首项，起点取最小底层 key；边界返回完整开链。反转 dedge 反转次序，闭环允许循环移位。
- 有序边星查询会检查局部半面/单元存在性、所属关系、opposite 互指、反向边、重复关联及分支/不连通；发现这些异常返回空 vector，不返回部分序列。该检查不替代整体拓扑或几何验证。
- 其余邻接不承诺几何顺序；无效句柄的邻接返回空 vector。查询沿维护的局部关系访问，不扫描整个 Mesh。

面模板在 `mesh_element_template.h`。TET 参考坐标为 `(0,0,0),(1,0,0),(0,1,0),(0,0,1)`；HEX 下层为逆时针方形 0..3，上层 4..7 对应；PYRAMID 下层同 HEX，4 为顶点；PRISM 下层三角形 0..2，上层 3..5 对应。面环从单元外侧看为逆时针。

`order` 的数字是输入顶点数组的局部下标，行顺序决定 Cell 的半面顺序，环内顺序决定半面的 dedge 顺序。三角面用末尾 -1 补齐四项；PYRAMID 底面行可放在任意位置。

与 DynamicSolidMesh 相同，调用方负责提供合法、有向闭合的模板，并保证编辑前后的网格共形且流形；不检查自定义模板的下标/结构、重复单元、共享面方向或边/顶点邻域流形性。顶点句柄必须有效且互不重复，此项在修改前检查。坐标重合不自动焊接，也不验证几何退化、Jacobian、重叠或自交。网格可包含孤立顶点和多个独立连通分量。

无效句柄、重复顶点、非有限顶点坐标会返回空句柄/false，并在这些检查失败时保持网格不变。成功不代表通过拓扑或几何合法性验证；不保证内存分配异常时回滚。

删除单元断开 opposite、清理无用半面和有向边、更新 outgoing，顶点保留。只有 outgoing 为空的顶点可以删除。编辑前收集句柄快照，不要在活动 range-for 中增删。

**EntityPool 可能在插入时重新分配、删除时搬移末尾元素。编辑后应重新通过句柄获取元素，不跨编辑缓存元素裸指针。**稳定值句柄与元素内存地址是两个不同概念，这一点与 DynamicSolidMesh 分页存储的实现差异有关。

复制直接深拷贝四个存储，保留句柄值与全部连接关系；不会重建或重新编号。拷贝赋值使用 copy-and-swap。调用方不应依赖两个独立 Mesh 将来的插入生成相同值句柄。

## VTK I/O

```cpp
#include <solidmesh/mesh/mesh_io.h>
SolidMesh::Mesh mesh;
bool loaded = SolidMesh::MeshIO::read_vtk("input.vtk", mesh);
bool saved = loaded && SolidMesh::MeshIO::write_vtk("output.vtk", mesh);
```

支持 VTK Legacy ASCII Unstructured Grid 的 TET/HEX/PRISM/PYRAMID，其他类型跳过。读入在临时 Mesh 上构建，失败保留原 Mesh；拓扑合法性仍由输入提供方保证。写出使用 double 的 `max_digits10` 保留往返精度。

Cell 不保存原始有序顶点表。写出从有向半面匹配单元模板，恢复一个等价的 VTK 局部编号，保留面环方向。默认模板可恢复原编号；自定义合法模板可能输出不同但等价的编号。不能把简单去重后的顶点集合直接用作 VTK 单元连接。

## 旧接口迁移

这是公开 API/ABI 重构，需要重新编译调用方。

| 旧接口 | 新接口 |
|---|---|
| `PolyhedralMesh` | `Mesh`；旧类型名保留为别名 |
| `add_vertex({x,y,z})` | `add_isolated_vertex(x,y,z)` |
| `add_cell(CellType::Tet, vertices)` | `add_tet(std::array<VertexHandle,4>)`；其他类型同理 |
| `v.position()` / `v.set_position(p)` | `mesh.vertex(v)->pos` / `mesh.set_vertex_position(v,x,y,z)` |
| `h.is_valid()` | 相应的 `mesh.vertex/dedge/halfface/cell(h) != nullptr` |
| `HalfFaceHandle` | `HalffaceHandle` |
| `c.adjacent_cells()` | `mesh.cell_cells(c)` |
| `edge_cells(v0,v1)` | 从 v0 的 outgoing 取得目标为 v1 的 DedgeHandle，再调用 `edge_cells(edge)` |
| `duplicate()` | `copy()`，现在保留句柄值 |
| `id()`、`*_at()`、mutable pool | 使用强类型值句柄和 range，不暴露底层索引 |

旧 Face/FaceHandle、无向 face_map、vertex_cell_adj、元素上的链式查询以及内置 validate 已移除。需要的高层派生查询可由调用方组合元素关系；测试框架用独立拓扑不变量验证替代内部 validate。旧 include 路径 `polyhedral_mesh.h`、`handles.h`、`mesh_elements.h`、`cell_topology.h` 只转发到新头文件，不恢复旧方法。
