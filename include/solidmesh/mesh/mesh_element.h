#pragma once
#include "handle.h"
#include "storage.h"
#include "mesh_element_template.h"
#include "solidmesh/math/vector3.h"
#include <array>
#include <vector>
namespace SolidMesh
{
    using Real = double;
    template <class T>
    using ElementStorage = detail::PoolStorage<T>;
    
    struct Vertex;
    struct Dedge;
    struct Halfface;
    struct Cell;
    using HandleSize = ElementStorage<Vertex>::key::id_type;
    using VertexHandle = Handle<Vertex, HandleSize>;
    using DedgeHandle = Handle<Dedge, HandleSize>;
    using HalffaceHandle = Handle<Halfface, HandleSize>;
    using CellHandle = Handle<Cell, HandleSize>;

    struct Vertex
    {
        Vector3 pos;
        std::vector<DedgeHandle> outgoing;
    };

    struct Dedge
    {
        VertexHandle v0;
        VertexHandle v1;
        std::vector<HalffaceHandle> incident_halffaces;
    };

    struct Halfface
    {
        FaceType type = FaceType::UNSET;
        std::array<DedgeHandle, 4> dedges;
        CellHandle cell;
        HalffaceHandle opposite;
    };

    struct Cell
    {
        CellType type = CellType::UNSET;
        std::array<HalffaceHandle, 6> halffaces;
    };
}
