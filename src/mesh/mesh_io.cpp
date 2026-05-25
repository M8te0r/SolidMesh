#include <solidmesh/mesh/mesh_io.h>

#include <solidmesh/mesh/io_helper.h>

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>

namespace SolidMesh
{
    namespace
    {
        bool read_size(FastAsciiReader &reader, std::size_t &value)
        {
            std::uint64_t raw = 0;
            if (!reader.read_uint64(raw) ||
                raw > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()))
            {
                return false;
            }

            value = static_cast<std::size_t>(raw);
            return true;
        }

        bool read_uint32(FastAsciiReader &reader, std::uint32_t &value)
        {
            std::uint64_t raw = 0;
            if (!reader.read_uint64(raw) ||
                raw > static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()))
            {
                return false;
            }

            value = static_cast<std::uint32_t>(raw);
            return true;
        }

        bool contains_case_insensitive(std::string line, const std::string &text)
        {
            auto to_upper = [](unsigned char c) {
                return static_cast<char>(std::toupper(c));
            };
            std::transform(line.begin(), line.end(), line.begin(), to_upper);

            std::string upper_text = text;
            std::transform(upper_text.begin(), upper_text.end(), upper_text.begin(), to_upper);
            return line.find(upper_text) != std::string::npos;
        }

        std::size_t vtk_cell_vertex_count(int vtk_type)
        {
            switch (vtk_type)
            {
            case 10:
                return 4; // VTK_TETRA
            case 12:
                return 8; // VTK_HEXAHEDRON
            case 13:
                return 6; // VTK_WEDGE
            case 14:
                return 5; // VTK_PYRAMID
            default:
                return 0;
            }
        }

        bool is_supported_vtk_cell(int vtk_type)
        {
            return vtk_cell_vertex_count(vtk_type) != 0;
        }
    }

    bool MeshIO::read_vtk(const std::string &filename, VtkRawData &out_data)
    {
        try
        {
            FastAsciiReader reader(filename);

            std::string line;
            if (!reader.read_line(line))
            {
                return false;
            }
            if (!reader.read_line(line))
            {
                return false;
            }
            if (!reader.read_line(line) || !contains_case_insensitive(line, "ASCII"))
            {
                return false;
            }
            if (!reader.read_line(line) || !contains_case_insensitive(line, "UNSTRUCTURED_GRID"))
            {
                return false;
            }

            std::vector<std::vector<double>> vertices;
            std::vector<std::vector<std::uint32_t>> raw_cells;
            std::vector<int> raw_cell_types;

            bool have_points = false;
            bool have_cells = false;
            bool have_cell_types = false;

            std::string token;
            while (reader.read_token(token))
            {
                if (token == "POINTS")
                {
                    std::size_t point_count = 0;
                    std::string scalar_type;
                    if (!read_size(reader, point_count) ||
                        point_count > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()) ||
                        !reader.read_token(scalar_type))
                    {
                        return false;
                    }

                    vertices.clear();
                    vertices.resize(point_count, std::vector<double>(3));
                    for (std::size_t i = 0; i < point_count; ++i)
                    {
                        if (!reader.read_double(vertices[i][0]) ||
                            !reader.read_double(vertices[i][1]) ||
                            !reader.read_double(vertices[i][2]))
                        {
                            return false;
                        }
                    }

                    have_points = true;
                    continue;
                }

                if (token == "CELLS")
                {
                    std::size_t cell_count = 0;
                    std::size_t total_size = 0;
                    if (!read_size(reader, cell_count) ||
                        !read_size(reader, total_size) ||
                        total_size < cell_count)
                    {
                        return false;
                    }

                    raw_cells.clear();
                    raw_cells.resize(cell_count);

                    std::size_t consumed = 0;
                    for (std::size_t cell = 0; cell < cell_count; ++cell)
                    {
                        std::size_t vertex_count = 0;
                        if (!read_size(reader, vertex_count))
                        {
                            return false;
                        }

                        consumed += vertex_count + 1;
                        raw_cells[cell].resize(vertex_count);
                        for (std::size_t v = 0; v < vertex_count; ++v)
                        {
                            if (!read_uint32(reader, raw_cells[cell][v]) ||
                                (have_points && raw_cells[cell][v] >= vertices.size()))
                            {
                                return false;
                            }
                        }
                    }

                    if (consumed != total_size)
                    {
                        return false;
                    }

                    have_cells = true;
                    continue;
                }

                if (token == "CELL_TYPES")
                {
                    std::size_t cell_type_count = 0;
                    if (!read_size(reader, cell_type_count))
                    {
                        return false;
                    }

                    raw_cell_types.clear();
                    raw_cell_types.resize(cell_type_count);
                    for (std::size_t i = 0; i < cell_type_count; ++i)
                    {
                        std::uint32_t vtk_type = 0;
                        if (!read_uint32(reader, vtk_type))
                        {
                            return false;
                        }
                        raw_cell_types[i] = static_cast<int>(vtk_type);
                    }

                    have_cell_types = true;
                    if (have_points && have_cells)
                    {
                        break;
                    }
                }
            }

            if (!have_points || !have_cells || !have_cell_types ||
                raw_cell_types.size() != raw_cells.size())
            {
                return false;
            }

            std::vector<std::vector<std::uint32_t>> cells;
            std::vector<int> cell_types;
            cells.reserve(raw_cells.size());
            cell_types.reserve(raw_cell_types.size());

            for (std::size_t i = 0; i < raw_cells.size(); ++i)
            {
                const int vtk_type = raw_cell_types[i];
                const std::size_t expected_count = vtk_cell_vertex_count(vtk_type);
                if (expected_count == 0)
                {
                    continue;
                }
                if (raw_cells[i].size() != expected_count)
                {
                    return false;
                }
                for (const std::uint32_t vertex_id : raw_cells[i])
                {
                    if (vertex_id >= vertices.size())
                    {
                        return false;
                    }
                }

                cells.push_back(std::move(raw_cells[i]));
                cell_types.push_back(vtk_type);
            }

            out_data.vertices = std::move(vertices);
            out_data.cells = std::move(cells);
            out_data.cell_types = std::move(cell_types);
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    bool MeshIO::write_vtk(const std::string &filename, const VtkRawData &data)
    {
        try
        {
            if (data.cells.size() != data.cell_types.size())
            {
                return false;
            }

            std::size_t supported_cell_count = 0;
            std::size_t total_cell_size = 0;
            for (std::size_t i = 0; i < data.cells.size(); ++i)
            {
                const std::size_t expected_count = vtk_cell_vertex_count(data.cell_types[i]);
                if (expected_count == 0)
                {
                    continue;
                }
                if (data.cells[i].size() != expected_count)
                {
                    return false;
                }
                for (const std::uint32_t vertex_id : data.cells[i])
                {
                    if (vertex_id >= data.vertices.size())
                    {
                        return false;
                    }
                }

                ++supported_cell_count;
                total_cell_size += expected_count + 1;
            }

            FastAsciiWriter writer(filename);

            if (!writer.write_string("# vtk DataFile Version 3.0\n") ||
                !writer.write_string("SolidMesh export\n") ||
                !writer.write_string("ASCII\n") ||
                !writer.write_string("DATASET UNSTRUCTURED_GRID\n") ||
                !writer.write_string("POINTS ") ||
                !writer.write_uint64(static_cast<std::uint64_t>(data.vertices.size())) ||
                !writer.write_string(" double\n"))
            {
                return false;
            }

            for (const auto &vertex : data.vertices)
            {
                if (vertex.size() < 3 ||
                    !writer.write_double(vertex[0]) ||
                    !writer.write_char(' ') ||
                    !writer.write_double(vertex[1]) ||
                    !writer.write_char(' ') ||
                    !writer.write_double(vertex[2]) ||
                    !writer.write_char('\n'))
                {
                    return false;
                }
            }

            if (!writer.write_string("\nCELLS ") ||
                !writer.write_uint64(static_cast<std::uint64_t>(supported_cell_count)) ||
                !writer.write_char(' ') ||
                !writer.write_uint64(static_cast<std::uint64_t>(total_cell_size)) ||
                !writer.write_char('\n'))
            {
                return false;
            }

            for (std::size_t i = 0; i < data.cells.size(); ++i)
            {
                if (!is_supported_vtk_cell(data.cell_types[i]))
                {
                    continue;
                }

                if (!writer.write_uint64(static_cast<std::uint64_t>(data.cells[i].size())))
                {
                    return false;
                }
                for (const std::uint32_t vertex_id : data.cells[i])
                {
                    if (!writer.write_char(' ') ||
                        !writer.write_uint64(static_cast<std::uint64_t>(vertex_id)))
                    {
                        return false;
                    }
                }
                if (!writer.write_char('\n'))
                {
                    return false;
                }
            }

            if (!writer.write_string("\nCELL_TYPES ") ||
                !writer.write_uint64(static_cast<std::uint64_t>(supported_cell_count)) ||
                !writer.write_char('\n'))
            {
                return false;
            }

            for (std::size_t i = 0; i < data.cell_types.size(); ++i)
            {
                if (!is_supported_vtk_cell(data.cell_types[i]))
                {
                    continue;
                }

                if (!writer.write_uint64(static_cast<std::uint64_t>(data.cell_types[i])) ||
                    !writer.write_char('\n'))
                {
                    return false;
                }
            }

            return writer.flush() && writer.ok();
        }
        catch (...)
        {
            return false;
        }
    }

} // namespace SolidMesh
