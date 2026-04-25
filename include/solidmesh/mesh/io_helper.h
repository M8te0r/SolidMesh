#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace SolidMesh
{
    class FastAsciiReader
    {
    public:
        explicit FastAsciiReader(const std::string &path);

        ~FastAsciiReader();

        FastAsciiReader(const FastAsciiReader &) = delete;
        FastAsciiReader &operator=(const FastAsciiReader &) = delete;

        bool read_token(std::string &token);

        bool read_line(std::string &line);

        bool read_uint64(std::uint64_t &value);

        bool read_int(int &value);

        bool read_double(double &value);

    private:
        static constexpr int eof = -1;
        static constexpr std::size_t buffer_size = 1 << 20;

        static bool is_space(int c) noexcept;

        static bool is_digit(int c) noexcept;

        int read_char();

        std::FILE *file_ = nullptr;
        std::vector<char> buffer_;
        std::size_t position_ = 0;
        std::size_t end_ = 0;
    };

    class FastAsciiWriter
    {
    public:
        explicit FastAsciiWriter(const std::string &path);

        ~FastAsciiWriter();

        FastAsciiWriter(const FastAsciiWriter &) = delete;
        FastAsciiWriter &operator=(const FastAsciiWriter &) = delete;

        bool write_char(char c);

        bool write_string(const char *text);

        bool write_string(const std::string &text);

        bool write_uint64(std::uint64_t value);

        bool write_int(int value);

        bool write_double(double value);

        bool flush();

        bool ok() const noexcept { return ok_; }

    private:
        static constexpr std::size_t buffer_size = 1 << 20;

        bool write_bytes(const char *data, std::size_t size);

        std::FILE *file_ = nullptr;
        std::vector<char> buffer_;
        std::size_t position_ = 0;
        bool ok_ = true;
    };
}
