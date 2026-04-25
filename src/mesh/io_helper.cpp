#include "solidmesh/mesh/io_helper.h"

#include <cctype>
#include <cerrno>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <stdexcept>
#include <string>


namespace SolidMesh
{
    FastAsciiReader::FastAsciiReader(const std::string &path)
        : file_(std::fopen(path.c_str(), "rb")), buffer_(buffer_size)
    {
        if (file_ == nullptr)
        {
            throw std::runtime_error("failed to open file: " + path);
        }
    }

    FastAsciiReader::~FastAsciiReader()
    {
        if (file_ != nullptr)
        {
            std::fclose(file_);
        }
    }

    bool FastAsciiReader::read_token(std::string &token)
    {
        token.clear();

        int c = read_char();
        while (c != eof && is_space(c))
        {
            c = read_char();
        }
        if (c == eof)
        {
            return false;
        }

        while (c != eof && !is_space(c))
        {
            token.push_back(static_cast<char>(c));
            c = read_char();
        }
        return true;
    }

    bool FastAsciiReader::read_line(std::string &line)
    {
        line.clear();

        int c = read_char();
        if (c == eof)
        {
            return false;
        }

        while (c != eof && c != '\n')
        {
            if (c != '\r')
            {
                line.push_back(static_cast<char>(c));
            }
            c = read_char();
        }
        return true;
    }


    bool FastAsciiReader::read_uint64(std::uint64_t &value)
    {
        value = 0;

        int c = read_char();
        while (c != eof && is_space(c))
        {
            c = read_char();
        }
        if (c == '+')
        {
            c = read_char();
        }
        if (c == eof || !is_digit(c))
        {
            return false;
        }

        while (c != eof && is_digit(c))
        {
            value = value * 10 + static_cast<std::uint64_t>(c - '0');
            c = read_char();
        }
        return true;
    }

    bool FastAsciiReader::read_int(int &value)
    {
        value = 0;
        int sign = 1;

        int c = read_char();
        while (c != eof && is_space(c))
        {
            c = read_char();
        }
        if (c == '-')
        {
            sign = -1;
            c = read_char();
        }
        else if (c == '+')
        {
            c = read_char();
        }
        if (c == eof || !is_digit(c))
        {
            return false;
        }

        while (c != eof && is_digit(c))
        {
            value = value * 10 + (c - '0');
            c = read_char();
        }
        value *= sign;
        return true;
    }

    bool FastAsciiReader::read_double(double &value)
    {
        std::string token;
        if (!read_token(token))
        {
            return false;
        }

        char *end = nullptr;
        errno = 0;
        value = std::strtod(token.c_str(), &end);
        return errno == 0 && end != token.c_str();
    }

    bool FastAsciiReader::is_space(int c) noexcept
    {
        return std::isspace(static_cast<unsigned char>(c)) != 0;
    }

    bool FastAsciiReader::is_digit(int c) noexcept
    {
        return c >= '0' && c <= '9';
    }

    int FastAsciiReader::read_char()
    {
        if (position_ == end_)
        {
            end_ = std::fread(buffer_.data(), 1, buffer_.size(), file_);
            position_ = 0;
            if (end_ == 0)
            {
                return eof;
            }
        }
        return static_cast<unsigned char>(buffer_[position_++]);
    }

    FastAsciiWriter::FastAsciiWriter(const std::string &path)
        : file_(std::fopen(path.c_str(), "wb")), buffer_(buffer_size)
    {
        if (file_ == nullptr)
        {
            throw std::runtime_error("failed to open file: " + path);
        }
    }

    FastAsciiWriter::~FastAsciiWriter()
    {
        if (file_ != nullptr)
        {
            flush();
            std::fclose(file_);
        }
    }

    bool FastAsciiWriter::write_char(char c)
    {
        return write_bytes(&c, 1);
    }

    bool FastAsciiWriter::write_string(const char *text)
    {
        return write_bytes(text, std::strlen(text));
    }

    bool FastAsciiWriter::write_string(const std::string &text)
    {
        return write_bytes(text.data(), text.size());
    }

    bool FastAsciiWriter::write_uint64(std::uint64_t value)
    {
        char text[32];
        const int count = std::snprintf(
            text, sizeof(text), "%llu", static_cast<unsigned long long>(value));
        return count > 0 && write_bytes(text, static_cast<std::size_t>(count));
    }

    bool FastAsciiWriter::write_int(int value)
    {
        char text[32];
        const int count = std::snprintf(text, sizeof(text), "%d", value);
        return count > 0 && write_bytes(text, static_cast<std::size_t>(count));
    }

    bool FastAsciiWriter::write_double(double value)
    {
        char text[64];
        const int count = std::snprintf(text, sizeof(text), "%.6g", value);
        return count > 0 && write_bytes(text, static_cast<std::size_t>(count));
    }

    bool FastAsciiWriter::flush()
    {
        if (!ok_ || position_ == 0)
        {
            return ok_;
        }

        const std::size_t written = std::fwrite(buffer_.data(), 1, position_, file_);
        ok_ = written == position_;
        position_ = 0;
        return ok_;
    }

    bool FastAsciiWriter::write_bytes(const char *data, std::size_t size)
    {
        if (!ok_)
        {
            return false;
        }

        if (size > buffer_.size())
        {
            if (!flush())
            {
                return false;
            }
            ok_ = std::fwrite(data, 1, size, file_) == size;
            return ok_;
        }

        if (position_ + size > buffer_.size() && !flush())
        {
            return false;
        }

        std::memcpy(buffer_.data() + position_, data, size);
        position_ += size;
        return true;
    }
}
