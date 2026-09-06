#include "watchdog/config.hpp"

#include <cctype>
#include <fstream>
#include <sstream>
#include <string_view>

namespace watchdog
{
namespace
{

class JsonCursor
{
public:
    explicit JsonCursor(std::string_view in) : in_(in) {}

    void SkipWs()
    {
        while (pos_ < in_.size() && std::isspace(static_cast<unsigned char>(in_[pos_])))
        {
            ++pos_;
        }
    }

    bool Consume(char c)
    {
        SkipWs();
        if (pos_ >= in_.size() || in_[pos_] != c)
        {
            return false;
        }
        ++pos_;
        return true;
    }

    double ParseNumber()
    {
        SkipWs();
        size_t start = pos_;
        if (pos_ < in_.size() && (in_[pos_] == '-' || in_[pos_] == '+'))
        {
            ++pos_;
        }
        while (pos_ < in_.size() && std::isdigit(static_cast<unsigned char>(in_[pos_])))
        {
            ++pos_;
        }
        if (pos_ < in_.size() && in_[pos_] == '.')
        {
            ++pos_;
            while (pos_ < in_.size() && std::isdigit(static_cast<unsigned char>(in_[pos_])))
            {
                ++pos_;
            }
        }
        if (start == pos_)
        {
            throw std::runtime_error("json: expected number");
        }
        return std::stod(std::string(in_.substr(start, pos_ - start)));
    }

    std::string ParseString()
    {
        SkipWs();
        if (!Consume('"'))
        {
            throw std::runtime_error("json: expected '\"'");
        }
        std::string out;
        while (pos_ < in_.size())
        {
            char c = in_[pos_++];
            if (c == '"')
            {
                return out;
            }
            if (c == '\\' && pos_ < in_.size())
            {
                out += in_[pos_++];
            }
            else
            {
                out += c;
            }
        }
        throw std::runtime_error("json: unterminated string");
    }

    void Expect(char c)
    {
        if (!Consume(c))
        {
            throw std::runtime_error(std::string("json: expected '") + c + "'");
        }
    }

    bool Peek(char c) const
    {
        size_t p = pos_;
        while (p < in_.size() && std::isspace(static_cast<unsigned char>(in_[p])))
        {
            ++p;
        }
        return p < in_.size() && in_[p] == c;
    }

    std::vector<std::string> ParseStringArray()
    {
        Expect('[');
        SkipWs();
        std::vector<std::string> items;
        if (Consume(']'))
        {
            return items;
        }
        for (;;)
        {
            items.push_back(ParseString());
            SkipWs();
            if (Consume(']'))
            {
                break;
            }
            Expect(',');
        }
        return items;
    }

private:
    std::string_view in_;
    size_t pos_{0};
};

std::string ReadFile(const std::filesystem::path& path)
{
    std::ifstream in(path, std::ios::binary);
    if (!in)
    {
        throw std::runtime_error("cannot open config: " + path.string());
    }
    std::ostringstream oss;
    oss << in.rdbuf();
    return oss.str();
}

ProcessSpec ParseOneProcess(JsonCursor& c)
{
    c.Expect('{');
    ProcessSpec spec;
    bool got_name = false;
    bool got_cmd = false;

    while (true)
    {
        c.SkipWs();
        const std::string pk = c.ParseString();
        c.SkipWs();
        c.Expect(':');
        c.SkipWs();
        if (pk == "name")
        {
            spec.name = c.ParseString();
            got_name = true;
        }
        else if (pk == "command")
        {
            if (c.Peek('"'))
            {
                spec.command = {c.ParseString()};
            }
            else
            {
                spec.command = c.ParseStringArray();
            }
            got_cmd = true;
        }
        else if (pk == "cwd")
        {
            if (c.Peek('"'))
            {
                spec.cwd = c.ParseString();
            }
        }
        else if (pk == "restart_delay_sec")
        {
            const double sec = c.ParseNumber();
            spec.restart_delay = std::chrono::milliseconds(static_cast<int>(sec * 1000.0));
        }
        else
        {
            throw std::runtime_error("unknown field in process: '" + pk + "'");
        }
        c.SkipWs();
        if (c.Consume('}'))
        {
            break;
        }
        c.Expect(',');
    }

    if (!got_name || !got_cmd || spec.command.empty())
    {
        throw std::runtime_error("process entry needs name and non-empty command");
    }
    return spec;
}

}  // namespace

WatchdogConfig LoadConfig(const std::filesystem::path& path)
{
    const std::string raw = ReadFile(path);
    JsonCursor cur{std::string_view(raw)};

    cur.Expect('{');
    WatchdogConfig out;

    while (true)
    {
        cur.SkipWs();
        if (cur.Consume('}'))
        {
            break;
        }
        const std::string key = cur.ParseString();
        cur.SkipWs();
        cur.Expect(':');
        cur.SkipWs();

        if (key == "check_interval_sec")
        {
            const double sec = cur.ParseNumber();
            out.check_interval = std::chrono::milliseconds(static_cast<int>(sec * 1000.0));
        }
        else if (key == "processes")
        {
            cur.Expect('[');
            cur.SkipWs();
            if (cur.Consume(']'))
            {
                // empty
            }
            else
            {
                for (;;)
                {
                    out.processes.push_back(ParseOneProcess(cur));
                    cur.SkipWs();
                    if (cur.Consume(']'))
                    {
                        break;
                    }
                    cur.Expect(',');
                }
            }
        }
        else
        {
            throw std::runtime_error("unknown root key: '" + key + "'");
        }

        cur.SkipWs();
        if (cur.Consume('}'))
        {
            break;
        }
        cur.Expect(',');
    }

    if (out.processes.empty())
    {
        throw std::runtime_error("config: 'processes' is empty");
    }
    return out;
}

}  // namespace watchdog
