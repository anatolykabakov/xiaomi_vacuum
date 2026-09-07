#pragma once

#include <mutex>
#include <string_view>

namespace updater
{

class Logger
{
public:
    void Log(std::string_view line);

private:
    std::mutex mutex_;
};

}  // namespace updater
