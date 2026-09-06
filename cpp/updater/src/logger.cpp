#include "updater/logger.h"

#include <iostream>

namespace updater
{

void Logger::Log(std::string_view line)
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::cout << line << '\n';
}

}  // namespace updater
