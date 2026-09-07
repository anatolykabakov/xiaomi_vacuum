#pragma once

#include <chrono>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace watchdog
{

struct ProcessSpec
{
    std::string name;
    std::vector<std::string> command;
    std::filesystem::path cwd;  // empty = не менять
    std::chrono::milliseconds restart_delay{1000};
};

struct WatchdogConfig
{
    std::chrono::milliseconds check_interval{2000};
    std::vector<ProcessSpec> processes;
};

/// Читает JSON (поля как в watchdog_config.example.json).
WatchdogConfig LoadConfig(const std::filesystem::path& path);

}  // namespace watchdog
