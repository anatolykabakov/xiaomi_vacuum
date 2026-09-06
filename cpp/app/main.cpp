#include "robot_app.h"
#include "utils/utils.h"

#include <cstdio>
#include <exception>
#include <string>

#include <tclap/CmdLine.h>
#include <tclap/ValueArg.h>

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/spdlog.h>

void setupLogging()
{
    constexpr std::size_t kMaxBytes = 5 * 1024 * 1024;  // 5 MiB на файл
    constexpr std::size_t kMaxFiles = 3;                // robot.log + .1 + .2
    auto sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        "/opt/xiaomi_robot.log",  // путь на роботе
        kMaxBytes,
        kMaxFiles
    );
    auto logger = std::make_shared<spdlog::logger>("xiaomi", sink);
    logger->set_level(spdlog::level::debug);
    logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");
    spdlog::set_default_logger(logger);
    spdlog::flush_on(spdlog::level::info);  // сброс буфера на диск для warn+
}

int main(int argc, char** argv)
{
    try
    {
        TCLAP::CmdLine cmd("xiaomi_robot app", ' ', "0.1");
        TCLAP::ValueArg<std::string>
            config_arg("c", "config", "Path to config JSON", false, "config/config.json", "path");
        cmd.add(config_arg);
        cmd.parse(argc, argv);

        project::utils::crashHandler();

        setupLogging();

        auto config = project::app::RobotApp::Config::LoadFromJson(config_arg.getValue());
        project::app::RobotApp app(config);
        app.Start();
    }
    catch (const std::exception& ex)
    {
        std::fprintf(stderr, "xiaomi_robot: %s\n", ex.what());
        return 3;
    }

    return 0;
}
