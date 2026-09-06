#pragma once

#include "updater/logger.h"

#include <atomic>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace updater
{

struct OtaArgs
{
    std::string url;
    std::string md5_hex;
};

class OtaController
{
public:
    OtaController(
        Logger& log,
        std::filesystem::path pkg_path,
        std::filesystem::path install_script
    );

    /// Разбор одной строки датаграммы. Пустой optional = не OTA или ошибка (уже залогировано).
    static std::optional<OtaArgs> ParseLine(std::string_view line, Logger& log);

    /// Неблокирующая постановка OTA; false если уже идёт другая.
    bool TryStartAsync(OtaArgs args);

private:
    static std::string ShellSingleQuote(const std::string& s);
    bool RunCommand(const std::string& cmd);
    bool DownloadWithCurl(const std::string& url);
    bool VerifyMd5(const std::string& expected_hex);
    bool RunInstallScript();
    void RunPipeline(OtaArgs args);

    Logger& log_;
    std::filesystem::path pkg_path_;
    std::filesystem::path install_script_;
    std::atomic<bool> busy_{false};
};

}  // namespace updater
