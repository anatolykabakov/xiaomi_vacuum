#include "updater/ota_controller.h"

#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <sstream>
#include <thread>

namespace updater
{

OtaController::OtaController(
    Logger& log,
    std::filesystem::path pkg_path,
    std::filesystem::path install_script
)
    : log_(log)
    , pkg_path_(std::move(pkg_path))
    , install_script_(std::move(install_script))
{
}

std::optional<OtaArgs> OtaController::ParseLine(std::string_view line, Logger& log)
{
    std::string line_str(line);
    std::istringstream iss(line_str);
    std::string cmd;
    iss >> cmd;
    if (cmd != "OTA" && cmd != "ota")
    {
        log.Log(std::string("unknown command: ") + line_str);
        return std::nullopt;
    }
    std::string url;
    std::string md5;
    iss >> url >> md5;
    if (url.empty())
    {
        log.Log("bad OTA: missing url (format: OTA <url> <md5_32_hex>)");
        return std::nullopt;
    }
    if (md5.empty())
    {
        log.Log(
            "bad OTA: missing md5 — often $(md5sum /path/to/file) expanded to empty "
            "(wrong path or md5sum error). Need 32 hex chars after url."
        );
        return std::nullopt;
    }
    if (md5.size() != 32)
    {
        log.Log(
            "bad OTA: md5 must be 32 hex chars, got len=" + std::to_string(md5.size()) +
            " value='" + md5 + "'"
        );
        return std::nullopt;
    }
    if (url.rfind("http://", 0) != 0 && url.rfind("https://", 0) != 0)
    {
        log.Log("url must start with http:// or https://");
        return std::nullopt;
    }
    return OtaArgs{std::move(url), std::move(md5)};
}

bool OtaController::TryStartAsync(OtaArgs args)
{
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true))
    {
        log_.Log("OTA already in progress, ignore");
        return false;
    }
    std::thread(
        [this, args = std::move(args)]() mutable
        {
            RunPipeline(std::move(args));
            busy_ = false;
        }
    ).detach();
    return true;
}

std::string OtaController::ShellSingleQuote(const std::string& s)
{
    std::string out = "'";
    for (char c : s)
    {
        if (c == '\'')
        {
            out += "'\\''";
        }
        else
        {
            out += c;
        }
    }
    out += '\'';
    return out;
}

bool OtaController::RunCommand(const std::string& cmd)
{
    int st = std::system(cmd.c_str());
    if (st == -1)
    {
        log_.Log(std::string("system() failed: ") + std::strerror(errno));
        return false;
    }
    if (!WIFEXITED(st) || WEXITSTATUS(st) != 0)
    {
        log_.Log("command failed, status=" + std::to_string(st) + " cmd=" + cmd);
        return false;
    }
    return true;
}

bool OtaController::DownloadWithCurl(const std::string& url)
{
    const std::string cmd = "curl -f -sS --connect-timeout 10 --max-time 0 -o " +
                            ShellSingleQuote(pkg_path_.string()) + " " + ShellSingleQuote(url);
    log_.Log(std::string("download: ") + cmd);
    return RunCommand(cmd);
}

bool OtaController::VerifyMd5(const std::string& expected_hex)
{
    const std::string cmd = "echo " + ShellSingleQuote(expected_hex + "  " + pkg_path_.string()) +
                            " | md5sum -c - >/dev/null 2>&1";
    return RunCommand(cmd);
}

bool OtaController::RunInstallScript()
{
    const std::string script = install_script_.string();
    struct stat st
    {
    };
    if (stat(script.c_str(), &st) != 0 || !S_ISREG(st.st_mode))
    {
        log_.Log("install script missing or not a file: " + script);
        return false;
    }
    const std::string cmd =
        "/bin/sh " + ShellSingleQuote(script) + " " + ShellSingleQuote(pkg_path_.string());
    log_.Log(std::string("install: ") + cmd);
    return RunCommand(cmd);
}

void OtaController::RunPipeline(OtaArgs args)
{
    log_.Log("OTA thread started");
    std::error_code ec;
    std::filesystem::remove(pkg_path_, ec);

    if (!DownloadWithCurl(args.url))
    {
        log_.Log("OTA failed at download");
        return;
    }
    if (!VerifyMd5(args.md5_hex))
    {
        log_.Log("OTA failed at md5 check");
        return;
    }
    if (!RunInstallScript())
    {
        log_.Log("OTA failed at install script");
        return;
    }
    log_.Log("OTA finished OK");
}

}  // namespace updater
