#pragma once

#include "updater/logger.h"
#include "updater/ota_controller.h"
#include "updater/udp_listener.h"

#include <cstdint>
#include <filesystem>

namespace updater
{

class UpdaterApp
{
public:
    static constexpr uint16_t kDefaultPort = 54322;
    static constexpr const char* kDefaultPkgPath = "/tmp/updater_example.pkg";

    UpdaterApp(std::filesystem::path install_script, uint16_t port = kDefaultPort);

    int Run();

private:
    uint16_t port_;
    std::filesystem::path pkg_path_;
    std::filesystem::path install_script_;
    Logger logger_;
    OtaController ota_;
};

std::filesystem::path ResolveInstallScript(int argc, char** argv);

}  // namespace updater
