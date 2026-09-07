#include "updater/updater_app.h"

#include <arpa/inet.h>
#include <netinet/in.h>

#include <string>
#include <vector>

namespace updater
{

UpdaterApp::UpdaterApp(std::filesystem::path install_script, uint16_t port)
    : port_(port)
    , pkg_path_(kDefaultPkgPath)
    , install_script_(std::move(install_script))
    , ota_(logger_, pkg_path_, install_script_)
{
}

int UpdaterApp::Run()
{
    UdpListener udp(port_);
    if (!udp.OpenAndBind())
    {
        return 1;
    }

    logger_.Log("updater_example listening UDP " + std::to_string(port_));
    logger_.Log("send: OTA <http-url> <md5>");
    logger_.Log("install script: " + install_script_.string());

    std::vector<char> buf(65536);
    for (;;)
    {
        sockaddr_in peer{};
        ssize_t n = 0;
        if (!udp.RecvFrom(buf, peer, n))
        {
            continue;
        }

        char peer_str[INET_ADDRSTRLEN]{};
        inet_ntop(AF_INET, &peer.sin_addr, peer_str, sizeof(peer_str));
        logger_.Log(
            std::string("from ") + peer_str + ":" + std::to_string(ntohs(peer.sin_port)) +
            " len=" + std::to_string(n)
        );

        std::string line(buf.data());
        while (!line.empty() && (line.back() == '\n' || line.back() == '\r'))
        {
            line.pop_back();
        }

        if (auto ota = OtaController::ParseLine(line, logger_))
        {
            ota_.TryStartAsync(std::move(*ota));
        }
    }
}

std::filesystem::path ResolveInstallScript(int argc, char** argv)
{
    if (argc >= 2)
    {
        return argv[1];
    }
    return std::filesystem::path(argv[0]).parent_path() / "install_firmware.sh";
}

}  // namespace updater
