#pragma once

#include <netinet/in.h>
#include <sys/types.h>
#include <cstdint>
#include <vector>

namespace updater
{

/// RAII UDP-сокет: bind на IPv4 ANY, приём датаграмм.
class UdpListener
{
public:
    explicit UdpListener(uint16_t port);
    ~UdpListener();

    UdpListener(const UdpListener&) = delete;
    UdpListener& operator=(const UdpListener&) = delete;
    UdpListener(UdpListener&&) = delete;
    UdpListener& operator=(UdpListener&&) = delete;

    bool OpenAndBind();

    /// Блокирует до прихода датаграммы. Возвращает false при ошибке recvfrom.
    bool RecvFrom(std::vector<char>& buf, sockaddr_in& peer, ssize_t& out_len);

private:
    void Close();

    uint16_t port_;
    int fd_{-1};
};

}  // namespace updater
