#include "updater/udp_listener.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstdio>

namespace updater
{

UdpListener::UdpListener(uint16_t port) : port_(port) {}

UdpListener::~UdpListener()
{
    Close();
}

bool UdpListener::OpenAndBind()
{
    Close();
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0)
    {
        perror("socket");
        return false;
    }
    int one = 1;
    setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
    {
        perror("bind");
        Close();
        return false;
    }
    return true;
}

bool UdpListener::RecvFrom(std::vector<char>& buf, sockaddr_in& peer, ssize_t& out_len)
{
    socklen_t peer_len = sizeof(peer);
    out_len =
        recvfrom(fd_, buf.data(), buf.size() - 1, 0, reinterpret_cast<sockaddr*>(&peer), &peer_len);
    if (out_len < 0)
    {
        perror("recvfrom");
        return false;
    }
    buf[static_cast<size_t>(out_len)] = '\0';
    return true;
}

void UdpListener::Close()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

}  // namespace updater
