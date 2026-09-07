#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>

namespace
{
constexpr int K_SOCKET_READ_TIMEOUT_MS = 3000;
constexpr size_t K_HTTP_RESPONSE_BUFFER_SIZE = 512;
constexpr int K_HTTP_SUCCESS_MIN = 200;
constexpr int K_HTTP_SUCCESS_MAX = 300;
}  // namespace

bool SendHttpRequestLocal(
    const std::string& method,
    const std::string& host,
    int port,
    const std::string& path,
    const std::string& body,
    int& status_code
)
{
    status_code = 0;
    const int sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1)
    {
        ::close(sock);
        return false;
    }
    if (connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
    {
        ::close(sock);
        return false;
    }

    std::ostringstream req;
    req << method << " " << path << " HTTP/1.1\r\n"
        << "Host: " << host << ":" << port << "\r\n"
        << "Connection: close\r\n"
        << "Content-Type: application/json\r\n"
        << "Content-Length: " << body.size() << "\r\n\r\n"
        << body;
    const std::string request = req.str();
    if (send(sock, request.data(), request.size(), 0) < 0)
    {
        ::close(sock);
        return false;
    }

    // NOLINTBEGIN(misc-include-cleaner)
    struct pollfd pfd
    {
    };
    pfd.fd = sock;
    pfd.events = POLLIN;
    if (::poll(&pfd, 1, K_SOCKET_READ_TIMEOUT_MS) <= 0)
    {
        ::close(sock);
        return false;
    }
    // NOLINTEND(misc-include-cleaner)

    char buf[K_HTTP_RESPONSE_BUFFER_SIZE];
    const ssize_t n = recv(sock, buf, sizeof(buf) - 1, 0);
    ::close(sock);
    if (n <= 0)
    {
        return false;
    }
    buf[n] = '\0';
    int http_code = 0;
    if (std::sscanf(buf, "HTTP/%*d.%*d %d", &http_code) == 1)
    {
        status_code = http_code;
        return true;
    }
    return false;
}

bool TriggerValetudoBasicControlAction(const std::string& host, int port, const std::string& action)
{
    struct Endpoint
    {
        const char* method;
        const char* path;
        std::string body;
    };
    const std::string action_body = std::string("{\"action\":\"") + action + "\"}";
    const Endpoint endpoints[] = {
        {"PUT", "/api/v2/robot/capabilities/BasicControlCapability", action_body},
        {"POST", "/api/v2/robot/capabilities/BasicControlCapability", action_body},
        {"PUT", "/api/start_cleaning", "{}"},
        {"POST", "/api/start_cleaning", "{}"},
        {"PUT", "/api/v2/robot/capabilities/BasicControlCapability/start", "{}"},
        {"POST", "/api/v2/robot/capabilities/BasicControlCapability/start", "{}"},
        {"PUT", "/api/v2/robot/capabilities/BasicControlCapability/start/set", "{}"},
        {"POST", "/api/v2/robot/capabilities/BasicControlCapability/start/set", "{}"},
    };
    int last_status = 0;
    for (const auto& ep : endpoints)
    {
        int status = 0;
        const bool ok = SendHttpRequestLocal(ep.method, host, port, ep.path, ep.body, status);
        std::fprintf(
            stderr,
            "[valetudo] action=%s %s %s -> ok=%d status=%d\n",
            action.c_str(),
            ep.method,
            ep.path,
            ok ? 1 : 0,
            status
        );
        last_status = status;
        if (ok && status >= K_HTTP_SUCCESS_MIN && status < K_HTTP_SUCCESS_MAX)
        {
            return true;
        }
    }
    std::fprintf(
        stderr,
        "[valetudo] action=%s failed (last HTTP status=%d). "
        "Likely unsupported endpoint for this Valetudo build.\n",
        action.c_str(),
        last_status
    );
    return false;
}
