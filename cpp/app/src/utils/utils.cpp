#include "utils/utils.h"

#include <json/reader.h>
#include <json/value.h>

#include <execinfo.h>
#include <sys/resource.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

namespace project
{
namespace utils
{
namespace
{

constexpr int K_SIGNAL_POLL_MS = 100;

}  // namespace

void WaitForSignal()
{
    static std::atomic<bool> stop_requested{false};
    auto signal_handler = [](int signum) { stop_requested.store(true, std::memory_order_relaxed); };
    std::signal(SIGINT, signal_handler);   // Ctrl+C
    std::signal(SIGTERM, signal_handler);  // docker/systemd shutdown
    while (!stop_requested.load(std::memory_order_relaxed))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(K_SIGNAL_POLL_MS));
    }
}

Json::Value LoadJsonConfig(const std::string& path)
{
    Json::Value root;
    Json::Reader reader;

    std::ifstream file(path);
    if (!file.is_open())
    {
        std::cout << "Failed to open configuration file: " << path << std::endl;
        return root;
    }

    if (!reader.parse(file, root, true))
    {
        std::cout << "Failed to parse configuration\n" << reader.getFormattedErrorMessages();
    }
    return root;
}

void EnableCoreDump()
{
    rlimit core_limit{};
    core_limit.rlim_cur = RLIM_INFINITY;
    core_limit.rlim_max = RLIM_INFINITY;
    if (setrlimit(RLIMIT_CORE, &core_limit) != 0)
    {
        std::cerr << "setrlimit(RLIMIT_CORE) failed: " << std::strerror(errno) << std::endl;
    }
}

void crashHandler()
{
    EnableCoreDump();
    auto printBacktrace = [](int sig)
    {
        void* frames[64];
        int size = backtrace(frames, 64);
        char** symbols = backtrace_symbols(frames, size);
        if (symbols)
        {
            for (int i = 0; i < size; ++i)
            {
                std::cerr << symbols[i] << "\n";
            }
        }
        free(symbols);
        ::signal(sig, SIG_DFL);
        ::raise(sig);
    };
    ::signal(SIGSEGV, printBacktrace);
    ::signal(SIGFPE, printBacktrace);
    ::signal(SIGABRT, printBacktrace);
    ::signal(SIGILL, printBacktrace);
    ::signal(SIGBUS, printBacktrace);
}

}  // namespace utils
}  // namespace project
