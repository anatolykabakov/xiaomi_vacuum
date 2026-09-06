#pragma once

#include "watchdog/config.hpp"

#include <sys/types.h>
#include <atomic>
#include <chrono>
#include <vector>

namespace watchdog
{

/// Запускает дочерние процессы по конфигу и перезапускает при выходе.
class Supervisor
{
public:
    explicit Supervisor(WatchdogConfig config);
    ~Supervisor();

    Supervisor(const Supervisor&) = delete;
    Supervisor& operator=(const Supervisor&) = delete;

    /// Блокирует до SIGINT/SIGTERM; возвращает 0.
    int Run();

    static void RequestStop();

private:
    struct Entry
    {
        ProcessSpec spec;
        pid_t pid{-1};
    };

    static std::atomic<bool> stop_requested_;

    void StartAll();
    void ShutdownChildren();
    void Spawn(Entry& e);
    void PollAndMaybeRestart(Entry& e);

    WatchdogConfig config_;
    std::vector<Entry> entries_;
};

}  // namespace watchdog
