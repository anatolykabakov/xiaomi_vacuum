#include "watchdog/supervisor.hpp"

#include <sys/wait.h>
#include <unistd.h>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <thread>

namespace watchdog
{

std::atomic<bool> Supervisor::stop_requested_{false};

namespace
{

extern "C" void SignalHandler(int /*sig*/)
{
    Supervisor::RequestStop();
}

}  // namespace

void Supervisor::RequestStop()
{
    stop_requested_ = true;
}

Supervisor::Supervisor(WatchdogConfig config) : config_(std::move(config))
{
    entries_.reserve(config_.processes.size());
    for (auto& p : config_.processes)
    {
        entries_.push_back(Entry{.spec = std::move(p), .pid = -1});
    }
}

Supervisor::~Supervisor()
{
    ShutdownChildren();
}

void Supervisor::Spawn(Entry& e)
{
    if (stop_requested_)
    {
        return;
    }
    pid_t pid = fork();
    if (pid < 0)
    {
        perror("fork");
        return;
    }
    if (pid == 0)
    {
        if (!e.spec.cwd.empty())
        {
            if (chdir(e.spec.cwd.string().c_str()) != 0)
            {
                perror("chdir");
                _exit(126);
            }
        }
        std::vector<char*> argv;
        argv.reserve(e.spec.command.size() + 1);
        for (auto& s : e.spec.command)
        {
            argv.push_back(s.data());
        }
        argv.push_back(nullptr);
        execvp(argv[0], argv.data());
        perror("execvp");
        _exit(127);
    }
    e.pid = pid;
    std::cerr << "[watchdog] started '" << e.spec.name << "' pid=" << pid << std::endl;
}

void Supervisor::StartAll()
{
    for (auto& e : entries_)
    {
        Spawn(e);
    }
}

void Supervisor::ShutdownChildren()
{
    for (auto& e : entries_)
    {
        if (e.pid > 0)
        {
            kill(e.pid, SIGTERM);
        }
    }
    using namespace std::chrono_literals;
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    for (auto& e : entries_)
    {
        if (e.pid <= 0)
        {
            continue;
        }
        int status = 0;
        while (std::chrono::steady_clock::now() < deadline)
        {
            const pid_t r = waitpid(e.pid, &status, WNOHANG);
            if (r == e.pid)
            {
                e.pid = -1;
                break;
            }
            if (r < 0)
            {
                perror("waitpid");
                e.pid = -1;
                break;
            }
            std::this_thread::sleep_for(50ms);
        }
        if (e.pid > 0)
        {
            kill(e.pid, SIGKILL);
            waitpid(e.pid, &status, 0);
            e.pid = -1;
        }
    }
}

void Supervisor::PollAndMaybeRestart(Entry& e)
{
    if (e.pid <= 0)
    {
        return;
    }
    int status = 0;
    const pid_t r = waitpid(e.pid, &status, WNOHANG);
    if (r == 0)
    {
        return;
    }
    if (r < 0)
    {
        perror("waitpid");
        e.pid = -1;
        return;
    }
    if (r != e.pid)
    {
        return;
    }
    std::cerr << "[watchdog] '" << e.spec.name << "' exited status=" << status << std::endl;
    e.pid = -1;
    if (stop_requested_)
    {
        return;
    }
    std::this_thread::sleep_for(e.spec.restart_delay);
    if (stop_requested_)
    {
        return;
    }
    Spawn(e);
}

int Supervisor::Run()
{
    struct sigaction sa
    {
    };
    sa.sa_handler = SignalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    stop_requested_ = false;
    StartAll();

    while (!stop_requested_)
    {
        std::this_thread::sleep_for(config_.check_interval);
        for (auto& e : entries_)
        {
            PollAndMaybeRestart(e);
        }
    }

    std::cerr << "[watchdog] shutting down…\n";
    ShutdownChildren();
    return 0;
}

}  // namespace watchdog
