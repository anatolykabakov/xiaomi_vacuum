#include "watchdog/config.hpp"
#include "watchdog/supervisor.hpp"

#include <iostream>

int main(int argc, char** argv)
{
    try
    {
        const char* path = (argc >= 2) ? argv[1] : "watchdog_config.json";
        auto cfg = watchdog::LoadConfig(path);
        watchdog::Supervisor sup(std::move(cfg));
        return sup.Run();
    }
    catch (const std::exception& ex)
    {
        std::cerr << "[watchdog] error: " << ex.what() << '\n';
        return 1;
    }
}
