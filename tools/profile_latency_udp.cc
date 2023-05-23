#include "profiling_common.hpp"
#include <fstream>
#include <nlohmann/json.hpp>

int
main(int argc, char** argv)
{
    UDPSendRecvMonitor monitor(100, 4550, 15550);
    monitor.run(5000);

    auto           statistics = monitor.statistics();
    nlohmann::json j;
    j["description"] = "MAVTunnel end to end latency (UDP, Pi4)";
    if (argc > 1)
    {
        j["argument"] = argv[1];
    }
    j["total_tx"] = statistics.total_tx;
    j["total_rx"] = statistics.total_rx;
    j["drops"]    = statistics.drops;
    j["latency"]  = {
        {"unit",  "ns"                   },
        { "min",  statistics.latency.min },
        { "max",  statistics.latency.max },
        { "mean", statistics.latency.mean}
    };

    char        filename[128];
    std::time_t now = std::time(nullptr);
    std::tm*    tm  = std::localtime(&now);
    if (argc > 1)
    {
        std::sprintf(filename, "%s-%s-%04d-%02d0-%02d-%02d%02d%02d.json",
            argv[0], argv[1], tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
            tm->tm_hour, tm->tm_min, tm->tm_sec);
    }
    else
    {
        std::sprintf(filename, "%s-%04d-%02d0-%02d-%02d%02d%02d.json", argv[0],
            tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour,
            tm->tm_min, tm->tm_sec);
    }
    std::ofstream out(filename);
    out << j.dump(4) << std::endl;
    out.flush();
    out.close();

    return 0;
}
