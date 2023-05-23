#include "profiling_common.hpp"
#include <nlohmann/json.hpp>
#include <fstream>

int main(int argc, char ** argv)
{
    SerialSendRecvMonitor monitor(5, "/dev/ttyUSB0", "/dev/ttyUSB1");
    monitor.run(5000);

    auto statistics = monitor.statistics();
    nlohmann::json j;
    j["description"] = "MAVTunnel end to end latency (UART, 115200, Pi4)";
    j["total_tx"] = statistics.total_tx;
    j["total_rx"] = statistics.total_rx;
    j["drops"] = statistics.drops;
    j["latency"] = {
        {"unit", "ns"},
        {"min", statistics.latency.min},
        {"max", statistics.latency.max},
        {"mean", statistics.latency.mean}
    };

    char filename[128];
    std::time_t now = std::time(nullptr);
    std::tm * tm = std::localtime(&now);
    std::sprintf(filename, "%s-%04d-%02d0-%02d.json", argv[0], tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday);
    std::ofstream out(filename);
    out << j.dump(4) << std::endl;
    out.flush();
    out.close();

    return 0;
}
