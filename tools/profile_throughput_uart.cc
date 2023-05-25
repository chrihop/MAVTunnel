#include "throughput_common.hpp"
#include <fstream>
#include <nlohmann/json.hpp>

int
main(int argc, char** argv)
{
    SerialThroughputMonitor monitor("/dev/ttyUART_IO1", "/dev/ttyUART_IO2");
    monitor.run(100, 0, 240, 32);

    nlohmann::json j;
    j["description"] = "MAVTunnel Throughput (UART, 115200, Pi4)";
    if (argc > 1)
    {
        j["argument"] = argv[1];
    }

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
