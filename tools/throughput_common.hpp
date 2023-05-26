#pragma once

#include <chrono>
#include <numeric>
#include <vector>

#include <v2.0/ardupilotmega/mavlink.h>

using time_point = std::chrono::system_clock::time_point;
using std::chrono::high_resolution_clock;

static inline time_point
time_ns()
{
    return high_resolution_clock::now();
}


struct ThroughputEntry
{
    size_t     payload_size {};
    size_t     total_bytes {};
    size_t     total_msgs {};
    size_t     total_drops {};
    size_t     total_drop_bytes {};
    time_point start {}, end {};

    explicit ThroughputEntry(size_t payload_size);
    void     finish(size_t total_bytes, size_t total_msgs, size_t total_drops);

    uint64_t throughput_bps();
    uint64_t throughput_msgps();
    uint64_t drop_rate_bps();
    double   drop_rate_percent();
};


class OverallThroughput
{
protected:
    size_t                        payload_size {}, total_messages {};
    size_t                        tx_count {}, rx_count {};
    size_t                        rx_seq {};
    mavlink_status_t              status {};
    mavlink_logging_data_t        data {};

public:
    std::vector<ThroughputEntry*> entries {};
    OverallThroughput() = default;
    virtual ~OverallThroughput();
    void start(size_t payload_size, size_t messages);
    bool next(mavlink_message_t* msg);
    void drop(size_t bytes);
    void check_in(mavlink_message_t* msg);
};

class ThroughputMonitor
{
protected:
    OverallThroughput overall {};
    mavlink_message_t tx_msg {}, rx_msg {};
    mavlink_status_t  rx_status {};
    uint8_t           tx_buf[MAVLINK_MAX_PACKET_LEN] {};
    uint8_t           rx_buf[8192] {};

    virtual void      send(uint8_t* buf, size_t len) = 0;
    virtual ssize_t   recv(uint8_t* buf, size_t len) = 0;

    void one_pass(size_t payload_size, size_t messages);

public:
    ThroughputMonitor()          = default;
    virtual ~ThroughputMonitor() = default;
    void run(size_t n, size_t payload_sz_start, size_t payload_sz_end,
        size_t payload_sz_step);
    OverallThroughput& get_metrics() { return overall; }
};

#include <sys/socket.h>

class SerialThroughputMonitor : public ThroughputMonitor
{
protected:
    int fd_send {-1}, fd_recv {-1}, epoll_recv{-1};
    sockaddr sa_send {};
    void send(uint8_t* buf, size_t len) override;
    ssize_t recv(uint8_t* buf, size_t len) override;

    static void configure(int fd) ;

public:
    SerialThroughputMonitor(const char* f_send, const char * f_recv);
    ~SerialThroughputMonitor() override;
};


class UDPThroughputMonitor : public ThroughputMonitor
{
protected:
    int fd_send {-1}, fd_recv {-1}, epoll_recv{-1};
    sockaddr sa_send {};
    void send(uint8_t* buf, size_t len) override;
    ssize_t recv(uint8_t* buf, size_t len) override;

public:
    UDPThroughputMonitor(uint16_t port_send, uint16_t port_recv);
    ~UDPThroughputMonitor() override;
};
