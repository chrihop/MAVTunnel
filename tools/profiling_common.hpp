#pragma once

#include <cstdint>
#include <vector>
#include <unordered_map>
#include <ctime>
#include <thread>
#include <mutex>
#include <atomic>
#include <string>

#include <v2.0/ardupilotmega/mavlink.h>

static inline unsigned long long time_ns()
{
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (unsigned long long)ts.tv_sec * (int)1e9 + ts.tv_nsec;
}

struct MavlinkMsgHash
{
    size_t operator()(const mavlink_message_t & msg) const
    {
        return std::hash<uint32_t>()(msg.msgid << 8 | msg.seq);
    }
};

struct LatencyEntry
{
    uint32_t msgid;
    uint8_t seqnum;
    size_t key;
    int64_t start{-1}, end{-1};

    LatencyEntry(uint32_t msgid, uint8_t seqnum)
        : msgid(msgid), seqnum(seqnum)
    {
        this->key = std::hash<size_t>()(msgid << 8 | seqnum);
        this->start = (int64_t) time_ns();
    }

    void set_end(uint64_t ts)
    {
        this->end = (int64_t) ts;
    }
};

struct Statistics
{
    uint64_t min{UINT64_MAX}, max{0}, mean{0};
};

class EndToEndLatency
{
protected:
    std::vector<LatencyEntry *> entries;
    std::unordered_map<size_t, LatencyEntry *> ongoing;
    std::mutex mtx;

public:
    EndToEndLatency() = default;
    virtual ~EndToEndLatency()
    {
        for (auto e : entries)
        {
            delete e;
        }
    }

    void markSend(mavlink_message_t * msg)
    {
        auto * e = new LatencyEntry(msg->msgid, msg->seq);

        std::lock_guard<std::mutex> lock(mtx);
        entries.emplace_back(e);
        ongoing[e->key] = e;
    }

    void markReceive(mavlink_message_t * msg)
    {
        auto key = std::hash<size_t>()(msg->msgid << 8 | msg->seq);

        std::lock_guard<std::mutex> lock(mtx);
        auto it = ongoing.find(key);
        if (it != ongoing.end())
        {
            it->second->set_end(time_ns());
            ongoing.erase(it);
        }
    }

    size_t total()
    {
        return entries.size();
    }

    size_t drops() const {
        return ongoing.size();
    }

    Statistics statistics() const
    {
        Statistics s;
        uint64_t sum = 0, count = 0;
        for (auto e : entries)
        {
            if (e->start == -1 || e->end == -1)
            {
                continue;
            }
            auto latency = e->end - e->start;
            s.min = std::min(s.min, (uint64_t) latency);
            s.max = std::max(s.max, (uint64_t) latency);
            sum += latency;
            count++;
        }
        s.mean = -1;
        if (count > 0)
        {
            s.mean = sum / count;
        }
        return s;
    }
};

struct SendRecvStatistics
{
    size_t total_tx{}, total_rx{}, drops{};
    Statistics latency;
};

class SendRecvMonitor
{
protected:
    size_t messages;
    size_t rx_count{0}, tx_count{0};
    std::atomic<bool> running{true};
    std::unique_ptr<std::thread> sender{}, receiver{};
    uint8_t rx_buf[4096]{};
    mavlink_message_t rx_msg{};
    mavlink_status_t rx_status{};

    virtual void send(uint8_t * bytes, size_t len) = 0;
    virtual ssize_t recv(uint8_t * bytes, size_t len) = 0;
    virtual void interrupt_send() = 0;
    virtual void interrupt_recv() = 0;

    void sender_thread()
    {
        printf("sender thread created ...\n");
        for (size_t i = 0; i < messages && running; i++)
        {
            mavlink_command_long_t cmd{};
            cmd.target_system = 1;
            cmd.target_component = 1;
            cmd.command = MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
            cmd.confirmation = 0;
            cmd.param1 = 0;
            cmd.param2 = 0;
            cmd.param3 = 0;
            cmd.param4 = 0;
            cmd.param5 = 0;
            cmd.param6 = 0;
            cmd.param7 = 0;

            mavlink_message_t msg{};
            mavlink_msg_command_long_encode(255, 1, &msg, &cmd);
            mavlink_finalize_message(&msg, 255, 1,
                mavlink_min_message_length(&msg), msg.len,
                mavlink_get_crc_extra(&msg));
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            ssize_t len = mavlink_msg_to_send_buffer(buf, &msg);
            recorder.markSend(&msg);
            this->send(buf, len);
            tx_count++;

            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
    }

    void receiver_thread()
    {
        printf("receiver thread created ...\n");
        while (running && rx_count < messages)
        {
            ssize_t len = this->recv(rx_buf, sizeof(rx_buf));
            if (len > 0)
            {
                for (size_t i = 0; i < len; i++)
                {
                    if (mavlink_parse_char(MAVLINK_COMM_1, rx_buf[i], &rx_msg, &rx_status))
                    {
                        recorder.markReceive(&rx_msg);
                        rx_count++;
                    }
                }
            }
        }
    }

public:
    EndToEndLatency recorder{};

    explicit SendRecvMonitor(size_t n): messages{n}
    {
    }
    virtual ~SendRecvMonitor() = default;

    void start()
    {
        running = true;
        sender = std::make_unique<std::thread>(&SendRecvMonitor::sender_thread, this);
        receiver = std::make_unique<std::thread>(&SendRecvMonitor::receiver_thread, this);
    }

    void stop()
    {
        running = false;
        interrupt_send();
        if (sender)
        {
            sender->join();
        }
        interrupt_recv();
        if (receiver)
        {
            receiver->join();
        }
    }

    void run(uint64_t timeout_ms)
    {
        start();
        while (running && tx_count < messages)
        {
            printf("tx %zu/%zu, rx %zu\n",
                tx_count, messages, rx_count);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        auto timeout_time = time_ns() + timeout_ms * 1000000;
        while (running && rx_count < messages && time_ns() < timeout_time)
        {
            printf("tx %zu, rx %zu/%zu\n",
                tx_count, rx_count, messages);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        if (!receiver->joinable() || !sender->joinable())
        {
            stop();
        }
        else
        {
            sender->join();
            receiver->join();
        }

        Statistics s = recorder.statistics();
        printf("tx %zu, rx %zu, drops %zu, latency (ns): min %zu, max %zu, mean %zu\n",
            tx_count, rx_count, recorder.drops(),
            s.min, s.max, s.mean);
    }

    SendRecvStatistics statistics() const
    {
        return SendRecvStatistics{
            .total_tx = tx_count,
            .total_rx = rx_count,
            .drops = recorder.drops(),
            .latency = recorder.statistics()
        };
    }
};

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class SerialSendRecvMonitor : public SendRecvMonitor
{
private:
    void configure() const
    {
        struct termios tio{};
        tcgetattr(fd_send, &tio);
        cfmakeraw(&tio);
        tio.c_cflag |= CS8 | CLOCAL | CREAD;
        tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
        cfsetispeed(&tio, B115200);
        cfsetospeed(&tio, B115200);
        tcsetattr(fd_send, TCSANOW, &tio);

        tcgetattr(fd_recv, &tio);
        cfmakeraw(&tio);
        tio.c_cflag |= CS8 | CLOCAL | CREAD;
        tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
        cfsetispeed(&tio, B115200);
        cfsetospeed(&tio, B115200);
        tcsetattr(fd_recv, TCSANOW, &tio);
    }

protected:
    int fd_send{-1}, fd_recv{-1};

    void send(uint8_t * bytes, size_t len) override
    {
        ::write(fd_send, bytes, len);
    }

    ssize_t recv(uint8_t * bytes, size_t len) override
    {
        return ::read(fd_recv, bytes, len);
    }

    void interrupt_send() override
    {
        tcdrain(fd_send);
    }

    void interrupt_recv() override
    {
        tcflush(fd_recv, TCIOFLUSH);
    }

public:
    SerialSendRecvMonitor(size_t n, const std::string& dev_send, const std::string& dev_recv)
        : SendRecvMonitor(n)
    {
        fd_send = ::open(dev_send.c_str(), O_WRONLY | O_NOCTTY | O_SYNC);
        if (fd_send < 0)
        {
            throw std::runtime_error("could not open serial device " + dev_send);
        }
        fd_recv = ::open(dev_recv.c_str(), O_RDONLY | O_NOCTTY | O_SYNC);
        if (fd_recv < 0)
        {
            throw std::runtime_error("could not open serial device " + dev_recv);
        }
        configure();
    }
};

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

class UDPSendRecvMonitor : public SendRecvMonitor
{
protected:
    int fd_send{-1}, fd_recv{-1};
    struct sockaddr_in addr{};

    void send(uint8_t * bytes, size_t len) override
    {
        ::sendto(fd_send, bytes, len, 0, (struct sockaddr *)&addr, sizeof(addr));
    }

    ssize_t recv(uint8_t * bytes, size_t len) override
    {
        return ::recvfrom(fd_recv, bytes, len, 0, nullptr, nullptr);
    }

    void interrupt_send() override
    {
    }

    void interrupt_recv() override
    {
    }

public:
    UDPSendRecvMonitor(size_t n, const std::string& ip, uint16_t port, uint16_t port_recv)
        : SendRecvMonitor(n)
    {
        fd_send = ::socket(AF_INET, SOCK_DGRAM, 0);
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        inet_aton(ip.c_str(), &addr.sin_addr);

        fd_recv = ::socket(AF_INET, SOCK_DGRAM, 0);
        struct sockaddr_in addr_recv{};
        addr_recv.sin_family = AF_INET;
        addr_recv.sin_port = htons(port_recv);
        addr_recv.sin_addr.s_addr = INADDR_ANY;
        int rv = ::bind(fd_recv, (struct sockaddr *)&addr_recv, sizeof(addr_recv));
        if (rv < 0)
        {
            throw std::runtime_error("could not bind to port " + std::to_string(port_recv));
        }
    }
};
