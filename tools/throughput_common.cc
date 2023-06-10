

#include "throughput_common.hpp"

ThroughputEntry::ThroughputEntry(size_t payload_size)
    : payload_size(payload_size)
{
    start = std::chrono::high_resolution_clock::now();
}

void
ThroughputEntry::finish(
    size_t total_bytes, size_t total_msgs, size_t total_drops)
{
    this->end         = std::chrono::high_resolution_clock::now();
    this->total_bytes = total_bytes;
    this->total_msgs  = total_msgs;
    this->total_drops = total_drops;
}

uint64_t
ThroughputEntry::throughput_bps()
{
    double ns = static_cast<double>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            this->end - this->start)
            .count());
    return (uint64_t)((double)total_bytes * 1e9 / (double)ns);
}

uint64_t
ThroughputEntry::throughput_msgps()
{
    double ns = static_cast<double>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            this->end - this->start)
            .count());
    return (uint64_t)((double)total_msgs * 1e9 / (double)ns);
}

uint64_t
ThroughputEntry::drop_rate_bps()
{
    double ns = static_cast<double>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            this->end - this->start)
            .count());
    return (uint64_t)((double)total_drop_bytes * 1e9 / (double)ns);
}

double
ThroughputEntry::drop_rate_percent()
{
    return ((double)total_drops / (double)total_msgs * 100.0);
}

OverallThroughput::~OverallThroughput()
{
    for (auto entry : entries)
    {
        delete entry;
    }
}

void
OverallThroughput::start(size_t payload_size, size_t messages)
{
    this->payload_size   = payload_size;
    this->total_messages = messages;
    this->tx_count       = 0;
    this->rx_count       = 0;
    this->rx_seq         = 1;
    memset(&this->status, 0, sizeof(this->status));
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    this->data.length = payload_size;
    memset(data.data, 0, sizeof(data.data));
    std::iota(data.data, data.data + payload_size, 0);

    entries.push_back(new ThroughputEntry(this->payload_size));
}

bool
OverallThroughput::next(mavlink_message_t* msg)
{
    if (tx_count < total_messages)
    {
        this->data.sequence = tx_count;
        mavlink_msg_logging_data_encode(1, 1, msg, &data);
        mavlink_finalize_message_chan(msg, 1, 2, MAVLINK_COMM_0,
            mavlink_min_message_length(msg),
            MAVLINK_MSG_ID_LOGGING_DATA_LEN, mavlink_get_crc_extra(msg));
        tx_count++;
        return true;
    }

    this->entries.back()->finish(
        rx_count * (MAVLINK_NUM_NON_PAYLOAD_BYTES + payload_size), tx_count,
        tx_count - rx_count);
    return false;
}

void
OverallThroughput::drop(size_t bytes)
{
    entries.back()->total_drop_bytes += bytes;
}

void
OverallThroughput::check_in(mavlink_message_t* msg)
{
    rx_count++;
    int d = ((int)msg->seq - (int)rx_seq + 256) % 256;
    if (d > 1)
    {
        entries.back()->total_drops += d - 1;
    }
    rx_seq = (msg->seq + 1) % 256;
}

void
ThroughputMonitor::one_pass(size_t payload_sz, size_t n)
{
    memset(&rx_status, 0, sizeof(rx_status));
    mavlink_reset_channel_status(MAVLINK_COMM_1);
    overall.start(payload_sz, n);
    while (overall.next(&tx_msg))
    {
        size_t len = mavlink_msg_to_send_buffer(tx_buf, &tx_msg);
        send(tx_buf, len);

        while (len > 0)
        {
            ssize_t rv = recv(rx_buf, sizeof(rx_buf));
            if (rv < 0)
            {
                overall.drop(len);
                break;
            }
            len -= rv;

            size_t start_pos = 0;
            for (size_t i = 0; i < rv; i++)
            {
                if (mavlink_parse_char(MAVLINK_COMM_1, rx_buf[i], &rx_msg, &rx_status))
                {
                    overall.check_in(&rx_msg);
                    start_pos = i;
                    continue;
                }
                if (rx_status.packet_rx_drop_count > 0)
                {
                    overall.drop(start_pos - i);
                    start_pos = i;
                }
            }
        }
    }

    ThroughputEntry& e = *overall.entries.back();
    printf("Payload size: %zu Throughput %lu B/s %lu msg/s Drop Rate %lu B/s "
           "Drop %.2f %%\n",
        payload_sz, e.throughput_bps(), e.throughput_msgps(), e.drop_rate_bps(),
        e.drop_rate_percent());
}

void
ThroughputMonitor::run(size_t n, size_t payload_sz_start, size_t payload_sz_end,
    size_t payload_sz_step)
{
    printf("warmup ...\n");
    size_t warmup = 3;
    while (warmup-- > 0)
    {
        one_pass(100, 100);
    }

    for (size_t payload_sz = payload_sz_start; payload_sz <= payload_sz_end;
         payload_sz += payload_sz_step)
    {
        one_pass(payload_sz, n);
    }
}

#include <fcntl.h>
#include <stdexcept>
#include <sys/epoll.h>
#include <termios.h>
#include <unistd.h>

void
SerialThroughputMonitor::configure(int fd)
{
    termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0)
    {
        throw std::runtime_error("Failed to get serial port attributes");
    }

    cfmakeraw(&tty);
    tty.c_cflag |= CS8 | CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        throw std::runtime_error("Failed to set serial port attributes");
    }
}

SerialThroughputMonitor::SerialThroughputMonitor(
    const char* f_send, const char* f_recv)
{
    fd_send = ::open(f_send, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_send < 0)
    {
        throw std::runtime_error("Failed to open serial port");
    }
    configure(fd_send);

    fd_recv = ::open(f_recv, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_recv < 0)
    {
        throw std::runtime_error("Failed to open serial port");
    }
    configure(fd_recv);

    epoll_recv = epoll_create1(0);
    if (epoll_recv < 0)
    {
        throw std::runtime_error("Failed to create epoll");
    }

    epoll_event ev {};
    ev.events  = EPOLLIN;
    ev.data.fd = fd_recv;
    if (epoll_ctl(epoll_recv, EPOLL_CTL_ADD, fd_recv, &ev) < 0)
    {
        throw std::runtime_error("Failed to add fd to epoll");
    }
}

SerialThroughputMonitor::~SerialThroughputMonitor()
{
    if (fd_send >= 0)
    {
        ::close(fd_send);
    }
    if (fd_recv >= 0)
    {
        ::close(fd_recv);
    }
    if (epoll_recv >= 0)
    {
        ::close(epoll_recv);
    }
}

void
SerialThroughputMonitor::send(uint8_t* buf, size_t len)
{
    ::write(fd_send, buf, len);
}

ssize_t
SerialThroughputMonitor::recv(uint8_t* buf, size_t len)
{
    epoll_event epoll_event[1] {};
    int         n_events = epoll_wait(epoll_recv, epoll_event, 1, 100);
    if (n_events < 0)
    {
        throw std::runtime_error("Failed to wait for epoll");
    }
    else if (n_events == 0 || epoll_event[0].data.fd != fd_recv)
    {
        return -1;
    }

    ssize_t n = ::read(fd_recv, buf, len);
    return n;
}

#include <arpa/inet.h>
#include <sys/socket.h>

UDPThroughputMonitor::UDPThroughputMonitor(
    const uint16_t port_send, const uint16_t port_recv)
{
    fd_send = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_send < 0)
    {
        throw std::runtime_error("Failed to create socket");
    }

    fd_recv = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_recv < 0)
    {
        throw std::runtime_error("Failed to create socket");
    }

    sockaddr_in addr {};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(port_recv);
    if (bind(fd_recv, (sockaddr*)&addr, sizeof(addr)) < 0)
    {
        throw std::runtime_error("Failed to bind socket");
    }

    addr.sin_port = htons(port_send);
    if (bind(fd_send, (sockaddr*)&addr, sizeof(addr)) < 0)
    {
        throw std::runtime_error("Failed to bind socket");
    }

    uint8_t   buf[1];
    socklen_t len = sizeof(sa_send);
    printf("Waiting for connection ...\n");
    ssize_t rv = ::recvfrom(fd_send, buf, 1, 0, (sockaddr*)&sa_send, &len);
    if (rv < 0)
    {
        throw std::runtime_error("Failed to establish connection");
    }
    printf(
        "client %s connected\n", inet_ntoa(((sockaddr_in*)&sa_send)->sin_addr));

    epoll_recv = epoll_create1(0);
    if (epoll_recv < 0)
    {
        throw std::runtime_error("Failed to create epoll");
    }

    epoll_event ev {};
    ev.events  = EPOLLIN;
    ev.data.fd = fd_recv;
    if (epoll_ctl(epoll_recv, EPOLL_CTL_ADD, fd_recv, &ev) < 0)
    {
        throw std::runtime_error("Failed to add fd to epoll");
    }
}

UDPThroughputMonitor::~UDPThroughputMonitor()
{
    if (fd_send >= 0)
    {
        ::close(fd_send);
    }
    if (fd_recv >= 0)
    {
        ::close(fd_recv);
    }
    if (epoll_recv >= 0)
    {
        ::close(epoll_recv);
    }
}

void
UDPThroughputMonitor::send(uint8_t* buf, size_t len)
{
    ::sendto(fd_send, buf, len, 0, (sockaddr*)&sa_send, sizeof(sa_send));
}

ssize_t
UDPThroughputMonitor::recv(uint8_t* buf, size_t len)
{
    epoll_event epoll_event[1] {};
    int         n_events = epoll_wait(epoll_recv, epoll_event, 1, 100);
    if (n_events < 0)
    {
        throw std::runtime_error("Failed to wait for epoll");
    }
    else if (n_events == 0 || epoll_event[0].data.fd != fd_recv)
    {
        return -1;
    }

    sockaddr_in addr {};
    socklen_t   addr_len = sizeof(addr);
    ssize_t n = ::recvfrom(fd_recv, buf, len, 0, (sockaddr*)&addr, &addr_len);
    return n;
}
