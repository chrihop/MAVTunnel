#include <gtest/gtest.h>
#include <endpoint_linux_uart.h>
#include <codec_passthrough.h>
#include <pty.h>

#include <vector>
#include <numeric>

struct mavtunnel_t tunnel;
struct endpoint_linux_uart_t ep_a, ep_b;
int master_a, master_b;
const char * device_a = "./uart-a";
const char * device_b = "./uart-b";

static int create_mock_pts(const char * link)
{
    int master, slave;
    char pts[256];

    if (openpty(&master, &slave, pts, nullptr, nullptr) < 0)
    {
        perror("openpty");
        exit(1);
    }

    if (symlink(pts, link) < 0)
    {
        perror("symlink");
        exit(1);
    }

    return master;
}

static void destroy_mock_pts(int master, const char * link)
{
    close(master);
    unlink(link);
}

class EndpointLinuxUartTest : public ::testing::Test
{
public:
    void SetUp() override
    {
        mavtunnel_init(&tunnel, 0);
        master_a = create_mock_pts(device_a);
        master_b = create_mock_pts(device_b);

        ep_linux_uart_init(&ep_a, device_a);
        ep_linux_uart_init(&ep_b, device_b);
    }
    void TearDown() override
    {
        ep_linux_uart_destroy(&ep_a);
        ep_linux_uart_destroy(&ep_b);

        destroy_mock_pts(master_a, device_a);
        destroy_mock_pts(master_b, device_b);
    }
};

TEST_F(EndpointLinuxUartTest, ep_linux_uart_attach_reader)
{
    ep_linux_uart_attach_reader(&tunnel, &ep_a);
    ep_linux_uart_attach_writer(&tunnel, &ep_b);
    codec_passthrough_attach(&tunnel);

    std::vector<uint8_t> data(256);
    std::iota(data.begin(), data.end(), 0);
    std::vector<uint8_t> buffer(1024);

    ssize_t n;
    write(master_a, data.data(), data.size());
    n = tunnel.reader.read(&tunnel.reader, buffer.data(), buffer.size());
    ASSERT_EQ(n, data.size());
    ASSERT_EQ(memcmp(data.data(), buffer.data(), n), 0);

    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(1, 1, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0);
    n = tunnel.writer.write(&tunnel.writer, &msg);
    ASSERT_EQ(n, MERR_OK);

    std::vector<uint8_t> serialized(256);
    ssize_t m = mavlink_msg_to_send_buffer(serialized.data(), &msg);

    n = read(master_b, buffer.data(), buffer.size());
    ASSERT_EQ(n, m);
    ASSERT_EQ(memcmp(buffer.data(), serialized.data(), n), 0);
}
