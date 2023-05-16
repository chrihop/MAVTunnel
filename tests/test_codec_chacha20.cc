#include <gtest/gtest.h>
#include <codec_chacha20.h>

#include <vector>
#include <numeric>
#include "tunnel.h"

struct mavtunnel_t A, B;
struct stream_cipher_t encoder, decoder;

class CodecChacha20Test : public ::testing::Test
{
public:
    void SetUp() override
    {
        mavtunnel_init(&A, 0);
        mavtunnel_init(&B, 0);

        codec_chacha20_attach(&A, &encoder);
        codec_chacha20_attach(&B, &decoder);
    }
    void TearDown() override
    {
    }
};

TEST_F(CodecChacha20Test, encdec)
{
    mavlink_message_t msg;

    mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_ARDUPILOTMEGA, 1, 2, 3);
    EXPECT_EQ(msg.len, 9);

    mavlink_message_t plaintext_msg;
    memcpy(&plaintext_msg, &msg, sizeof(msg));

    A.codec.encode(&A.codec, &msg);
    EXPECT_EQ(msg.len, 9);
    EXPECT_NE(memcmp(&msg, &plaintext_msg, sizeof(msg)), 0);

    mavlink_message_t decoded_msg;
    memcpy(&decoded_msg, &msg, sizeof(msg));
    B.codec.encode(&B.codec, &msg);
    EXPECT_EQ(msg.len, 9);
    EXPECT_EQ(memcmp(&msg, &plaintext_msg, sizeof(msg)), 0);
}

TEST_F(CodecChacha20Test, enc_transfer_parse_dec)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_ARDUPILOTMEGA, 1, 2, 3);
    EXPECT_EQ(msg.len, 9);

    mavlink_message_t plaintext_msg;
    memcpy(&plaintext_msg, &msg, sizeof(msg));

    A.codec.encode(&A.codec, &msg);
    EXPECT_EQ(msg.len, 9);
    EXPECT_NE(memcmp(&msg, &plaintext_msg, sizeof(msg)), 0);

    vector<uint8_t> internal(255);
    int n = mavlink_msg_to_send_buffer(internal.data(), &msg);
    EXPECT_LT(n, 255);

    mavlink_message_t recv_msg;
    mavlink_status_t  recv_status;
    for (int i = 0; i < n; i++)
    {
        if (mavlink_parse_char(MAVLINK_COMM_0, internal[i], &recv_msg, &recv_status))
        {
            break;
        }
    }
    EXPECT_EQ(recv_status.packet_rx_drop_count, 0);
    EXPECT_EQ(recv_msg.len, 9);

    B.codec.encode(&B.codec, &recv_msg);
    EXPECT_EQ(memcmp(recv_msg.payload64, plaintext_msg.payload64, msg.len), 0);
}
