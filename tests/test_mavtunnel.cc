#include <gtest/gtest.h>
#include <codec_chacha20.h>

#include <vector>
#include <numeric>
#include "tunnel.h"

struct mavtunnel_t A, B;
struct stream_cipher_t encoder, decoder;

class TestMavtunnel : public ::testing::Test
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

TEST_F(TestMavtunnel, outbuffer)
{
    mavlink_message_t msg;

    mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_ARDUPILOTMEGA, 1, 2, 3);
    EXPECT_EQ(msg.len, 9);

    mavlink_message_t plaintext_msg;
    memcpy(&plaintext_msg, &msg, sizeof(msg));

    A.codec.encode(&A.codec, &msg);
    EXPECT_EQ(msg.len, 9);
    EXPECT_NE(memcmp(&msg, &plaintext_msg, sizeof(msg)), 0);

    uint8_t outbuffer[MAVLINK_MAX_PACKET_LEN];
    size_t outbuffer_len;
    outbuffer_len = mavtunnel_finalize_message(outbuffer, &msg);
    EXPECT_EQ(outbuffer[0], plaintext_msg.magic);
    EXPECT_EQ(outbuffer[1], plaintext_msg.len);
    EXPECT_EQ(outbuffer[2], plaintext_msg.incompat_flags);
    EXPECT_EQ(outbuffer[3], plaintext_msg.compat_flags);
    EXPECT_EQ(outbuffer[4], plaintext_msg.seq);
    EXPECT_EQ(outbuffer[5], plaintext_msg.sysid);
    EXPECT_EQ(outbuffer[6], plaintext_msg.compid);
    EXPECT_EQ(outbuffer[7], plaintext_msg.msgid && 0xFF);
    EXPECT_EQ(outbuffer[8], (plaintext_msg.msgid >> 8) && 0xFF);
    EXPECT_EQ(outbuffer[9], (plaintext_msg.msgid >> 16) && 0xFF);

    mavtunnel_error_t err = mavtunnel_check_out_buffer(outbuffer, outbuffer_len);
    EXPECT_EQ(err, MERR_OK);

    outbuffer_len = mavtunnel_finalize_message(outbuffer, &msg);
    outbuffer[11] ++;
    err = mavtunnel_check_out_buffer(outbuffer, outbuffer_len);
    EXPECT_EQ(err, MERR_BAD_MESSAGE);

    outbuffer_len = mavtunnel_finalize_message(outbuffer, &msg);
    outbuffer_len --;
    err = mavtunnel_check_out_buffer(outbuffer, outbuffer_len);
    EXPECT_EQ(err, MERR_BAD_MESSAGE);

    outbuffer_len = mavtunnel_finalize_message(outbuffer, &msg);
    outbuffer_len ++;
    err = mavtunnel_check_out_buffer(outbuffer, outbuffer_len);
    EXPECT_EQ(err, MERR_BAD_MESSAGE);
}
