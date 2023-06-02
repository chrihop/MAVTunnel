#include <v2.0/ardupilotmega/mavlink.h>

#include "os.h"
#include "tunnel.h"

#include "endpoint_certikos_uart.h"

#define DEBUG_MODE 0

void
mavtunnel_init(struct mavtunnel_t* ctx, size_t id)
{
    ASSERT(ctx != NULL);
    ASSERT(id < 4);

    ctx->mode = MT_STATUS_OK;
    ctx->id = id;
    atomic_store(&ctx->terminate, false);
    memset(&ctx->rx_status, 0, sizeof(ctx->rx_status));
    memset(&ctx->tx_status, 0, sizeof(ctx->tx_status));

    mavlink_reset_channel_status(id);

#ifdef MAVTUNNEL_PROFILING
    ctx->last_update_us = time_us();
    ctx->exec_time_us = 0;
    memset(ctx->count, 0, sizeof(ctx->count));
    memset(ctx->last, 0, sizeof(ctx->last));
#endif
}

#ifdef MAVTUNNEL_PROFILING
static const char * perf_metric_name[MAX_MT_PERF_METRICS] =
{
    "rx",
    "rx_byte",
    "drop",
    "seq_err",
    "tx",
    "tx_byte",
};

static void mavtunnel_perf_update(struct mavtunnel_t * ctx)
{
    uint64_t now = time_us();
    if (now < ctx->last_update_us + MAVTUNNEL_UPDATE_INTERVAL_US)
    {
        return;
    }
    uint64_t interval = now - ctx->last_update_us;

    uint64_t rate[MAX_MT_PERF_METRICS];
    for (int i = 0; i < MAX_MT_PERF_METRICS; i++)
    {
        rate[i] = (ctx->count[i] - ctx->last[i]) * 1000000 / interval;
        ctx->last[i] = ctx->count[i];
    }

    uint64_t cpu_load = ctx->exec_time_us * 100000 / interval;
    ctx->exec_time_us = 0;

    INFO("tunnel %ld: CPU %3lu.%03lu%%\n", ctx->id, cpu_load / 1000, cpu_load % 1000);
    printf("\trates: ");
    for (int i = 0; i < MAX_MT_PERF_METRICS; i++)
    {
        printf("%s %3lu.%03lu k/s, ", perf_metric_name[i], rate[i] / 1000,
            rate[i] % 1000);
    }
    printf("\n\ttotals: ");
    for (int i = 0; i < MAX_MT_PERF_METRICS; i++)
    {
        printf("%s %6lu, ", perf_metric_name[i], ctx->count[i]);
    }
    printf("\n");
    ctx->last_update_us = now;
}
#endif

static size_t
mavtunnel_finalize_message(uint8_t * buf, mavlink_status_t * status, mavlink_message_t * msg)
{
    uint8_t crc_extra = mavlink_get_crc_extra(msg);
    size_t min_length = mavlink_min_message_length(msg);
    mavlink_finalize_message_buffer(msg, msg->sysid, msg->compid, status, min_length, msg->len, crc_extra);
    size_t len = mavlink_msg_to_send_buffer(buf, msg);
    return len;
}


enum mavtunnel_error_t
mavtunnel_spin_once(struct mavtunnel_t* ctx)
{
    ASSERT(ctx != NULL);

    int                    rv;
    ssize_t                n;
    enum mavtunnel_error_t err;

    n = ctx->reader.read(
        &ctx->reader, ctx->read_buffer, MAVTUNNEL_READ_BUFFER_SIZE);

    if (n < 0)
    {
        WARN("tunnel %ld failed to read (%zd)\n", ctx->id, n);
        return MERR_END;
    }

#if (DEBUG_MODE == 1)
    puthex(ctx->read_buffer, n);
#endif

#ifdef MAVTUNNEL_PROFILING
    uint64_t exec_start = time_us();
#endif

    for (ssize_t i = 0; i < n; i++)
    {
#ifdef MAVTUNNEL_PROFILING
        ctx->count[MT_PERF_RECV_BYTE]++;
#endif
        rv = mavlink_parse_char(
            ctx->id, ctx->read_buffer[i], &ctx->rx_msg, &ctx->rx_status);

        if (rv == MAVLINK_FRAMING_INCOMPLETE)
        {
            if (ctx->rx_status.parse_error != 0)
            {
                WARN("tunnel %ld: %lu parse errors\n", ctx->id,
                    ctx->rx_status.parse_error);
            }
            if (ctx->rx_status.packet_rx_drop_count != 0)
            {
                ctx->count[MT_PERF_DROP_COUNT] += ctx->rx_status.packet_rx_drop_count;

                WARN("tunnel %ld: %d packets dropped. total dropped %lu\n", ctx->id,
                    ctx->rx_status.packet_rx_drop_count, ctx->count[MT_PERF_DROP_COUNT]);
                continue;
            }
        }
        else if (rv == MAVLINK_FRAMING_OK)
        {
            ctx->count[MT_PERF_RECV_COUNT] ++;
            if ((err = ctx->codec.encode(&ctx->codec, &ctx->rx_msg)) != MERR_OK)
            {
                WARN(
                    "tunnel %ld failed to encode message (%d)\n", ctx->id, err);
                continue;
            }

            static uint32_t prev_seq = 0;
            if(ctx->rx_msg.seq != (prev_seq+1)%256)
            {
                //WARN("tunnel %ld: out of order seq %u -> %u.\n", ctx->id, prev_seq, ctx->rx_msg.seq);
                ctx->count[MT_PERF_SEQ_ERR]++;
            }
            prev_seq = ctx->rx_msg.seq;



            ctx->tx_status.current_tx_seq = ctx->rx_msg.seq;
            size_t len = mavtunnel_finalize_message(ctx->tx_buf, &ctx->tx_status, &ctx->rx_msg);

            if ((err = ctx->writer.write(&ctx->writer, ctx->tx_buf, len)) != MERR_OK)
            {
                WARN("tunnel %ld failed to write message (%d)\n", ctx->id, err);
                continue;
            }
#if (DEBUG_MODE == 1)
            INFO("tunnel %ld: send message seq[%d] id[%02x] size[%d]\n",
                ctx->id, ctx->msg.seq, ctx->msg.msgid, ctx->msg.len);
#endif
            ctx->count[MT_PERF_SENT_COUNT] ++;
            ctx->count[MT_PERF_SENT_BYTE] += len;

            static size_t prev_rx_bytes = 0;
            size_t expected_len = ctx->count[MT_PERF_RECV_BYTE] - prev_rx_bytes;
            if(expected_len != len)
            {
                WARN("tunnel %ld: lost %u bytes\n", ctx->id, expected_len - len);
            }
            prev_rx_bytes = ctx->count[MT_PERF_RECV_BYTE];
        }
    }

#ifdef MAVTUNNEL_PROFILING
    ctx->exec_time_us += time_us() - exec_start;
#endif

#ifdef MAVTUNNEL_PROFILING
    mavtunnel_perf_update(ctx);
#endif

    return MERR_OK;
}

void
mavtunnel_spin(struct mavtunnel_t* ctx)
{
    ASSERT(ctx != NULL && ctx->mode == MT_STATUS_OK);
    enum mavtunnel_error_t err = MERR_OK;
    INFO("tunnel %ld start\n", ctx->id);
    while (err != MERR_END && !atomic_load(&ctx->terminate))
    {
        err = mavtunnel_spin_once(ctx);
    }
}

void mavtunnel_exit(struct mavtunnel_t * ctx)
{
    if (ctx != NULL)
    {
        atomic_store(&ctx->terminate, true);
    }
}

