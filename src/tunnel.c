#include <v2.0/ardupilotmega/mavlink.h>

#include "os.h"
#include "tunnel.h"

#define DEBUG_MODE 0

void
mavtunnel_init(struct mavtunnel_t* ctx, size_t id)
{
    ASSERT(ctx != NULL);
    ASSERT(id < 4);

    ctx->mode = MT_STATUS_OK;
    ctx->id = id;
    atomic_store(&ctx->terminate, false);
    memset(&ctx->status, 0, sizeof(ctx->status));

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
    "recv",
    "recv_bytes",
    "drop",
    "send",
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

    INFO("tunnel %ld: CPU %3lu.%-3lu%%, ", ctx->id, cpu_load / 1000, cpu_load % 1000);
    for (int i = 0; i < MAX_MT_PERF_METRICS; i++)
    {
        printf("%s rate %3lu.%-3lu k/s, ", perf_metric_name[i], rate[i] / 1000,
            rate[i] % 1000);
    }
    printf("total: ");
    for (int i = 0; i < MAX_MT_PERF_METRICS; i++)
    {
        printf("%s %6lu, ", perf_metric_name[i], ctx->count[i]);
    }
    printf("\n");
    ctx->last_update_us = now;
}
#endif

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
        ctx->count[MT_PERF_RECV_BYTE]++;
        rv = mavlink_parse_char(
            ctx->id, ctx->read_buffer[i], &ctx->msg, &ctx->status);
        if (rv == 0)
        {
            if (ctx->status.packet_rx_drop_count != 0)
            {
                ctx->count[MT_PERF_DROP_COUNT] += ctx->status.packet_rx_drop_count;

                WARN("tunnel %ld: %d packets dropped. total dropped %lu\n", ctx->id,
                    ctx->status.packet_rx_drop_count, ctx->count[MT_PERF_DROP_COUNT]);
                continue;
            }
        }
        else if (rv == 1)
        {
            ctx->count[MT_PERF_RECV_COUNT] ++;
            if ((err = ctx->codec.encode(&ctx->codec, &ctx->msg)) != MERR_OK)
            {
                WARN(
                    "tunnel %ld failed to encode message (%d)\n", ctx->id, err);
                continue;
            }

            if ((err = ctx->writer.write(&ctx->writer, &ctx->msg)) != MERR_OK)
            {
                WARN("tunnel %ld failed to write message (%d)\n", ctx->id, err);
                continue;
            }
#if (DEBUG_MODE == 1)
            INFO("tunnel %ld: send message seq[%d] id[%02x] size[%d]\n",
                ctx->id, ctx->msg.seq, ctx->msg.msgid, ctx->msg.len);
#endif
            ctx->count[MT_PERF_SENT_COUNT] ++;
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
