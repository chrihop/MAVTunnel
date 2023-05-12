#include <v1.0/ardupilotmega/mavlink.h>

#include "os.h"
#include "tunnel.h"

void
mavtunnel_init(struct mavtunnel_t* ctx, struct mavtunnel_reader_t* reader,
    struct mavtunnel_writer_t* writer, struct mavtunnel_codec_t* codec)
{
    ASSERT(ctx != NULL);
    ASSERT(reader != NULL);
    ASSERT(writer != NULL);
    ASSERT(codec != NULL);
    ASSERT(reader->read_byte != NULL);
    ASSERT(writer->write != NULL);
    ASSERT(codec->encode != NULL);

    ctx->reader = reader;
    ctx->writer = writer;
    ctx->codec  = codec;
    ctx->mode   = MT_STATUS_OK;

#ifdef MAVTUNNEL_PROFILING
    ctx->message_count = 0;
    ctx->byte_count    = 0;
    ctx->drop_count    = 0;
#endif
}

enum mavtunnel_error_t
mavtunnel_spin_once(struct mavtunnel_t* ctx)
{
    int                    rv;
    int                    byte;
    int                    drop;
    enum mavtunnel_error_t err;
    if ((byte = ctx->reader->read_byte(ctx->reader)) < 0)
    {
        WARN("tunnel %ld failed to read (%d)\n", ctx->id, byte);
        return MERR_END;
    }

    drop = ctx->status.packet_rx_drop_count;
    rv   = mavlink_parse_char(MAVLINK_COMM_0, byte, &ctx->msg, &ctx->status);
    if (rv == 0)
    {
        if (drop < ctx->status.packet_rx_drop_count)
        {
            ctx->drop_count++;
            WARN("tunnel %ld dropped %d packets\n", ctx->id,
                ctx->status.packet_rx_drop_count - drop);
            return MERR_BAD_MESSAGE;
        }

        ctx->byte_count++;
        return MERR_OK;
    }
    else if (rv == 1)
    {
        ctx->message_count++;
        ctx->byte_count++;
        if ((err = ctx->codec->encode(ctx->codec, &ctx->msg) != MERR_OK))
        {
            WARN("tunnel %ld failed to encode message (%d)\n", ctx->id, err);
            return err;
        }

        if ((err = ctx->writer->write(ctx->writer, &ctx->msg) != MERR_OK))
        {
            WARN("tunnel %ld failed to write message (%d)\n", ctx->id, err);
            return err;
        }
        return MERR_OK;
    }
    return MERR_BAD_MESSAGE;
}

void
mavtunnel_spin(struct mavtunnel_t* ctx)
{
    ASSERT(ctx != NULL && ctx->mode == MT_STATUS_OK);
    enum mavtunnel_error_t err = MERR_OK;
    while (err != MERR_END)
    {
        err = mavtunnel_spin_once(ctx);
    }
}
