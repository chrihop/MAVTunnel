#include "codec_passthrough.h"

static enum mavtunnel_error_t
    codec_passthrough_encode(struct mavtunnel_codec_t* ctx, mavlink_message_t* msg)
{
    return MERR_OK;
}

void codec_passthrough_attach(struct mavtunnel_t* ctx)
{
    ctx->codec.object = NULL;
    ctx->codec.encode = codec_passthrough_encode;
}
