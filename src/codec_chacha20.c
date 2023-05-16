#include <mbedtls_abstraction.h>
#include "codec_chacha20.h"

static uint8_t stream_key[32] = {
    0x1f, 0x2e, 0x3d, 0x4c,
    0x5b, 0x6a, 0x79, 0x88,
    0x97, 0xa6, 0xb5, 0xc4,
    0xd3, 0xe2, 0xf1, 0x00,
    0x0f, 0x1e, 0x2d, 0x3c,
    0x4b, 0x5a, 0x69, 0x78,
    0x87, 0x96, 0xa5, 0xb4,
    0xc3, 0xd2, 0xe1, 0xf0};

static uint8_t stream_iv[12] = {
    0x00, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07,
    0x00, 0x00, 0x00, 0x00};

static uint8_t stream_counter = 0;

static void stream_cipher_init(struct stream_cipher_t * ctx)
{
    mbedtls_chacha20_init(&ctx->chacha20);
    mbedtls_call(mbedtls_chacha20_setkey, &ctx->chacha20, stream_key);
}

static void stream_cipher_encode(struct stream_cipher_t * ctx, uint8_t * bytes, size_t len)
{
    ASSERT(len <= sizeof(ctx->buffer));
    mbedtls_call(mbedtls_chacha20_starts, &ctx->chacha20, stream_iv, stream_counter);
    mbedtls_call(mbedtls_chacha20_update, &ctx->chacha20, len, bytes, ctx->buffer);
    memcpy(bytes, ctx->buffer, len);
}

static enum mavtunnel_error_t
codec_chacha20_encode(struct mavtunnel_codec_t * codec, mavlink_message_t * msg)
{
    struct stream_cipher_t * cipher = (struct stream_cipher_t *)codec->object;
    stream_cipher_encode(cipher, (uint8_t *) msg->payload64, msg->len);
    return MERR_OK;
}

void codec_chacha20_attach(struct mavtunnel_t * ctx, struct stream_cipher_t * cipher)
{
    stream_cipher_init(cipher);
    ctx->codec.object = cipher;
    ctx->codec.encode = codec_chacha20_encode;
}
