#ifndef _MAVTUNNEL_CODEC_CHACHA20_H_
#define _MAVTUNNEL_CODEC_CHACHA20_H_

#include <mbedtls/chacha20.h>
#include "tunnel.h"

struct stream_cipher_t
{
    mbedtls_chacha20_context chacha20;
    uint8_t                  buffer[1024];
};

#if __cplusplus
extern "C" {
#endif

void codec_chacha20_attach(struct mavtunnel_t * ctx, struct stream_cipher_t * cipher);

#if __cplusplus
};
#endif


#endif /* !_MAVTUNNEL_CODEC_CHACHA20_H_ */
