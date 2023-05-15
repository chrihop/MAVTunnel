#ifndef _CODEC_PASSTHROUGH_H_
#define _CODEC_PASSTHROUGH_H_

#include "os.h"
#include "tunnel.h"

#if __cplusplus
extern "C" {
#endif

void codec_passthrough_attach(struct mavtunnel_t* ctx);

#if __cplusplus
};
#endif


#endif /* !_CODEC_PASSTHROUGH_H_ */
