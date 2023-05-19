#ifndef _ENDPOINT_CERTIKOS_UART_H_
#define _ENDPOINT_CERTIKOS_UART_H_

#include "os.h"
#include "tunnel.h"

#include <types.h>

struct endpoint_certikos_uart_t
{
    size_t          dev;
    console_id_t stream;
    atomic_bool terminated;
    uint8_t     out[MAVTUNNEL_OUTPUT_BUFFER_SIZE];
};

#if __cplusplus
extern "C" {
#endif

void ep_certikos_uart_init(struct endpoint_certikos_uart_t * ep, size_t uart);
void ep_certikos_uart_attach_reader(struct mavtunnel_t * tunnel, struct endpoint_certikos_uart_t * ep);
void ep_certikos_uart_attach_writer(struct mavtunnel_t * tunnel, struct endpoint_certikos_uart_t * ep);

#if __cplusplus
};
#endif


#endif /* _ENDPOINT_CERTIKOS_UART_H_ */
