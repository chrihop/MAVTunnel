#ifndef _MAVTUNNEL_ENDPOINT_LINUX_UART_H_
#define _MAVTUNNEL_ENDPOINT_LINUX_UART_H_

#include "os.h"
#include "tunnel.h"

#include <sys/epoll.h>

struct endpoint_linux_uart_t
{
    int            fd;
    int            epoll;
    struct         epoll_event event[2];
    char*          device_path;
    int            terminate_fd;
    atomic_bool    terminated;
    uint8_t        out[4096];
};

#if __cplusplus
extern "C" {
#endif

int ep_linux_uart_init(struct endpoint_linux_uart_t * ep, const char * device_path);

void ep_linux_uart_destroy(struct endpoint_linux_uart_t * ep);

void ep_linux_uart_attach_reader(struct mavtunnel_t * tunnel, struct endpoint_linux_uart_t * ep);

void ep_linux_uart_attach_writer(struct mavtunnel_t * tunnel, struct endpoint_linux_uart_t * ep);

void ep_linux_uart_interrupt(struct endpoint_linux_uart_t * ep);

#if __cplusplus
};
#endif


#endif /* !_MAVTUNNEL_ENDPOINT_LINUX_UART_H_ */
