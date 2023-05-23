#ifndef _MAVTUNNEL_ENDPOINT_LINUX_UDP_H_
#define _MAVTUNNEL_ENDPOINT_LINUX_UDP_H_

#include "os.h"
#include "tunnel.h"
#include <arpa/inet.h>
#include <sys/epoll.h>

struct endpoint_linux_udp_t
{
    int                fd;
    struct sockaddr    client;
    atomic_bool        has_client;
    int                epoll, terminate_fd;
    struct epoll_event event[2];
    atomic_bool        terminated;
};

#if __cplusplus
extern "C"
{
#endif

enum mavtunnel_error_t
ep_linux_udp_init(struct endpoint_linux_udp_t* ep, uint16_t port);

void ep_linux_udp_destroy(struct endpoint_linux_udp_t* ep);

void ep_linux_udp_interrupt(struct endpoint_linux_udp_t* ep);

void ep_linux_udp_attach_reader(
    struct mavtunnel_t* tunnel, struct endpoint_linux_udp_t* ep);

void ep_linux_udp_attach_writer(
    struct mavtunnel_t* tunnel, struct endpoint_linux_udp_t* ep);

#if __cplusplus
};
#endif

#endif /* !_MAVTUNNEL_ENDPOINT_LINUX_UDP_H_ */
