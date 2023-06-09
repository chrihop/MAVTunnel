#ifndef _MAVTUNNEL_ENDPOINT_LINUX_UDP_CLIENT_H_
#define _MAVTUNNEL_ENDPOINT_LINUX_UDP_CLIENT_H_

#include "os.h"
#include "tunnel.h"
#include <arpa/inet.h>
#include <sys/epoll.h>

struct endpoint_linux_udp_client_t
{
    int                fd;
    struct sockaddr    server;
    int                epoll, terminate_fd;
    struct epoll_event event[2];
    atomic_bool        terminated;
};

#if __cplusplus
extern "C"
{
#endif

enum mavtunnel_error_t ep_linux_udp_client_init(
    struct endpoint_linux_udp_client_t* ep, const char* ip, uint16_t port);

void ep_linux_udp_client_destroy(struct endpoint_linux_udp_client_t* ep);

void ep_linux_udp_client_interrupt(struct endpoint_linux_udp_client_t* ep);

void ep_linux_udp_client_attach_reader(
    struct mavtunnel_t* tunnel, struct endpoint_linux_udp_client_t* ep);

void ep_linux_udp_client_attach_writer(
    struct mavtunnel_t* tunnel, struct endpoint_linux_udp_client_t* ep);

#if __cplusplus
};
#endif

#endif /* !_MAVTUNNEL_ENDPOINT_LINUX_UDP_CLIENT_H_ */
