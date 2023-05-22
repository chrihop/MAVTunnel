#ifndef MAVTUNNEL_LINUX
#error "This file is only for Linux"
#endif

#include "endpoint_linux_udp.h"

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include <netinet/in.h>
#include <sys/eventfd.h>
#include <sys/socket.h>

#include <v2.0/ardupilotmega/mavlink.h>

static const size_t SOCKADDR_SIZE = sizeof(struct sockaddr);

static enum mavtunnel_error_t
ep_linux_udp_epoll(struct endpoint_linux_udp_t* ep)
{
    int rv;
    ep->epoll = epoll_create1(0);
    if (ep->epoll < 0)
    {
        WARN("Failed to create epoll instance: %s\n", strerror(errno));
        return MERR_DEVICE_ERROR;
    }

    struct epoll_event ev;
    ev.events  = EPOLLIN;
    ev.data.fd = ep->fd;
    rv         = epoll_ctl(ep->epoll, EPOLL_CTL_ADD, ep->fd, &ev);
    if (rv < 0)
    {
        WARN("Failed to add socket to epoll: %s\n", strerror(errno));
        return MERR_DEVICE_ERROR;
    }

    ep->terminate_fd = eventfd(0, EFD_NONBLOCK);
    if (ep->terminate_fd < 0)
    {
        WARN("Failed to create eventfd: %s\n", strerror(errno));
        return MERR_DEVICE_ERROR;
    }
    ev.events  = EPOLLIN;
    ev.data.fd = ep->terminate_fd;
    rv         = epoll_ctl(ep->epoll, EPOLL_CTL_ADD, ep->terminate_fd, &ev);
    if (rv < 0)
    {
        WARN("Failed to add eventfd to epoll: %s\n", strerror(errno));
        return MERR_DEVICE_ERROR;
    }
    return MERR_OK;
}

enum mavtunnel_error_t
ep_linux_udp_init(struct endpoint_linux_udp_t* ep, uint16_t port)
{
    ASSERT(ep != NULL);

    ep->fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (ep->fd == -1)
    {
        WARN("Failed to create socket: %s\n", strerror(errno));
        return MERR_DEVICE_ERROR;
    }

    int rv;
    rv = fcntl(ep->fd, F_SETFL, O_NONBLOCK);
    if (rv < 0)
    {
        WARN("Failed to set socket to non-blocking: %s\n", strerror(errno));
        return MERR_DEVICE_ERROR;
    }

    if (ep_linux_udp_epoll(ep) != MERR_OK)
    {
        return MERR_DEVICE_ERROR;
    }

    struct sockaddr_in addr;
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    rv                   = bind(ep->fd, (struct sockaddr*)&addr, sizeof(addr));
    if (rv < 0)
    {
        WARN("Failed to bind socket port %u: %s\n", port, strerror(errno));
        return MERR_DEVICE_ERROR;
    }

    atomic_store(&ep->has_client, false);
    atomic_store(&ep->terminated, false);
    return MERR_OK;
}

void
ep_linux_udp_destroy(struct endpoint_linux_udp_t* ep)
{
    ASSERT(ep != NULL);

    if (ep->terminate_fd >= 0)
    {
        close(ep->terminate_fd);
    }
    if (ep->epoll >= 0)
    {
        close(ep->epoll);
    }
    if (ep->fd >= 0)
    {
        close(ep->fd);
    }
}

void
ep_linux_udp_interrupt(struct endpoint_linux_udp_t* ep)
{
    eventfd_write(ep->terminate_fd, 1);
}

static ssize_t
ep_linux_udp_read(struct mavtunnel_reader_t* rd, uint8_t* bytes, size_t len)
{
    ASSERT(rd != NULL && rd->object != NULL);
    ASSERT(bytes != NULL);

    struct endpoint_linux_udp_t* ep;
    ep = (struct endpoint_linux_udp_t*)rd->object;
    int n_events;
    n_events = epoll_wait(ep->epoll, ep->event, 2, -1);
    if (n_events < 0)
    {
        WARN("Failed to wait for epoll events: %s\n", strerror(errno));
        atomic_store(&ep->terminated, true);
        return -MERR_END;
    }
    else if (n_events == 0)
    {
        return 0;
    }

    if (ep->event[0].data.fd == ep->terminate_fd)
    {
        atomic_store(&ep->terminated, true);
        return -MERR_END;
    }

    ssize_t   n;
    socklen_t client_len = SOCKADDR_SIZE;
    n = recvfrom(ep->fd, bytes, len, 0, &ep->client, &client_len);
    if (n < 0)
    {
        WARN("Failed to read from socket: %s\n", strerror(errno));
        atomic_store(&ep->terminated, true);
        return -MERR_END;
    }
    if (!atomic_load(&ep->has_client))
    {
        INFO("Client connected <--> %s:%u\n",
            inet_ntoa(((struct sockaddr_in*)&ep->client)->sin_addr),
            ntohs(((struct sockaddr_in*)&ep->client)->sin_port));
        atomic_store(&ep->has_client, true);
    }

    return n;
}

static enum mavtunnel_error_t
ep_linux_udp_write(struct mavtunnel_writer_t* wr, mavlink_message_t* msg)
{
    ASSERT(wr != NULL && wr->object != NULL);
    ASSERT(msg != NULL);

    struct endpoint_linux_udp_t* ep;
    ep = (struct endpoint_linux_udp_t*)wr->object;

    if (!atomic_load(&ep->has_client))
    {
        return MERR_OK;
    }

    size_t  len = mavtunnel_finalize_message(ep->out, msg);
    ssize_t n   = sendto(ep->fd, ep->out, len, 0, &ep->client, SOCKADDR_SIZE);
    if (n < 0)
    {
        WARN("Failed to write to socket: %s\n", strerror(errno));
        atomic_store(&ep->terminated, true);
        return MERR_DEVICE_ERROR;
    }
    return MERR_OK;
}

void
ep_linux_udp_attach_reader(
    struct mavtunnel_t* tunnel, struct endpoint_linux_udp_t* ep)
{
    ASSERT(tunnel != NULL);
    ASSERT(ep != NULL);

    tunnel->reader.read   = ep_linux_udp_read;
    tunnel->reader.object = ep;
}

void
ep_linux_udp_attach_writer(
    struct mavtunnel_t* tunnel, struct endpoint_linux_udp_t* ep)
{
    ASSERT(tunnel != NULL);
    ASSERT(ep != NULL);

    tunnel->writer.write  = ep_linux_udp_write;
    tunnel->writer.object = ep;
}
