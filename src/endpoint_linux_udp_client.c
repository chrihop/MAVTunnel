#include "endpoint_linux_udp_client.h"

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include <netinet/in.h>
#include <sys/eventfd.h>
#include <sys/socket.h>

#include <v2.0/ardupilotmega/mavlink.h>
static const size_t SOCKADDR_SIZE = sizeof(struct sockaddr);


static enum mavtunnel_error_t
ep_linux_udp_epoll(struct endpoint_linux_udp_client_t* ep)
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
ep_linux_udp_client_init(
    struct endpoint_linux_udp_client_t* ep, const char* ip, uint16_t port)
{
    ASSERT(ep != NULL);
    ASSERT(ip != NULL);

    ep->fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (ep->fd < 0)
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

    memset(&ep->server, 0, sizeof(ep->server));
    ep->server.sa_family = AF_INET;
    inet_pton(AF_INET, ip, &((struct sockaddr_in*)&ep->server)->sin_addr);
    ((struct sockaddr_in*)&ep->server)->sin_port = htons(port);

    uint8_t buf[1] = { 0 };
    ssize_t n      = sendto(ep->fd, buf, 1, 0, &ep->server, SOCKADDR_SIZE);
    if (n < 0)
    {
        WARN("Failed to connect to server %s:%u: %s\n", ip, port,
            strerror(errno));
        return MERR_DEVICE_ERROR;
    }

    struct sockaddr_in addr;
    socklen_t          addr_len = sizeof(struct sockaddr_in);
    rv = getsockname(ep->fd, (struct sockaddr*)&addr, &addr_len);
    if (rv < 0)
    {
        WARN("Failed to get socket name: %s\n", strerror(errno));
        return MERR_DEVICE_ERROR;
    }

    INFO("Connected to server %u <--> %s:%u\n", ntohs(addr.sin_port), ip, port);

    enum mavtunnel_error_t err;
    err = ep_linux_udp_epoll(ep);
    atomic_store(&ep->terminated, false);
    return err;
}

void
ep_linux_udp_client_destroy(struct endpoint_linux_udp_client_t* ep)
{
    ASSERT(ep != NULL);
    close(ep->fd);
    close(ep->epoll);
    close(ep->terminate_fd);
}

void
ep_linux_udp_client_interrupt(struct endpoint_linux_udp_client_t* ep)
{
    eventfd_write(ep->terminate_fd, 1);
}

static ssize_t
ep_linux_client_read(struct mavtunnel_reader_t* rd, uint8_t* bytes, size_t len)
{
    ASSERT(rd != NULL);
    ASSERT(bytes != NULL);
    ASSERT(rd->object != NULL);

    struct endpoint_linux_udp_client_t* ep;
    ep = rd->object;
    int n_events;
    n_events = epoll_wait(ep->epoll, ep->event, 2, -1);
    if (n_events < 0)
    {
        WARN("Failed to wait for events: %s\n", strerror(errno));
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

    ssize_t n;
    n = recvfrom(ep->fd, bytes, len, 0, NULL, NULL);
    if (n < 0)
    {
        WARN("Failed to read from socket: %s\n", strerror(errno));
        atomic_store(&ep->terminated, true);
        return -MERR_END;
    }

    return n;
}

static enum mavtunnel_error_t
ep_linux_udp_client_write(struct mavtunnel_writer_t* wr, mavlink_message_t* msg)
{
    ASSERT(wr != NULL);
    ASSERT(msg != NULL);
    ASSERT(wr->object != NULL);

    struct endpoint_linux_udp_client_t* ep;
    ep = wr->object;

    size_t  len = mavtunnel_finalize_message(ep->out, msg);
    ssize_t n;
    n = sendto(ep->fd, ep->out, len, 0, &ep->server, SOCKADDR_SIZE);
    if (n < 0)
    {
        WARN("Failed to write to socket: %s\n", strerror(errno));
        atomic_store(&ep->terminated, true);
        return MERR_DEVICE_ERROR;
    }

    return MERR_OK;
}

void
ep_linux_udp_client_attach_reader(
    struct mavtunnel_t* tunnel, struct endpoint_linux_udp_client_t* ep)
{
    ASSERT(tunnel != NULL);
    ASSERT(ep != NULL);

    tunnel->reader.read   = ep_linux_client_read;
    tunnel->reader.object = ep;
}

void
ep_linux_udp_client_attach_writer(
    struct mavtunnel_t* tunnel, struct endpoint_linux_udp_client_t* ep)
{
    ASSERT(tunnel != NULL);
    ASSERT(ep != NULL);

    tunnel->writer.write  = ep_linux_udp_client_write;
    tunnel->writer.object = ep;
}
