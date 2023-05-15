#ifndef MAVTUNNEL_LINUX
#error "This file is only for Linux"
#endif

#include "endpoint_linux_uart.h"

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#include <v2.0/ardupilotmega/mavlink.h>

static int
ep_linux_uart_config(struct endpoint_linux_uart_t* ep)
{
    enum mavtunnel_error_t err = MERR_OK;
    struct termios * options = malloc(sizeof(struct termios));

    if (tcgetattr(ep->fd, options) == -1)
    {
        WARN("Failed to get UART device attributes %s\n", ep->device_path);
        err = MERR_DEVICE_ERROR;
        goto cleanup;
    }

    cfmakeraw(options);
    options->c_cflag |= CS8 | CLOCAL | CREAD;
    options->c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    cfsetispeed(options, B115200);
    cfsetospeed(options, B115200);

    if (tcsetattr(ep->fd, TCSANOW, options) != 0)
    {
        WARN("Failed to set UART device options! %s\n", strerror(errno));
        err = MERR_DEVICE_ERROR;
        goto cleanup;
    }

cleanup:
    free(options);
    return err;
}

static int
ep_linux_uart_epoll(struct endpoint_linux_uart_t * ep)
{
    if ((ep->epoll = epoll_create1(0)) < 0)
    {
        WARN("Failed to create epoll instance: %s\n", strerror(errno));
        return MERR_END;
    }

    struct epoll_event ev;
    ev.events = EPOLLIN;
    ev.data.fd = ep->fd;
    if (epoll_ctl(ep->epoll, EPOLL_CTL_ADD, ep->fd, &ev) < 0)
    {
        WARN("Failed to add UART device to epoll: %s\n", strerror(errno));
        return MERR_END;
    }

    return MERR_OK;
}

int
ep_linux_uart_init(struct endpoint_linux_uart_t * ep, const char * device_path)
{
    ASSERT(ep != NULL);
    ASSERT(device_path != NULL);

    ep->device_path = strdup(device_path);
    if ((ep->fd = open(device_path, O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
    {
        WARN("Failed to open UART device %s: %s\n", device_path, strerror(errno));
        return MERR_END;
    }

    ep_linux_uart_config(ep);
    ep_linux_uart_epoll(ep);
    atomic_store(&ep->terminated, false);

    return MERR_OK;
}

static ssize_t
ep_linux_uart_read(struct mavtunnel_reader_t * rd,
    uint8_t * bytes, size_t len)
{
    ASSERT(rd != NULL);
    ASSERT(rd->object != NULL);
    struct endpoint_linux_uart_t * ep = rd->object;

    int n_events;
    if ((n_events = epoll_wait(ep->epoll, ep->event, 1, -1)) < 0)
    {
        WARN("Failed to wait for UART device %s: %s\n", ep->device_path, strerror(errno));
        atomic_store(&ep->terminated, true);
        return -MERR_END;
    } else if (n_events == 0)
    {
        return 0;
    }

    if (ep->event[0].data.fd != ep->fd)
    {
        WARN("Invalid file descriptor returned from epoll\n");
        atomic_store(&ep->terminated, true);
        return -MERR_END;
    }

    ssize_t n = read(ep->fd, bytes, len);
    if (n < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            /* data is not available */
            return 0;
        }
        else
        {
            WARN("Failed to read from UART device %s\n", ep->device_path);
            atomic_store(&ep->terminated, true);
            return -MERR_END;
        }
    }
    return n;
}

static enum mavtunnel_error_t
ep_linux_uart_write(struct mavtunnel_writer_t * wr, mavlink_message_t * msg)
{
    ASSERT(wr != NULL);
    ASSERT(wr->object != NULL);
    struct endpoint_linux_uart_t * ep = wr->object;

    if (atomic_load(&ep->terminated))
    {
        return MERR_END;
    }

    int len = mavlink_msg_to_send_buffer(ep->out, msg);
    ssize_t written = write(ep->fd, ep->out, len);
    if (written == -1 || written < len)
    {
        WARN("Failed to write to UART device %s (%zd/%d)\n", ep->device_path,
            written, len);
        return MERR_DEVICE_ERROR;
    }
    return MERR_OK;
}

void
ep_linux_uart_destroy(struct endpoint_linux_uart_t * ep)
{
    ASSERT(ep != NULL);
    close(ep->fd);
    free(ep->device_path);
}

void
ep_linux_uart_attach_reader(struct mavtunnel_t * tunnel, struct endpoint_linux_uart_t * ep)
{
    ASSERT(tunnel != NULL);
    ASSERT(ep != NULL);

    tunnel->reader.read = ep_linux_uart_read;
    tunnel->reader.object = ep;
}

void
ep_linux_uart_attach_writer(struct mavtunnel_t * tunnel, struct endpoint_linux_uart_t * ep)
{
    ASSERT(tunnel != NULL);
    ASSERT(ep != NULL);

    tunnel->writer.write = ep_linux_uart_write;
    tunnel->writer.object = ep;
}
