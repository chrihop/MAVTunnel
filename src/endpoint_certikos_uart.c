#include "endpoint_certikos_uart.h"
#include <syscall.h>

void ep_certikos_uart_init(struct endpoint_certikos_uart_t * ep, size_t uart)
{
    ep->terminated = FALSE;
    ep->dev = uart;
    sys_device_control(uart, DEV_OPEN_CONSOLE, 0, (size_t *) &ep->stream);
}

static ssize_t ep_certikos_uart_read(struct mavtunnel_reader_t * rd,
    uint8_t * bytes, size_t len)
{
    ASSERT(rd != NULL);
    ASSERT(rd->object != NULL);
    struct endpoint_certikos_uart_t * ep = rd->object;

    if (ep->terminated)
    {
        return -MERR_END;
    }

    size_t n = reads(ep->stream, bytes, len);

    return (ssize_t) n;
}

static enum mavtunnel_error_t
    ep_certikos_uart_write(struct mavtunnel_writer_t * wr, const uint8_t * bytes, size_t len)
{
    ASSERT(wr != NULL);
    ASSERT(wr->object != NULL);
    struct endpoint_certikos_uart_t * ep = wr->object;

    if (ep->terminated)
    {
        return MERR_END;
    }

    int err = writes(ep->stream, bytes, len);
    return err == SYS_E_SUCC ? MERR_OK : MERR_DEVICE_ERROR;
}

void ep_certikos_uart_attach_reader(struct mavtunnel_t * tunnel, struct endpoint_certikos_uart_t * ep)
{
    ASSERT(tunnel != NULL);
    ASSERT(ep != NULL);

    tunnel->reader.read = ep_certikos_uart_read;
    tunnel->reader.object = ep;
}

void ep_certikos_uart_attach_writer(struct mavtunnel_t * tunnel, struct endpoint_certikos_uart_t * ep)
{
    ASSERT(tunnel != NULL);
    ASSERT(ep != NULL);

    tunnel->writer.write = ep_certikos_uart_write;
    tunnel->writer.object = ep;
}
