#include "os.h"
#include "tunnel.h"
#include "endpoint_linux_uart.h"
#include "codec_passthrough.h"
#include "codec_chacha20.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <termios.h>
#include <threads.h>
#include <signal.h>

struct mavtunnel_t up, down;
struct endpoint_linux_uart_t ep_sitl, ep_gcs;
struct stream_cipher_t encrypt, decrypt;

static atomic_bool to_exit = ATOMIC_VAR_INIT(false);

static int tunnel_thread(void * tunnel)
{
    if (tunnel == NULL)
    {
        return -1;
    }

    struct mavtunnel_t * ctx = (struct mavtunnel_t *)tunnel;
    if (ctx->mode != MT_STATUS_OK)
    {
        return -1;
    }

    enum mavtunnel_error_t err = MERR_OK;
    while (err != MERR_END && !atomic_load(&to_exit))
    {
        err = mavtunnel_spin_once(ctx);
    }

    atomic_store(&to_exit, true);
    return 0;
}

static void sig_int(int signum)
{
    (void)signum;
    atomic_store(&to_exit, true);
    ep_linux_uart_interrupt(&ep_sitl);
    ep_linux_uart_interrupt(&ep_gcs);
}

int main(int argc, char ** argv)
{
    mavtunnel_init(&up, 0);
    mavtunnel_init(&down, 1);
    ep_linux_uart_init(&ep_sitl, "/dev/ttyAMA1");
    ep_linux_uart_init(&ep_gcs, "/dev/ttyAMA2");

    ep_linux_uart_attach_reader(&up, &ep_sitl);
    ep_linux_uart_attach_writer(&up, &ep_gcs);
    codec_chacha20_attach(&up, &encrypt);

    ep_linux_uart_attach_reader(&down, &ep_gcs);
    ep_linux_uart_attach_writer(&down, &ep_sitl);
    codec_chacha20_attach(&down, &decrypt);

    signal(SIGINT, sig_int);
    atomic_store(&to_exit, false);
    thrd_t up_thread, down_thread;
    if (thrd_create(&up_thread, tunnel_thread, &up) != thrd_success)
    {
        printf("failed to create up thread\n");
        return -1;
    }

    if (thrd_create(&down_thread, tunnel_thread, &down) != thrd_success)
    {
        printf("failed to create down thread\n");
        return -1;
    }

    thrd_join(up_thread, NULL);
    thrd_join(down_thread, NULL);

    ep_linux_uart_destroy(&ep_sitl);
    ep_linux_uart_destroy(&ep_gcs);

    return 0;
}
