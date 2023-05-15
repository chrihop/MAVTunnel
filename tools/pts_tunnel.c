#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <pty.h>
#include <errno.h>
#include <signal.h>

static int create_pts(char * link)
{
    int master, slave;
    char pts[256];

    if (openpty(&master, &slave, pts, NULL, NULL) < 0)
    {
        perror("openpty");
        exit(1);
    }

    if (symlink(pts, link) < 0)
    {
        perror("symlink");
        exit(1);
    }

    return master;
}

struct virtual_uart_t
{
    char * devs[2];
    int fds[2];
    uint8_t buffer[4096];
};

static struct virtual_uart_t uart;

#define max(x, y)   ((x) > (y)? (x) : (y))

static int transfer(size_t from, size_t to, fd_set * masters)
{
    int fd_from = uart.fds[from];
    int fd_to = uart.fds[to];
    if (FD_ISSET(fd_from, masters))
    {
        ssize_t n = read(fd_from, uart.buffer, sizeof(uart.buffer));
        if (n < 0)
        {
            printf("error occurs when read from %s [%s]\n",
                uart.devs[from], strerror(errno));
            return -1;
        }
        write(fd_to, uart.buffer, n);
    }
    return 0;
}

static void spin()
{
    fd_set masters;
    while (true)
    {
        FD_ZERO(&masters);
        FD_SET(uart.fds[0], &masters);
        FD_SET(uart.fds[1], &masters);

        select(max(uart.fds[0], uart.fds[1]) + 1, &masters, NULL, NULL, NULL);
        if (transfer(0, 1, &masters) < 0) {break;}
        if (transfer(1, 0, &masters) < 0) {break;}
    }
}

static void cleanup()
{
    close(uart.fds[0]);
    close(uart.fds[1]);
    unlink(uart.devs[0]);
    unlink(uart.devs[1]);
}

static void pts()
{
    uart.fds[0] = create_pts(uart.devs[0]);
    uart.fds[1] = create_pts(uart.devs[1]);
    spin();
    cleanup();
}

void handle_sigint(int sig)
{
    cleanup();
    exit(0);
}

int main(int argc, char ** argv)
{
    if (argc != 3)
    {
        printf("error: %s <filename> <filename>\n", argv[0]);
        exit(0);
    }
    uart.devs[0] = argv[1];
    uart.devs[1] = argv[2];

    signal(SIGINT, handle_sigint);

    pts();

    return 0;
}
