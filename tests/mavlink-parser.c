#include "os.h"

#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <sys/epoll.h>
#include <termios.h>
#include <threads.h>
#include <unistd.h>

#include <v2.0/ardupilotmega/mavlink.h>

int
main(int argc, char** argv)
{
    int fd = open("./uart-sitl-tunnel", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0)
    {
        printf("failed to open uart-sitl-tunnel\n");
        return -1;
    }

    struct termios tio;
    if (tcgetattr(fd, &tio) < 0)
    {
        printf("failed to get uart-sitl-tunnel attributes\n");
        return -1;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= CS8 | CLOCAL | CREAD;
    tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);

    if (tcsetattr(fd, TCSANOW, &tio) < 0)
    {
        printf("failed to set uart-sitl-tunnel attributes\n");
        return -1;
    }

    int epfd = epoll_create1(0);
    if (epfd < 0)
    {
        printf("failed to create epoll instance\n");
        return -1;
    }

    struct epoll_event ev;
    ev.events  = EPOLLIN;
    ev.data.fd = fd;
    if (epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ev) < 0)
    {
        printf("failed to add uart-sitl-tunnel to epoll\n");
        return -1;
    }

    uint8_t buf[1024];
    while (true)
    {
        if (epoll_wait(epfd, &ev, 1, -1) < 0)
        {
            printf("failed to wait for uart-sitl-tunnel\n");
            return -1;
        }

        ssize_t n = read(fd, buf, sizeof(buf));
        if (n < 0)
        {
            printf("failed to read from uart-sitl-tunnel\n");
            return -1;
        }
        else if (n == 0)
        {
            printf("EOF\n");
            break;
        }

        puthex(buf, n);

        mavlink_message_t msg;
        mavlink_status_t  status;

        for (int i = 0; i < n; i++)
        {
            if (mavlink_parse_char(0, buf[i], &msg, &status))
            {
                printf("msg id: %d, size %d, seq %d, compo %d\n", msg.msgid,
                    msg.len, msg.seq, msg.compid);
            }

            if (status.packet_rx_drop_count > 0)
            {
                printf(">>>>>>>>>>>>>>>>>>>> drop %d\n",
                    status.packet_rx_drop_count);
            }
        }
    }

    exit(0);
}
