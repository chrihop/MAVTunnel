#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <pty.h>
#include <errno.h>
#include <signal.h>
#include <sys/epoll.h>

#include "pts_common.h"

int create_pts(char * link)
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

static int make_transfer(struct pts_junction_t * ctx, int fd_from,
    struct pts_group_t * group_to)
{
    ssize_t n = read(fd_from, ctx->buffer, sizeof(ctx->buffer));
    if (n < 0)
    {
        printf("error occurs when read from %s [%s]\n",
            ctx->up.pts[fd_from].fpath, strerror(errno));
        return -1;
    }

    for (int i = 0; i < group_to->n; i ++)
    {
        write(group_to->pts[i].fd, ctx->buffer, n);
    }

    return 0;
}

static int spin_once(struct pts_junction_t * ctx)
{
    ssize_t n_events = epoll_wait(ctx->epoll_fd, ctx->events, MAX_EVENTS, -1);
    if (n_events < 0)
    {
        perror("epoll_wait() failed");
        return -1;
    }

    int rv;
    for (int i = 0; i < n_events; i++)
    {
        int event_fd = ctx->events[i].data.fd;
        for (int j = 0; j < ctx->up.n; j ++)
        {
            if (event_fd == ctx->up.pts[j].fd)
            {
                rv = make_transfer(ctx, ctx->up.pts[j].fd, &ctx->down);
                if (rv < 0)
                {
                    return rv;
                }
            }
        }

        for (int j = 0; j < ctx->down.n; j++)
        {
            if (event_fd == ctx->down.pts[j].fd)
            {
                rv = make_transfer(ctx, ctx->down.pts[j].fd, &ctx->up);
                if (rv < 0)
                {
                    return rv;
                }
            }
        }
    }
    return 0;
}

static void wait_for(struct pts_junction_t * ctx, struct pts_group_t * grp)
{
    for (int i = 0; i < grp->n; i ++)
    {
        struct epoll_event ev;
        ev.events = EPOLLIN;
        ev.data.fd = grp->pts[i].fd;
        if (epoll_ctl(ctx->epoll_fd, EPOLL_CTL_ADD, grp->pts[i].fd, &ev) < 0)
        {
            perror("epoll_ctl() failed");
            exit(1);
        }
    }
}

void spin(struct pts_junction_t * ctx)
{
    int rv;
    ctx->epoll_fd = epoll_create1(0);
    if (ctx->epoll_fd < 0)
    {
        perror("epoll_create1() failed");
        exit(1);
    }

    wait_for(ctx, &ctx->up);
    wait_for(ctx, &ctx->down);

    while (true)
    {
        rv = spin_once(ctx);
        if (rv < 0)
        {
            break;
        }
    }
}


void pts_cleanup_group(struct pts_group_t * grp)
{
    for (int i = 0; i < grp->n; i ++)
    {
//        printf("close %d unlink %s\n", grp->pts[i].fd, grp->pts[i].fpath);
        close(grp->pts[i].fd);
        unlink(grp->pts[i].fpath);
    }
}

void pts_cleanup(struct pts_junction_t * ctx)
{
    pts_cleanup_group(&ctx->up);
    pts_cleanup_group(&ctx->down);
}

static struct pts_junction_t * _junction = NULL;

static void handle_sigint(int sig)
{
    if (_junction != NULL)
    {
        pts_cleanup(_junction);
    }
}

void pts_start(struct pts_junction_t * ctx)
{
    for (int i = 0; i < ctx->up.n; i++)
    {
        ctx->up.pts[i].fd = create_pts(ctx->up.pts[i].fpath);
    }
    for (int i = 0; i < ctx->down.n; i++)
    {
        ctx->down.pts[i].fd = create_pts(ctx->down.pts[i].fpath);
    }
    _junction = ctx;
    signal(SIGINT, handle_sigint);

    spin(ctx);
    pts_cleanup(ctx);
}

void pts_init(struct pts_junction_t * ctx)
{
    ctx->up.n = 0;
    ctx->down.n = 0;
}

void pts_add(struct pts_group_t * grp, char * fpath)
{
    grp->pts[grp->n].fpath = strdup(fpath);
    grp->n ++;
}
