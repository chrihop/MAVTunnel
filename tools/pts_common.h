#ifndef _PTS_COMMON_H_
#define _PTS_COMMON_H_

#include <sys/epoll.h>
#include <stdint.h>

struct pts_t
{
    char * fpath;
    int fd;
};

#define MAX_PTS 16
#define MAX_GROUPS 2
#define MAX_EVENTS (MAX_PTS * MAX_GROUPS)

struct pts_group_t
{
    struct pts_t pts[MAX_PTS];
    int n;
};

struct pts_junction_t
{
    struct pts_group_t up, down;
    struct epoll_event events[MAX_EVENTS];
    int epoll_fd;
    uint8_t buffer[4096];
};

#if __cplusplus
extern "C" {
#endif

int create_pts(char * link);
void spin(struct pts_junction_t * ctx);
void pts_start(struct pts_junction_t * ctx);
void pts_init(struct pts_junction_t * ctx);
void pts_add(struct pts_group_t * grp, char * fpath);

#if __cplusplus
};
#endif

#endif /* _PTS_COMMON_H_ */
