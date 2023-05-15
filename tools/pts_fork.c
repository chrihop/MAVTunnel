#include <stdio.h>
#include <stdlib.h>

#include "pts_common.h"

struct pts_junction_t junction;

int main(int argc, char ** argv)
{
    if (argc < 3)
    {
        printf("error: %s <uplink> <downlink 1> <downlink 2> ...\n", argv[0]);
        exit(0);
    }

    pts_init(&junction);

    pts_add(&junction.up, argv[1]);
    printf("[%s] <--> [", argv[1]);
    for (int i = 2; i < argc; i ++)
    {
        pts_add(&junction.down, argv[i]);
        printf("%s%s", i == 2? "" : ", ", argv[i]);
    }
    printf("]\n");

    pts_start(&junction);

    return 0;
}
