#ifndef _MAVTUNNEL_OS_H_
#define _MAVTUNNEL_OS_H_

#define likely(x)              __builtin_expect(!!(x), 1)
#define unlikely(x)            __builtin_expect(!!(x), 0)

#if __cplusplus

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cstdint>
#include <cstring>

using namespace std;

#else /* __cplusplus */

#ifndef offsetof
#define offsetof(type, member) __builtin_offsetof(type, member)
#endif /* offsetof */

#define static_assert _Static_assert

#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#endif /* __cplusplus */

#ifdef MAVTUNNEL_LINUX
#include <time.h>
#include <execinfo.h>
#include <stdbool.h>

static inline void
print_backtrace(void)
{
    void*  callstack[32];
    int    i, frames = backtrace(callstack, 32);
    char** strs = backtrace_symbols(callstack, frames);
    for (i = 0; i < frames; ++i)
    {
        if (i == 2)
        {
            printf(" ===> ");
        }
        else
        {
            printf("      ");
        }
        printf("%s\n", strs[i]);
    }
    free(strs);
}

static inline unsigned long long time_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (unsigned long long)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

#ifndef INFO
#define INFO(fmt, ...) printf("[I] " fmt, ##__VA_ARGS__)
#endif /* INFO */

#ifndef WARN
#define WARN(fmt, ...)                                                         \
    printf("[W] %s:%u (in %s()): " fmt, __FILE__, __LINE__, __FUNCTION__,      \
        ##__VA_ARGS__)
#endif /* WARN */

#if !defined(PANIC) || !defined(PRIMITIVE_PANIC_DEFINED)
#define PANIC(fmt, ...)                                                        \
    do                                                                         \
    {                                                                          \
        printf("[P] %s:%u (in %s()): " fmt, __FILE__, __LINE__, __FUNCTION__,  \
            ##__VA_ARGS__);                                                    \
        print_backtrace();                                                     \
        exit(0);                                                               \
    } while (0)
#endif /* PANIC */


#ifndef ASSERT
#define ASSERT(cond)                                                           \
    do                                                                         \
    {                                                                          \
        if (unlikely(!(cond)))                                                 \
        {                                                                      \
            printf("[P] %s:%u (in %s()): assertion failed: " #cond "\n",       \
                __FILE__, __LINE__, __FUNCTION__);                             \
            print_backtrace();                                                 \
            exit(0);                                                           \
        }                                                                      \
    } while (0)
#endif /* ASSERT */

#elif defined (_CERTIKOS_)

#include <stdio.h>
#include <time.h>
#include <stdatomic.h>

static inline unsigned long long time_us(void)
{
    uint64_t tsc = tsc();
    return tsc / tsc_khz() * 1000;
}

#ifndef INFO
#define INFO(fmt, ...)                                                         \
    do                                                                         \
    {                                                                          \
        printf("[I] " fmt, ##__VA_ARGS__);                                     \
    } while (0)
#endif

#else

#ifndef INFO
#define INFO(fmt, ...)  baremetal_printf("[I] " fmt, __VA_ARGS__)
#endif

#ifndef WARN
#define WARN(fmt, ...)  baremetal_printf("[W] " fmt, __VA_ARGS__)
#endif

#ifndef PANIC
#define PANIC(fmt, ...) baremetal_printf("[P] " fmt, __VA_ARGS__)
#endif

#endif

#if __cplusplus
extern "C" {
#endif

static inline void puthex(uint8_t * buf, size_t len)
{
    INFO("%lu: ", len);
    for (size_t i = 0; i < len; i++)
    {
        printf("%02x", buf[i]);
    }
    printf("\n");
}

#if __cplusplus
}
#endif

#endif /* !_MAVTUNNEL_OS_H_ */
