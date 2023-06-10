#ifndef _MAVTUNNEL_TUNNEL_H_
#define _MAVTUNNEL_TUNNEL_H_

#include "os.h"
#include <v2.0/ardupilotmega/mavlink.h>

enum mavtunnel_error_t
{
    MERR_OK,
    MERR_END,
    MERR_DEVICE_ERROR,
    MERR_BAD_CRC,
    MERR_BAD_MAGIC,
    MERR_BAD_LENGTH,
    MERR_BAD_MESSAGE,
    MERR_BAD_STATE,
    MERR_BAD_PROTOCOL,
    MERR_BAD_ID,
};

#define MAVTUNNEL_OUTPUT_BUFFER_SIZE 1024

struct mavtunnel_reader_t;

typedef ssize_t (*read_t)(struct mavtunnel_reader_t* ctx, uint8_t* bytes, size_t len);

struct mavtunnel_reader_t
{
    void * object;
    read_t read;
};

struct mavtunnel_writer_t;

typedef enum mavtunnel_error_t (*write_t)(struct mavtunnel_writer_t* ctx, const uint8_t* bytes, size_t len);

struct mavtunnel_writer_t
{
    void * object;
    write_t write;
};

struct mavtunnel_codec_t;

typedef enum mavtunnel_error_t (*encode_t)(
    struct mavtunnel_codec_t* ctx, mavlink_message_t* msg);

struct mavtunnel_codec_t
{
    void * object;
    encode_t encode;
};

enum mavtunnel_status_t
{
    MT_STATUS_UNINITIALIZED,
    MT_STATUS_OK,
    MT_STATUS_FAILURE,
};

#define MAVTUNNEL_READ_BUFFER_SIZE 1024
#define MAVTUNNEL_UPDATE_INTERVAL_US 2000000

enum mavtunnel_perf_metrics_t
{
    MT_PERF_RECV_COUNT = 0,
    MT_PERF_RECV_BYTE = 1,
    MT_PERF_DROP_BYTE,
    MT_PERF_DROP_COUNT,
    MT_PERF_SEQ_ERR,
    MT_PERF_SENT_COUNT,
    MT_PERF_SENT_BYTE,

    MAX_MT_PERF_METRICS,
};

struct mavtunnel_t
{
    size_t                     id;
    atomic_bool                terminate;
    enum mavtunnel_status_t    mode;
    struct mavtunnel_reader_t  reader;
    struct mavtunnel_writer_t  writer;
    struct mavtunnel_codec_t   codec;
    uint8_t                    read_buffer[MAVTUNNEL_READ_BUFFER_SIZE];
    mavlink_message_t          rx_msg;
    mavlink_status_t           rx_status;
    mavlink_status_t           tx_status;
    uint8_t                    tx_buf[MAVTUNNEL_OUTPUT_BUFFER_SIZE];
#ifdef MAVTUNNEL_PROFILING
    uint64_t count[MAX_MT_PERF_METRICS], last[MAX_MT_PERF_METRICS];
    uint64_t exec_time_us;
    uint64_t last_update_us;
#endif
};

#define MAVTUNNEL_INIT { .mode = MT_STATUS_UNINITIALIZED };

#if __cplusplus
extern "C"
{
#endif

void mavtunnel_init(struct mavtunnel_t* ctx, size_t id);

enum mavtunnel_error_t mavtunnel_spin_once(struct mavtunnel_t* ctx);

void mavtunnel_spin(struct mavtunnel_t* ctx);

void mavtunnel_exit(struct mavtunnel_t * ctx);

/**
 * Aux
 */
enum mavtunnel_error_t mavtunnel_check_out_buffer(uint8_t * buf, size_t len);

#if __cplusplus
};
#endif

#endif /* !_MAVTUNNEL_TUNNEL_H_ */
