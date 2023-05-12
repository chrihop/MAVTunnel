#ifndef _MAVTUNNEL_TUNNEL_H_
#define _MAVTUNNEL_TUNNEL_H_

enum mavtunnel_error_t
{
    MERR_OK,
    MERR_END,
    MERR_BAD_CRC,
    MERR_BAD_MAGIC,
    MERR_BAD_LENGTH,
    MERR_BAD_MESSAGE,
    MERR_BAD_STATE,
    MERR_BAD_PROTOCOL,
    MERR_BAD_ID,
};

struct mavtunnel_reader_t;

typedef int (*read_byte_t)(struct mavtunnel_reader_t* ctx);

struct mavtunnel_reader_t
{
    read_byte_t read_byte;
};

struct mavtunnel_writer_t;

typedef enum mavtunnel_error_t (*write_t)(
    struct mavtunnel_writer_t* ctx, mavlink_message_t* msg);

struct mavtunnel_writer_t
{
    write_t write;
};

struct mavtunnel_codec_t;

typedef enum mavtunnel_error_t (*encode_t)(
    struct mavtunnel_codec_t* ctx, mavlink_message_t* msg);

struct mavtunnel_codec_t
{
    encode_t encode;
};

enum mavtunnel_status_t
{
    MT_STATUS_UNINITIALIZED,
    MT_STATUS_OK,
    MT_STATUS_FAILURE,
};

struct mavtunnel_t
{
    size_t                     id;
    enum mavtunnel_status_t    mode;
    struct mavtunnel_reader_t* reader;
    struct mavtunnel_writer_t* writer;
    struct mavtunnel_codec_t*  codec;
    mavlink_message_t          msg;
    mavlink_status_t           status;
#ifdef MAVTUNNEL_PROFILING
    size_t message_count;
    size_t byte_count;
    size_t drop_count;
#endif
};

#define MAVTUNNEL_INIT { .mode = MT_STATUS_UNINITIALIZED };

#if __cplusplus
extern "C"
{
#endif

    void                   mavtunnel_init(struct mavtunnel_t* ctx,
                          struct mavtunnel_reader_t* reader, struct mavtunnel_writer_t* writer,
                          struct mavtunnel_codec_t* codec);

    enum mavtunnel_error_t mavtunnel_spin_once(struct mavtunnel_t* ctx);

    void                   mavtunnel_spin(struct mavtunnel_t* ctx);

#if __cplusplus
};
#endif

#endif /* !_MAVTUNNEL_TUNNEL_H_ */
