#include "os.h"
#include "tunnel.h"

enum mavtunnel_error_t
mavtunnel_check_out_buffer(uint8_t * buf, size_t len)
{
    mavlink_message_t msg;
    mavlink_status_t status;

    memset(&status, 0, sizeof(status));

    for (size_t i = 0; i < len; i++)
    {
        if (mavlink_parse_char(MAVLINK_COMM_3, buf[i], &msg, &status))
        {
            if (status.packet_rx_drop_count > 0)
            {
                return MERR_BAD_MESSAGE;
            }
            return i == len - 1 ? MERR_OK : MERR_BAD_MESSAGE;
        }
    }
    return MERR_BAD_MESSAGE;
}
