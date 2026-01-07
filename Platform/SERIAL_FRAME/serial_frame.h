#ifndef SERIAL_FRAME_H
#define SERIAL_FRAME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#define SERIAL_FRAME_HEAD              0xAA
#define SERIAL_FRAME_TAIL              0x0D
#define SERIAL_FRAME_MAX_DATA_LEN      64


typedef enum
{
    SERIAL_FRAME_OK = 0,
    SERIAL_FRAME_ERR_HEAD,
    SERIAL_FRAME_ERR_TAIL,
    SERIAL_FRAME_ERR_LEN,
    SERIAL_FRAME_ERR_CHECKSUM,
    SERIAL_FRAME_ERR_FORMAT
} serial_frame_ret_t;


typedef struct
{
    uint8_t  cmd;
    uint8_t  data[SERIAL_FRAME_MAX_DATA_LEN];
    uint8_t  data_len;
} serial_frame_t;


/**
 * @brief  串口帧解析（完整帧）
 * @param  raw      原始帧数据
 * @param  raw_len  原始帧长度
 * @param  out      解析后的帧
 * @retval serial_frame_ret_t
 */
serial_frame_ret_t serial_frame_parse(
    const uint8_t *raw,
    uint16_t raw_len,
    serial_frame_t *out
);
uint16_t serial_frame_build(
    uint8_t cmd,
    const uint8_t *data,
    uint8_t data_len,
    uint8_t *out_buf,
    uint16_t out_buf_size
);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_FRAME_H */
