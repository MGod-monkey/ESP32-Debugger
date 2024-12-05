#ifndef __PROGRAMMER_H__
#define __PROGRAMMER_H__

#include "prog_data.h"

#ifdef __cplusplus
extern "C" {
#endif

// C 接口函数声明
void programmer_init(void);
prog_err_def programmer_request_handle(char *buf, int len);
prog_err_def programmer_write_data(uint8_t *data, int len);

// 修改为使用指针而不是引用，使其兼容 C
void programmer_get_status(char *buf, int size, int *encode_len);

#ifdef __cplusplus
}
#endif

#endif /* __PROGRAMMER_H__ */