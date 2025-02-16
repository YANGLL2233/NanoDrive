#ifndef ERROR_H
#define ERROR_H
#include <stdint.h>
#include <stdbool.h>

/******************************************************/
// 错误码宏定义（兼容C89/C99，支持全32位）
#define ERROR_NONE                        UINT32_C(0x00000000) // 无错误
#define ERROR_TIMER_UPDATE_MISSED         (UINT32_C(1) << 0)   // 0x00000001
#define ERROR_MODULATION_MAGNITUDE        (UINT32_C(1) << 1)   // 0x00000002
#define ERROR_CURRENT_SENSE_SATURATION    (UINT32_C(1) << 2)   // 0x00000004
#define ERROR_UNSUPPORTED_ENCODER_MODE    (UINT32_C(1) << 3)   // 0x00000008
#define ERROR_ABS_SPI_COM_FAIL            (UINT32_C(1) << 4)   // 0x00000010
#define ERROR_UNSTABLE_GAIN               (UINT32_C(1) << 5)   // 0x00000020
#define ERROR_CURRENT_LIMIT_VIOLATION     (UINT32_C(1) << 6)   // 0x00000040
#define ERROR_UNKNOWN_CURRENT_MEASUREMENT (UINT32_C(1) << 7)   // 0x00000080
#define ERROR_BAD_TIMING                  (UINT32_C(1) << 8)   // 0x00000100
#define ERROR_9                           (UINT32_C(1) << 9)   // 0x00000200
#define ERROR_10                          (UINT32_C(1) << 10)  // 0x00000400
#define ERROR_11                          (UINT32_C(1) << 11)  // 0x00000800
#define ERROR_12                          (UINT32_C(1) << 12)  // 0x00001000
#define ERROR_13                          (UINT32_C(1) << 13)  // 0x00002000
#define ERROR_14                          (UINT32_C(1) << 14)  // 0x00004000
#define ERROR_15                          (UINT32_C(1) << 15)  // 0x00008000
#define ERROR_16                          (UINT32_C(1) << 16)  // 0x00010000
#define ERROR_17                          (UINT32_C(1) << 17)  // 0x00020000
#define ERROR_18                          (UINT32_C(1) << 18)  // 0x00040000
#define ERROR_19                          (UINT32_C(1) << 19)  // 0x00080000
#define ERROR_20                          (UINT32_C(1) << 20)  // 0x00100000
#define ERROR_21                          (UINT32_C(1) << 21)  // 0x00200000
#define ERROR_22                          (UINT32_C(1) << 22)  // 0x00400000
#define ERROR_23                          (UINT32_C(1) << 23)  // 0x00800000
#define ERROR_24                          (UINT32_C(1) << 24)  // 0x01000000
#define ERROR_25                          (UINT32_C(1) << 25)  // 0x02000000
#define ERROR_26                          (UINT32_C(1) << 26)  // 0x04000000
#define ERROR_27                          (UINT32_C(1) << 27)  // 0x08000000
#define ERROR_28                          (UINT32_C(1) << 28)  // 0x10000000
#define ERROR_29                          (UINT32_C(1) << 29)  // 0x20000000
#define ERROR_30                          (UINT32_C(1) << 30)  // 0x40000000
#define ERROR_31                          (UINT32_C(1) << 31)  // 0x80000000
#define ERROR_ALL                         UINT32_C(0xFFFFFFFF) // 所有错误位
/******************************************************/

void set_error(uint32_t error);
void clear_error(uint32_t error);
bool is_error(uint32_t error);

#endif
