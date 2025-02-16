#ifndef AS5047P_H
#define AS5047P_H

#include "main.h"

#define AS_ARR (16384U)

/*内容随外界变化的寄存器 可读不可写*/
#define NOP_ADDR 0x0000      // 启动读取过程寄存器地址
#define ERRFL_ADDR 0x0001    // 错误寄存器地址
#define PROG_ADDR 0x0003     // 编程寄存器地址
#define DIAAGC_ADDR 0x3FFC   // 诊断和AGC寄存器地址
#define MAG_ADDR 0x3FFD      // CORDIC寄存器地址
#define ANGLEUNC_ADDR 0x3FFE // 无动态角度误差补偿的测量角度寄存器地址
#define ANGLECOM_ADDR 0x3FFF // 带动态角度误差补偿的测量角度寄存器地址

/*配置选项寄存器 可读可写*/
#define ZPOSM 0x0016
#define ZPOSL 0x0017
#define SETTINGS1 0x0018
#define SETTINGS2 0x0019

/*读取数据命令*/
#define READ_ANGLECOM 0xFFFF
#define READ_NOP 0xC000
#define READ_ERRFL 0x4001

/*SPI配置*/
#define AP_SPI SPI1
#define AP_CS_PORT SPI1_NSS_GPIO_Port
#define AP_CS_PIN SPI1_NSS_Pin

#define AP_CS(x) (HAL_GPIO_WritePin(AP_CS_PORT, AP_CS_PIN, x ? GPIO_PIN_SET : GPIO_PIN_RESET))

unsigned int even_check(unsigned int x);
unsigned int read_as5047p_checked(unsigned int cmd);
unsigned int read_as5047p_uncheck(unsigned int cmd);
unsigned int read_as5047p_error(void);

#endif
