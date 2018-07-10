#ifndef BNO055_IMPL_H
#define BNO055_IMPL_H

extern "C" {
#include <bno055.h>
}
s8 I2C_bus_read(u8 dev_addr, u8 reg_addr, u8* reg_data, u8 cnt);
s8 I2C_bus_write(u8 dev_addr, u8 reg_addr, u8* reg_data, u8 cnt);
void delay_msec(u32 ms);

#endif /* BNO055_IMPL_H */
