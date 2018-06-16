#include <Arduino.h>
#include <Wire.h>

#include "bno055_impl.h"

s8 I2C_bus_read(u8 dev_addr, u8 reg_addr, u8* reg_data, u8 cnt) {
	s8 written = 0;
	Wire.beginTransmission(dev_addr);
	Wire.write(reg_addr);
	written = Wire.endTransmission();
	delayMicroseconds(150);
	Wire.requestFrom(dev_addr, cnt);
	while (Wire.available()) {
		*reg_data = Wire.read();
		reg_data++;
	}
	return written;
}

void delay_msec(u32 ms) {
	delay(ms);
}

s8 I2C_bus_write(u8 dev_addr, u8 reg_addr, u8* reg_data, u8 cnt) {
	s8 comres = 0;
	Wire.beginTransmission(dev_addr);
	Wire.write(reg_addr);
	for (unsigned char index = 0; index < cnt; index++) {
		Wire.write(*reg_data);
		reg_data++;
	}
	comres = Wire.endTransmission();
	delayMicroseconds(150);
	return comres;
}
