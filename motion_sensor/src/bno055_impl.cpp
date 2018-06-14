#include <Arduino.h>
#include <Wire.h>

#include "bno055_impl.h"

s8 I2C_bus_read(u8 dev_addr, u8 reg_addr, u8* reg_data, u8 cnt) {
	s8 written = 0;
	Wire.beginTransmission(dev_addr);  // Start of transmission
	Wire.write(reg_addr);              // Desired start register
	written = Wire.endTransmission();  // Stop of transmission
	delayMicroseconds(150);            // Caution Delay
	Wire.requestFrom(dev_addr, cnt);   // Request data
	while (Wire.available())           // The slave device may send less than requested (burst read)
	{
		*reg_data = Wire.read();  // Receive a byte
		reg_data++;               // Increment pointer
	}
	return written;
}

void delay_msec(u32 ms) {
	delay(ms);
}

s8 I2C_bus_write(u8 dev_addr, u8 reg_addr, u8* reg_data, u8 cnt) {
	s8 comres = 0;
	Wire.beginTransmission(dev_addr);  // Start of transmission
	Wire.write(reg_addr);              // Desired start register
	for (unsigned char index = 0; index < cnt;
	     index++)  // Note that the BNO055 supports burst write
	{
		Wire.write(*reg_data);  // Write the data
		reg_data++;             // Increment pointer
	}
	comres = Wire.endTransmission();  // Stop of transmission
	delayMicroseconds(150);           // Caution Delay
	return comres;
}
