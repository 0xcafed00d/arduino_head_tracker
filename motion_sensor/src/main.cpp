#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>

#include "bno055_impl.h"
#include "log.h"

/*
        rf24	Arduino leonardo

    1	GND		GND
    2	3.3v	3.3v
    3	CE		SDA (Ardiuno pin 2)
    4	CSN		SCL (Ardiuno pin 3)
    5	SCK		SCK
    6	MOSI	MOSI
    7	MISO	MISO
    8	NC
*/

RF24 radio(2, 3);

byte addresses[][6] = {"1Node", "2Node"};

bno055_t bno055 = {0};

static const int BNO055_RESET_PIN = A3;
static const int ZERO_HEADING_PIN = 4;

Logger loggr(Serial);

void setup() {
	Serial.begin(115200);

	Wire.begin();

	pinMode(ZERO_HEADING_PIN, INPUT_PULLUP);
	pinMode(BNO055_RESET_PIN, OUTPUT);

	digitalWrite(BNO055_RESET_PIN, LOW);
	delay_msec(300);
	digitalWrite(BNO055_RESET_PIN, HIGH);
	delay_msec(700);

	bno055.delay_msec = delay_msec;
	bno055.bus_read = I2C_bus_read;
	bno055.bus_write = I2C_bus_write;
	bno055.dev_addr = 0x28;

	auto res = bno055_init(&bno055);

	loggr << res << bno055.accel_rev_id << bno055.sw_rev_id;

	bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF_FMC_OFF);
}

float zeroHeading = 0;

void loop() {
	u8 sys_cal_stat = 0;
	bno055_get_sys_calib_stat(&sys_cal_stat);
	u8 accel_cal_stat = 0;
	bno055_get_accel_calib_stat(&accel_cal_stat);
	u8 gyro_cal_stat = 0;
	bno055_get_gyro_calib_stat(&gyro_cal_stat);
	u8 mag_cal_stat = 0;
	bno055_get_mag_calib_stat(&mag_cal_stat);

	bno055_euler_t q;
	bno055_read_euler_hrp(&q);
	bno055_euler_float_t f;
	bno055_convert_float_euler_hpr_deg(&f);

	delay(50);

	if (digitalRead(ZERO_HEADING_PIN) == 0) {
		zeroHeading = f.h;
	}

	f.h = f.h - zeroHeading;
	if (f.h <= -180.0f) {
		f.h += 360.0f;
	}
	if (f.h > 180.0f) {
		f.h -= 360.0f;
	}

	loggr << "(" << f.h << f.p << f.r << ")" << (int)sys_cal_stat << (int)accel_cal_stat
	      << (int)gyro_cal_stat << (int)mag_cal_stat;
}