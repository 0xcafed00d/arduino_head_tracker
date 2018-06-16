#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>
#include <printf.h>

#include "bno055_impl.h"
#include "log.h"

Logger loggr(Serial);

// ---------------------------------------------------------------
// RF24 Config
// ---------------------------------------------------------------
/*
        rf24	Arduino uno

    1	GND		GND
    2	3.3v	3.3v
    3	CE		pin 10
    4	CSN		pin 9
    5	SCK		SCK  pin 13
    6	MOSI	MOSI pin 11
    7	MISO	MISO pin 12
    8	NC
*/
static const int PIN_RF24_CE = 10;
static const int PIN_RF24_CSN = 9;

RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);

byte addresses[][6] = {"1Node", "2Node"};

void setup_rf24() {
	radio.begin();
	radio.printDetails();
}

// ---------------------------------------------------------------
// BNO055 Config
// ---------------------------------------------------------------
bno055_t bno055 = {0};
float zeroHeading = 0;

static const int PIN_BNO055_RESET = A3;
static const int PIN_ZERO_HEADING = 4;

void setup_bno055() {
	pinMode(PIN_ZERO_HEADING, INPUT_PULLUP);
	pinMode(PIN_BNO055_RESET, OUTPUT);

	digitalWrite(PIN_BNO055_RESET, LOW);
	delay_msec(300);
	digitalWrite(PIN_BNO055_RESET, HIGH);
	delay_msec(700);

	bno055.delay_msec = delay_msec;
	bno055.bus_read = I2C_bus_read;
	bno055.bus_write = I2C_bus_write;
	bno055.dev_addr = 0x28;

	auto res = bno055_init(&bno055);
	loggr << res << bno055.accel_rev_id << bno055.sw_rev_id;

	bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF_FMC_OFF);
}

void setup() {
	Serial.begin(115200);
	Wire.begin();
	SPI.begin();
	printf_begin();

	setup_bno055();
	setup_rf24();
}

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

	if (digitalRead(PIN_ZERO_HEADING) == 0) {
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