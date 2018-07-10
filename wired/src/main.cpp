#include <Arduino.h>
#include <FastLED.h>
#include <Wire.h>

#include "bno055_impl.h"
#include "log.h"
#include "utils.h"

Logger loggr(Serial);

// ---------------------------------------------------------------
// NeoPixel Config
// ---------------------------------------------------------------
static const int PIN_NEOPIXEL = A10;
CRGB led;

void setup_led() {
	FastLED.addLeds<NEOPIXEL, PIN_NEOPIXEL>(&led, 1);
	led = CRGB::Black;
	FastLED.show();
}

// ---------------------------------------------------------------
// BNO055 Config
// ---------------------------------------------------------------
bno055_t bno055 = {0};
float zeroHeading = 0;
float zeroPitch = 0;

static const int PIN_BNO055_RESET = A9;
static const int PIN_ZERO_HEADING = A8;

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

	bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
}

void setup() {
	Serial.begin(115200);
	Wire.begin();

	setup_led();
	setup_bno055();
}

struct PosData {
	float x;
	float y;
};

TimeOut updateTimer;

s8 dummy_calib_stat(u8* val) {
	*val = 1;
	return 0;
}

void showCalibState() {
	static int calViewIndex = 0;
	static TimeOut timer;
	static uint32_t colours[] = {0, 0x100000, 0x001000, 0x000010};
	static s8 (*calibFunctions[])(u8*) = {dummy_calib_stat, bno055_get_accel_calib_stat,
	                                      bno055_get_gyro_calib_stat, bno055_get_mag_calib_stat};

	if (timer.hasTimedOut()) {
		u8 calibStat;
		calibFunctions[calViewIndex](&calibStat);
		led = (calibStat == 3) ? 0 : colours[calViewIndex];
		FastLED.show();
		calViewIndex = (calViewIndex + 1) % 4;

		timer = TimeOut((3 - calibStat) * 100);
	}
}

float normaliseAngle(float a) {
	if (a <= -180.0f) {
		a += 360.0f;
	}
	if (a > 180.0f) {
		a -= 360.0f;
	}
	return a;
}

void loop() {
	showCalibState();

	if (updateTimer.hasTimedOut()) {
		updateTimer = TimeOut(100);

		bno055_euler_float_t f;
		bno055_convert_float_euler_hpr_deg(&f);

		if (digitalRead(PIN_ZERO_HEADING) == 0) {
			zeroHeading = f.h;
			zeroPitch = f.p;
		}

		f.h = normaliseAngle(f.h - zeroHeading);
		f.p = normaliseAngle(f.p - zeroPitch);

		loggr << "(" << f.h << f.p << f.r << ")";

		PosData pd{f.h, f.p};
	}
}