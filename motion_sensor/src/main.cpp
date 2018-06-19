#include <Arduino.h>
#include <FastLED.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>
#include <printf.h>

#include "bno055_impl.h"
#include "log.h"
#include "utils.h"

Logger loggr(Serial);

// ---------------------------------------------------------------
// NeoPixel Config
// ---------------------------------------------------------------
static const int PIN_NEOPIXEL = A0;
CRGB led;

void setup_led() {
	FastLED.addLeds<NEOPIXEL, PIN_NEOPIXEL>(&led, 1);
	led = CRGB::Pink;
	FastLED.show();
}

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
	radio.setPALevel(RF24_PA_LOW);
	radio.openWritingPipe(addresses[0]);
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

	setup_led();
	setup_bno055();
	setup_rf24();
}

struct PosData {
	float x;
	float y;
};

TimeOut updateTimer;

void showCalibState() {
	static int calViewIndex = 0;
	static TimeOut timer;
	static uint32_t colours[] = {0x200000, 0x002000, 0x000020};
	static s8 (*calibFunctions[])(u8*) = {bno055_get_accel_calib_stat, bno055_get_gyro_calib_stat,
	                                      bno055_get_mag_calib_stat};

	if (timer.hasTimedOut()) {
		timer = TimeOut(500);
		u8 calibStat;
		calibFunctions[calViewIndex](&calibStat);
		led = colours[calViewIndex] * (3 - calibStat) * 2;
		FastLED.show();
		calViewIndex = (calViewIndex + 1) % 3;
	}
}

void loop() {
	showCalibState();

	if (updateTimer.hasTimedOut()) {
		updateTimer = TimeOut(100);

		bno055_euler_float_t f;
		bno055_convert_float_euler_hpr_deg(&f);

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

		loggr << "(" << f.h << f.p << f.r << ")";

		PosData pd{f.h, f.p};
		loggr << radio.write(&pd, sizeof(pd));
	}
}