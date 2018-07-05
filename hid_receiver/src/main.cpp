#include <Arduino.h>
#include <Joystick.h>
#include <Keyboard.h>
#include <Mouse.h>
#include <RF24.h>
#include <printf.h>

#include "log.h"
/*
Joystick_ joystick(JOYSTICK_DEFAULT_REPORT_ID,
                   JOYSTICK_TYPE_JOYSTICK,
                   1,
                   0,
                   true,
                   true,
                   false,
                   false,
                   false,
                   false,
                   false,
                   false,
                   false,
                   false,
                   false);
*/
Joystick_ joystick;

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

Logger loggr(Serial);

void setup() {
	joystick.begin(false);
	Mouse.begin();
	Keyboard.begin();

	joystick.setXAxisRange(-9000, 9000);
	joystick.setYAxisRange(-9000, 9000);

	radio.begin();
	printf_begin();

	radio.setPALevel(RF24_PA_LOW);
	radio.openReadingPipe(1, addresses[0]);
	radio.startListening();

	radio.printDetails();
}

struct PosData {
	float x;
	float y;
};

void loop() {
	PosData p;

	if (radio.available()) {
		radio.read(&p, sizeof(p));
		loggr << p.x << p.y;
		joystick.setXAxis(p.x * 100);
		joystick.setYAxis(p.y * 100);
		joystick.sendState();
	}
}