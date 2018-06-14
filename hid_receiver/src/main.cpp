#include <Arduino.h>
#include <Joystick.h>
#include <Keyboard.h>
#include <Mouse.h>
#include <RF24.h>
#include <printf.h>

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

void setup() {
	joystick.begin();
	Mouse.begin();
	Keyboard.begin();

	while (!Serial) {
	}

	Serial.begin(115200);

	radio.begin();
	printf_begin();

	radio.setPALevel(RF24_PA_LOW);

	radio.openWritingPipe(addresses[0]);
	radio.openReadingPipe(1, addresses[1]);

	radio.startListening();

	radio.printDetails();
}

void loop() {
	uint32_t data;

	if (radio.available()) {
		radio.read(&data, sizeof(uint32_t));
		Serial.println(data);
	}
}