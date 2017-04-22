#include "../pixy/pixy.c";

void testSetLED(Pixy *this) {
	unsigned int i = 0;
	unsigned char r, g, b;

	while (i < 500) {
		// Calculate r, g, b such that it cycles through the colors.
		r = i & 0xff;
		g = (i * 3) & 0xff;
		b = (i / 3) & 0xff;
		setLED(this, r, g, b);

		sleep(20);
		i++;
	}
}

void testSetServos(Pixy *this) {
	writeDebugStream("Moving pan/tilt to max positions.\n");
	setServos(this, PIXY_RCS_MAX_POS, PIXY_RCS_MAX_POS);
	sleep(1000);

	writeDebugStream("Moving pan/tilt to min positions.\n");
	setServos(this, PIXY_RCS_MIN_POS, PIXY_RCS_MIN_POS);
	sleep(1000);
}

void testPrint(Pixy *this) {
	while (true) {
		update(this);
		print(this);

		sleep(200);
	}
}

void testPrint() {
	Pixy pixy;
	newPixy(&pixy, UART1);

	// Regular block.
	pixy.blocks[0].signature = 1;
	pixy.blocks[0].x = 2;
	pixy.blocks[0].y = 3;
	pixy.blocks[0].width = 4;
	pixy.blocks[0].height = 5;
	pixy.blocks[0].angle = 0;

	// Color code (CC).
	pixy.blocks[1].signature = 10;
	pixy.blocks[1].x = 20;
	pixy.blocks[1].y = 30;
	pixy.blocks[1].width = 40;
	pixy.blocks[1].height = 50;
	pixy.blocks[1].angle = 60;

	pixy.blockCount = 2;

	print(&pixy);
}

void testRealPixy(Pixy *this) {
	testSetLED(this);
	testSetServos(this);
	testPrint(this);
}

void testVirtualPixy() {
	testPrint();
}

task main() {
	Pixy pixy;
	newPixy(&pixy, UART1, baudRate19200);
	testRealPixy(&pixy);

//	testVirtualPixy();
}
