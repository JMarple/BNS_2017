#include "../navigator/navigator.c"

task main() {
	// The block of code below is equivalent to each of the two after it.
	EncoderWheel leftEncoder;
	newEncoderWheel(&leftEncoder);
	setPort(&leftEncoder, dgtl1);
	setPulsesPerRev(&leftEncoder, 360.0);
	setWheelDiameter(&leftEncoder, 3.25);
	setGearRatio(&leftEncoder, 1.0);
	setSlipFactor(&leftEncoder, 1.0);
	setInverted(&leftEncoder, false);

	EncoderWheel rightEncoder;
	newEncoderWheel(&rightEncoder, dgtl3, 360.0, 3.25, 1.0, 1.0, false);

	EncoderWheel middleEncoder;
	newEncoderWheel(&middleEncoder, dgtl5, 360.0, 3.25);

	// 7" between left and right encoder wheels, and initialize (x, y, heading) to (0.0, 0.0, 0.0).
	Navigator navigator;
	newNavigator(&navigator, &leftEncoder, &rightEncoder, &middleEncoder, 7.0, 0.0, 0.0, 0.0);

	float x, y, heading;

	while (true) {
		update(&navigator);

		x = getX(&navigator);
		y = getY(&navigator);
		heading = getHeading(&navigator);

		sleep(15);
	}
}
