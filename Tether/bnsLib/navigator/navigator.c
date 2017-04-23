#pragma systemFile

#if !defined(NAVIGATOR_C_)
#define NAVIGATOR_C_

#include "../util/math.c"
#include "../components/encoderWheel.c"
#include "../components/gyro.c"

typedef struct {
	EncoderWheel *leftEncoder;
	EncoderWheel *rightEncoder;
	EncoderWheel *middleEncoder;

	Gyro *gyro;

	float driveWidth;

	float x;
	float y;
	float heading;

	float lastL;
	float lastR;
	float lastM;

	float lastG;

	TSemaphore sem;
} Navigator;

Navigator *newNavigator(Navigator *this, EncoderWheel *leftEncoder,
		EncoderWheel *rightEncoder, EncoderWheel *middleEncoder, Gyro *gyro,
		float driveWidth, float x, float y, float heading) {
	if (this) {
		this->leftEncoder = leftEncoder;
		this->rightEncoder = rightEncoder;
		this->middleEncoder = middleEncoder;

		this->gyro = gyro;

		this->driveWidth = driveWidth;

		this->x = x;
		this->y = y;
		this->heading = heading;

		this->lastL = getDistance(leftEncoder);
		this->lastR = getDistance(rightEncoder);
		this->lastM = getDistance(middleEncoder);

		this->lastG = getAngle(gyro);

		semaphoreInitialize(this->sem);
	}
	return this;
}

Navigator *newNavigator(Navigator *this, EncoderWheel *leftEncoder,
		EncoderWheel *rightEncoder, EncoderWheel *middleEncoder,
		float driveWidth, float x, float y, float heading) {
	return newNavigator(this, leftEncoder, rightEncoder, middleEncoder, NULL,
			driveWidth, x, y, heading);
}

Navigator *newNavigator(Navigator *this, EncoderWheel *leftEncoder,
		EncoderWheel *rightEncoder, EncoderWheel *middleEncoder,
		float driveWidth) {
	return newNavigator(this, leftEncoder, rightEncoder, middleEncoder, NULL,
			driveWidth, 0.0, 0.0, 0.0);
}

Navigator *newNavigator(Navigator *this, EncoderWheel *leftEncoder,
		EncoderWheel *rightEncoder, float driveWidth, float x, float y,
		float heading) {
	return newNavigator(this, leftEncoder, rightEncoder, NULL, NULL, driveWidth,
			x, y, heading);
}

Navigator *newNavigator(Navigator *this, EncoderWheel *leftEncoder,
		EncoderWheel *rightEncoder, float driveWidth) {
	return newNavigator(this, leftEncoder, rightEncoder, NULL, NULL, driveWidth,
			0.0, 0.0, 0.0);
}

Navigator *newNavigator(Navigator *this, EncoderWheel *encoderWheel,
		Gyro *gyro, float x, float y, float heading) {
	return newNavigator(this, encoderWheel, NULL, NULL, gyro, 0.0, x, y,
			heading);
}

Navigator *newNavigator(Navigator *this, EncoderWheel *encoderWheel,
		Gyro *gyro) {
	return newNavigator(this, encoderWheel, NULL, NULL, gyro, 0.0, 0.0,
			0.0, 0.0);
}

Navigator *newNavigator(Navigator *this) {
	return newNavigator(this, NULL, NULL, NULL, NULL, 0.0, 0.0, 0.0, 0.0);
}

EncoderWheel *getLeftEncoder(Navigator *this) {
	return this ? this->leftEncoder : NULL;
}

void setLeftEncoder(Navigator *this, EncoderWheel *leftEncoder) {
	if (this) {
		semaphoreLock(this->sem);

		this->leftEncoder = leftEncoder;
		this->lastL = getDistance(leftEncoder);

		if (bDoesTaskOwnSemaphore(this->sem)) {
			semaphoreUnlock(this->sem);
		}
	}
}

EncoderWheel *getRightEncoder(Navigator *this) {
	return this ? this->rightEncoder : NULL;
}

void setRightEncoder(Navigator *this, EncoderWheel *rightEncoder) {
	if (this) {
		semaphoreLock(this->sem);

		this->rightEncoder = rightEncoder;
		this->lastR = getDistance(rightEncoder);

		if (bDoesTaskOwnSemaphore(this->sem)) {
			semaphoreUnlock(this->sem);
		}
	}
}

EncoderWheel *getMiddleEncoder(Navigator *this) {
	return this ? this->middleEncoder : NULL;
}

void setMiddleEncoder(Navigator *this, EncoderWheel *middleEncoder) {
	if (this) {
		semaphoreLock(this->sem);

		this->middleEncoder = middleEncoder;
		this->lastM = getDistance(middleEncoder);

		if (bDoesTaskOwnSemaphore(this->sem)) {
			semaphoreUnlock(this->sem);
		}
	}
}

EncoderWheel *getGyro(Navigator *this) {
	return this ? this->gyro : NULL;
}

void setGyro(Navigator *this, Gyro *gyro) {
	if (this) {
		semaphoreLock(this->sem);

		this->gyro = gyro;
		this->lastG = getAngle(gyro);

		if (bDoesTaskOwnSemaphore(this->sem)) {
			semaphoreUnlock(this->sem);
		}
	}
}

float getDriveWidth(Navigator *this) {
	return this ? this->driveWidth : 0.0;
}

void setDriveWidth(Navigator *this, float driveWidth) {
	if (this) {
		this->driveWidth = driveWidth;
	}
}

float getX(Navigator *this) {
	return this ? this->x : 0.0;
}

void setX(Navigator *this, float x) {
	if (this) {
		semaphoreLock(this->sem);

		this->x = x;

		if (bDoesTaskOwnSemaphore(this->sem)) {
			semaphoreUnlock(this->sem);
		}
	}
}

float getY(Navigator *this) {
	return this ? this->y : 0.0;
}

void setY(Navigator *this, float y) {
	if (this) {
		semaphoreLock(this->sem);

		this->y = y;

		if (bDoesTaskOwnSemaphore(this->sem)) {
			semaphoreUnlock(this->sem);
		}
	}
}

float getHeading(Navigator *this) {
	return this ? this->heading : 0.0;
}

void setHeading(Navigator *this, float heading) {
	if (this) {
		semaphoreLock(this->sem);

		this->heading = boundAngle0To2PiRadians(heading);

		if (bDoesTaskOwnSemaphore(this->sem)) {
			semaphoreUnlock(this->sem);
		}
	}
}

void update(Navigator *this) {
	if (this == NULL) {
		return;
	}
	semaphoreLock(this->sem);

	float diffL = getDistance(this->leftEncoder) - this->lastL;
	float diffR = getDistance(this->rightEncoder) - this->lastR;
	float diffM = getDistance(this->middleEncoder) - this->lastM;

	float diffG = getAngle(this->gyro) - this->lastG;

	this->lastL += diffL;
	this->lastR += diffR;
	this->lastM += diffM;

	this->lastG += diffG;

	if (bDoesTaskOwnSemaphore(this->sem)) {
		semaphoreUnlock(this->sem);
	}
	float diffH = this->gyro ? diffG : (diffR - diffL) / this->driveWidth;
	float tempHeading = this->heading + diffH / 2.0;
	float magnitude = (diffL + diffR) / (this->rightEncoder ? 2.0 : 1.0);

	this->x += magnitude * sin(tempHeading) + diffM * cos(tempHeading);
	this->y += magnitude * cos(tempHeading) + diffM * sin(tempHeading);
	this->heading = boundAngle0To2PiRadians(this->heading + diffH);
}

void print(Navigator *this) {
	if (this) {
		writeDebugStream("Drive Width: %d\n", this->driveWidth);
		writeDebugStream("x: %f\n", this->x);
		writeDebugStream("y: %f\n", this->y);
		writeDebugStream("Heading: %f\n", this->heading);
	}
}

#endif  // NAVIGATOR_C_
