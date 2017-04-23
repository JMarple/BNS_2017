#pragma systemFile

#if !defined(ENCODERWHEEL_C_)
#define ENCODERWHEEL_C_

#include "../util/string.c"

typedef struct {
	tSensors port;
	float pulsesPerRev;
	float wheelDiameter;
	float gearRatio;
	float slipFactor;
	bool inverted;
} EncoderWheel;

EncoderWheel *newEncoderWheel(EncoderWheel *this, tSensors port,
		float pulsesPerRev, float wheelDiameter, float gearRatio,
		float slipFactor, bool inverted) {
	if (this) {
		this->port = port;
		this->pulsesPerRev = pulsesPerRev;
		this->wheelDiameter = wheelDiameter;
		this->gearRatio = gearRatio;
		this->slipFactor = slipFactor;
		this->inverted = inverted;
	}
	return this;
}

EncoderWheel *newEncoderWheel(EncoderWheel *this, tSensors port,
		float pulsesPerRev, float wheelDiamter, float gearRatio,
		float slipFactor) {
	return newEncoderWheel(this, port, pulsesPerRev, wheelDiamter, gearRatio,
			slipFactor, false);
}

EncoderWheel *newEncoderWheel(EncoderWheel *this, tSensors port,
		float pulsesPerRev, float wheelDiamter, float gearRatio,
		bool inverted) {
	return newEncoderWheel(this, port, pulsesPerRev, wheelDiamter, gearRatio,
			1.0, inverted);
}

EncoderWheel *newEncoderWheel(EncoderWheel *this, tSensors port,
		float pulsesPerRev, float wheelDiamter, float gearRatio) {
	return newEncoderWheel(this, port, pulsesPerRev, wheelDiamter, gearRatio,
			1.0, false);
}

EncoderWheel *newEncoderWheel(EncoderWheel *this, tSensors port,
		float pulsesPerRev, float wheelDiamter, bool inverted) {
	return newEncoderWheel(this, port, pulsesPerRev, wheelDiamter, 1.0, 1.0,
			inverted);
}

EncoderWheel *newEncoderWheel(EncoderWheel *this, tSensors port,
		float pulsesPerRev, float wheelDiamter) {
	return newEncoderWheel(this, port, pulsesPerRev, wheelDiamter, 1.0, 1.0,
			false);
}

EncoderWheel *newEncoderWheel(EncoderWheel *this) {
	return newEncoderWheel(this, (tSensors)-1, 0.0, 0.0, 1.0, 1.0, false);
}

tSensors getPort(EncoderWheel *this) {
	return this ? this->port : (tSensors)-1;
}

void setPort(EncoderWheel *this, tSensors port) {
	if (this) {
		this->port = port;
	}
}

float getPulsesPerRev(EncoderWheel *this) {
	return this ? this->pulsesPerRev : 0.0;
}

void setPulsesPerRev(EncoderWheel *this, float pulsesPerRev) {
	if (this) {
		this->pulsesPerRev = pulsesPerRev;
	}
}

float getWheelDiameter(EncoderWheel *this) {
	return this ? this->wheelDiameter : 0.0;
}

void setWheelDiameter(EncoderWheel *this, float wheelDiameter) {
	if (this) {
		this->wheelDiameter = wheelDiameter;
	}
}

float getGearRatio(EncoderWheel *this) {
	return this ? this->gearRatio : 1.0;
}

void setGearRatio(EncoderWheel *this, float gearRatio) {
	if (this) {
		this->gearRatio = gearRatio;
	}
}

float getSlipFactor(EncoderWheel *this) {
	return this ? this->slipFactor : 1.0;
}

void setSlipFactor(EncoderWheel *this, float slipFactor) {
	if (this) {
		this->slipFactor = slipFactor;
	}
}

bool getInverted(EncoderWheel *this) {
	return this ? this->inverted : false;
}

void setInverted(EncoderWheel *this, bool inverted) {
	if (this) {
		this->inverted = inverted;
	}
}

long getPulses(EncoderWheel *this) {
	return this ? (this->inverted ? -1 : 1) * SensorValue[this->port] : 0;
}

float getDistance(EncoderWheel *this) {
	return this ? (getPulses(this) * PI * this->wheelDiameter)
			/ (this->pulsesPerRev * this->gearRatio * this->slipFactor) : 0.0;
}

void print(EncoderWheel *this) {
	if (this == NULL) {
		return;
	}
	writeDebugStream("Port: %s\n", toString(this->port));
	writeDebugStream("Pulses Per Rev: %f\n", this->pulsesPerRev);
	writeDebugStream("Wheel Diameter: %f\n", this->wheelDiameter);
	writeDebugStream("Gear Ratio: %f\n", this->gearRatio);
	writeDebugStream("Slip Factor: %f\n", this->slipFactor);
	writeDebugStream("Inverted: %s\n", toString(this->inverted));
	writeDebugStream("Distance: %f\n", getDistance(this));
}

#endif  // ENCODERWHEEL_C_
