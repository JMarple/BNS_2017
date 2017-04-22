#pragma systemFile

#if !defined(TRAPEZOIDALPROFILE_C_)
#define TRAPEZOIDALPROFILE_C_

typedef struct {
	float maxVel;
	float maxAcc;

	float v0;
	float v1;
} TrapezoidalProfile;

TrapezoidalProfile *newTrapezoidalProfile(TrapezoidalProfile *this, float maxVel,
		float maxAcc, float v0, float v1) {
	if (this) {
		this->maxVel = maxVel;
		this->maxAcc = maxAcc;

		this->v0 = v0;
		this->v1 = v1;
	}
	return this;
}

float getMaxVel(TrapezoidalProfile *this) {
	return this ? this->maxVel : 0.0;
}

void setMaxVel(TrapezoidalProfile *this, float maxVel) {
	if (this) {
		this->maxVel = maxVel;
	}
}

float getMaxAcc(TrapezoidalProfile *this) {
	return this ? this->maxAcc : 0.0;
}

void setMaxAcc(TrapezoidalProfile *this, float maxAcc) {
	if (this) {
		this->maxAcc = maxAcc;
	}
}

float getV0(TrapezoidalProfile *this) {
	return this ? this->v0 : 0.0;
}

void setV0(TrapezoidalProfile *this, float v0) {
	if (this) {
		this->v0 = v0;
	}
}

float getV1(TrapezoidalProfile *this) {
	return this ? this->v1 : 0.0;
}

void setV1(TrapezoidalProfile *this, float v1) {
	if (this) {
		this->v1 = v1;
	}
}

float update(TrapezoidalProfile *this, float error, float t) {
	if (this == NULL) {
		return 0.0;
	}
	// Find target velocity based on time (acceleration).
	float v = this->v0 + this->maxAcc * t;  // v = v0 + a*t
	// Keep target velocity within its limit.
	if (v > this->maxVel) {
		v = sgn(v) * this->maxVel;
	}
	// If it is time to decelerate (use v1 in place of v0 to decelerate).
	// ds = (v^2 - v0^2) / (2*a) derived from v^2 = v0^2 + 2*a*ds.
	if (error <= (pow(v, 2.0) - pow(this->v1, 2.0)) / (2.0 * this->maxAcc)) {
		// Find target velocity based on error (deceleration).
		// v = sqrt(v0^2 + 2*a*ds) derived from v^2 = v0^2 + 2*a*ds.
		v = sgn(this->maxAcc * error)
				* sqrt(fabs(pow(this->v1, 2.0) + 2 * this->maxAcc * error));
	}
	return v;  // Return target velocity.
}

#endif  // TRAPEZOIDALPROFILE_C_
