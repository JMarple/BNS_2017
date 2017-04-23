#pragma systemFile

#if !defined(SPLINE_C_)
#define SPLINE_C_

#include "../util/math.c"
#include "./waypoint.c"

typedef struct {
	float a;  // ax^5
	float b;  // + bx^4
	float c;  // + cx^3
	float d;  // + dx^2
	float e;  // + ex

	float yOffset;
	float xOffset;
	float knotDistance;
	float thetaOffset;
	float arcLength;
} Spline;

bool almostEqual(float x, float y) {
	return fabs(x - y) < 0.000001;
}

Spline *newSpline(Spline *this, float x0, float y0, float theta0, float x1,
		float y1, float theta1) {
	// Transform to the origin.
	this->xOffset = x0;
	this->yOffset = y0;
	float x1Hat = sqrt(pow(x1 - x0, 2.0) + pow(y1 - y0, 2.0));
	if (x1Hat == 0.0) {
		return NULL;
	}
	this->knotDistance = x1Hat;
	this->thetaOffset = atan2(y1 - y0, x1 - x0);
	float theta0Hat = getDifferenceInAngleRadians(this->thetaOffset, theta0);
	float theta1Hat = getDifferenceInAngleRadians(this->thetaOffset, theta1);
	// We cannot handle vertical slopes in our rotated, translated basis. This
	// would mean the user wants to end up 90 degrees off of the straight line
	// between p0 and p1.
	if (almostEqual(fabs(theta0Hat), PI / 2.0)
			|| almostEqual(fabs(theta1Hat), PI / 2.0)) {
		return NULL;
	}
	// We also cannot handle the case that the end angle is facing toward the
	// start angle (total turn > 90 degrees).
	if (fabs(getDifferenceInAngleRadians(theta0Hat, theta1Hat)) >= PI / 2.0) {
		return NULL;
	}
	// Turn angles into derivatives (slopes).
	float yp0Hat = tan(theta0Hat);
	float yp1Hat = tan(theta1Hat);
	// Calculate the quintic spline coefficients.
	this->a = -(3.0 * (yp0Hat + yp1Hat)) / pow(x1Hat, 4.0);
	this->b = (8.0 * yp0Hat + 7.0 * yp1Hat) / pow(x1Hat, 3.0);
	this->c = -(6.0 * yp0Hat + 4.0 * yp1Hat) / pow(x1Hat, 2.0);
	this->d = 0.0;
	this->e = yp0Hat;

	return this;
}

Spline *newSpline(Spline *this, Waypoint *beginning, Waypoint *end) {
	return newSpline(this, beginning->x, beginning->y, beginning->theta, end->x,
			end->y, end->theta);
}

void print(Spline *this) {
	if (this == NULL) {
		return;
	}
	writeDebugStream("a=%f; b=%f; c=%f; d=%f; e=%f\n", this->a, this->b,
			this->c, this->d, this->e);
	writeDebugStream("xOffset=%f; yOffset=%f; thetaOffset=%f\n", this->xOffset,
			this->yOffset, this->thetaOffset);
}

task main() {
	Spline spline;
	newSpline(&spline, 0.0, 0.0, 0.0, 10.0, 0.0, 0.7);
	clearDebugStream();
	print(&spline);
}

#endif  // SPLINE_C_
