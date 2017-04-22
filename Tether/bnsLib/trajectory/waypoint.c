#pragma systemFile

#if !defined(WAYPOINT_C_)
#define WAYPOINT_C_

typedef struct Waypoint {
	float x;
	float y;
	float theta;

	struct Waypoint *next;
} Waypoint;

Waypoint *newWaypoint(Waypoint *this, float x, float y, float theta,
		Waypoint *next) {
	if (this) {
		this->x = x;
		this->y = y;
		this->theta = theta;

		this->next = next;
	}
	return this;
}

Waypoint *newWaypoint(Waypoint *this, float x, float y, float theta) {
	return newWaypoint(this, x, y, theta, NULL);
}

Waypoint *newWaypoint(Waypoint *this, Waypoint *next) {
	return newWaypoint(this, 0.0, 0.0, 0.0, next);
}

Waypoint *newWaypoint(Waypoint *this) {
	return newWaypoint(this, 0.0, 0.0, 0.0, NULL);
}

float getX(Waypoint *this) {
	return this ? this->x : 0.0;
}

void setX(Waypoint *this, float x) {
	if (this) {
		this->x = x;
	}
}

float getY(Waypoint *this) {
	return this ? this->y : 0.0;
}

void setY(Waypoint *this, float y) {
	if (this) {
		this->y = y;
	}
}

float getTheta(Waypoint *this) {
	return this ? this->theta : 0.0;
}

void setTheta(Waypoint *this, float theta) {
	if (this) {
		this->theta = theta;
	}
}

Waypoint *getNext(Waypoint *this) {
	return this ? this->next : NULL;
}

void setNext(Waypoint *this, Waypoint *next) {
	if (this) {
		this->next = next;
	}
}

void print(Waypoint *this) {
	if (this == NULL) {
		return;
	}
	writeDebugStream("x: %f\n", this->x);
	writeDebugStream("y: %f\n", this->y);
	writeDebugStream("theta: %f\n", this->theta);
}

#endif  // WAYPOINT_C_
