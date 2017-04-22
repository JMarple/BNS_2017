#pragma systemFile

#if !defined(WAYPOINTSEQUENCE_C_)
#define WAYPOINTSEQUENCE_C_

#include "./waypoint.c"

typedef struct {
	Waypoint *head;
	Waypoint *tail;
} WaypointSequence;

WaypointSequence *newWaypointSequence(WaypointSequence *this, Waypoint *head,
		Waypoint *tail) {
	if (this) {
		this->head = head;
		this->tail = tail;
	}
	return this;
}

WaypointSequence *newWaypointSequence(WaypointSequence *this, Waypoint *head) {
	if (this == NULL || head == NULL) {
		return NULL;
	}
	Waypoint *current = head;

	// Find tail.
	while (current->next) {
		current = current->next;
	}
	this->head = head;
	this->tail = current;

	return this;
}
/*
WaypointSequence *addWaypoint(WaypointSquence *this, Waypoint *new) {
	if (this) {
		if (!this->head) {  // 0 Waypoints.
			this->head = new;
		} else if (!this->tail) {  // 1 Waypoint.
			this->head->next = new;
			this->tail = new;
		} else {
			this->
		}
	}
}
*/
#endif  // WAYPOINTSEQUENCE_C_
