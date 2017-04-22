#pragma systemFile

#if !defined(PID_C_)
#define PID_C_

typedef struct {
	float Kp;
	float Ki;
	float Kd;

	float setpoint;

	long time;
	float integral;
	float error;

	float controlVariable;

	TSemaphore sem;
} Pid;

Pid *newPid(Pid *this, float Kp, float Ki, float Kd, float setpoint) {
	if (this) {
		this->Kp = Kp;
		this->Ki = Ki;
		this->Kd = Kd;

		this->setpoint = setpoint;

		this->time = 0;
		this->integral = 0.0;
		this->error = 0.0;

		this->controlVariable = 0.0;

		semaphoreInitialize(this->sem);
	}
	return this;
}

float getKp(Pid *this) {
	return this ? this->Kp : 0.0;
}

void setKp(Pid *this, float Kp) {
	if (this) {
		this->Kp = Kp;
	}
}

float getKi(Pid *this) {
	return this ? this->Ki : 0.0;
}

void setKi(Pid *this, float Ki) {
	if (this) {
		this->Ki = Ki;
	}
}

float getKd(Pid *this) {
	return this ? this->Kd : 0.0;
}

void setKd(Pid *this, float Kd) {
	if (this) {
		this->Kd = Kd;
	}
}

float getSetpoint(Pid *this) {
	return this ? this->setpoint : 0.0;
}

void setSetpoint(Pid *this, float setpoint) {
	if (this) {
		semaphoreLock(this->sem);

		this->setpoint = setpoint;
		this->time = 0;

		if (bDoesTaskOwnSemaphore(this->sem)) {
			semaphoreUnlock(this->sem);
		}
	}
}

float getError(Pid *this) {
	return this ? this->error : 0.0;
}

float getControlVariable(Pid *this) {
	return this ? this->controlVariable : 0.0;
}

void update(Pid *this, float processVariable) {
	if (this == NULL) {
		return;
	}
	semaphoreLock(this->sem);

	float error = this->setpoint - processVariable;

	if (this->time == 0) {
		this->time = nSysTime;
		this->error = error;
	}
	long dt = nSysTime - this->time;

	this->integral += error * dt;

	float derivative = (dt == 0) ? 0.0 : ((error - this->error) / dt);

	this->controlVariable = this->Kp * error + this->Ki * this->integral + this->Kd * derivative;

	this->time += dt;
	this->error = error;

	if (bDoesTaskOwnSemaphore(this->sem)) {
		semaphoreUnlock(this->sem);
	}
}

#endif  // PID_C_
