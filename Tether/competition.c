// Forward references for functions in the competition template.
void pre_auton();
task autonomous();
task usercontrol();

// There will also be many other errors but perhaps this will be noticed as it
// will be at the top of the list.
#if !defined(VEX2)
#error "Switch to the VEX cortex platform"
#endif

// Forward references for functions in this file.
void allMotorsOff();

task main() {
	// Master CPU will not let competition start until powered on for at least
	// two seconds.
	sleep(2000);

	// Call function where the user can initialize sensors and variables.
	pre_auton();

	while (true) {
		// Remain in this loop while the robot is disabled.
		while (bIfiRobotDisabled) {
			sleep(25);
		}
		// The robot has become enabled
		if (bIfiAutonomousMode) {
			// Start the autonomous task.
			startTask(autonomous);

			// Waiting for autonomous phase to end.
			while (bIfiAutonomousMode && !bIfiRobotDisabled) {
				if (!bVEXNETActive && nVexRCReceiveState == vrNoXmiters) {
					// The transmitters are powered off!
					allMotorsOff();
				}
				sleep(25);
			}
			allMotorsOff();
		} else {
			// Start the usercontrol task
			startTask(usercontrol);

			// Here we repeat loop waiting for user control to end and
			// (optionally) start of a new competition run.
			while (!bIfiAutonomousMode && !bIfiRobotDisabled) {
				if (nVexRCReceiveState == vrNoXmiters) {
					// The transmitters are powered off!
					allMotorsOff();
				}
				sleep(25);
			}
			allMotorsOff();
		}
	}
}

/**
 * Function that will stop all motors.
 */
void allMotorsOff() {
	motor[port1] = 0;
	motor[port2] = 0;
	motor[port3] = 0;
	motor[port4] = 0;
	motor[port5] = 0;
	motor[port6] = 0;
	motor[port7] = 0;
	motor[port8] = 0;
	motor[port9] = 0;
	motor[port10] = 0;
}
