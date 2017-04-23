#if !defined(SETUP_H_)
#define SETUP_H_

// Master motor ports.
#define MASTER_DRIVE_MOTOR_L  	port9
#define MASTER_DRIVE_MOTOR_R  	port2
#define MASTER_LIFT_MOTOR     	port3
#define MASTER_LIFT_MOTOR_2   	port7

// Slave motor ports.
#define SLAVE_DRIVE_MOTOR_L   	port6
#define SLAVE_DRIVE_MOTOR_R   	port5
#define SLAVE_DRIVE_MOTOR_R2 	port4
#define SLAVE_LIFT_MOTOR      	port8

// Master digital ports.
#define MASTER_DRIVE_ENCODER  	dgtl5
#define MASTER_DRIVE_ENCODER_2	dgtl6
#define MASTER_FRONT_SONAR    	dgtl7
#define MASTER_FRONT_SONAR_2  	dgtl8
#define MASTER_CLAW_SOLENOID  	dgtl3

// Slave digital ports.
#define SLAVE_CLAW_SOLENOID   	dgtl4

// Master analog ports.
#define MASTER_GYRO           	in1
#define MASTER_LEFT_LINE      	in2
#define MASTER_RIGHT_LINE     	in3

// Slave analog ports.
#define SLAVE_GYRO            	in4

// Master serial ports.
#define MASTER_PIXY           	UART1

#endif  // SETUP_H_
