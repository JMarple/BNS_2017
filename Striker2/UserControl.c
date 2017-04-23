#ifndef __USER_CONTROL_C__
#define __USER_CONTROL_C__

task autokiller()
{
	while (true)
	{
		if (vexRT[Btn7R]) forceQuitAuton();
	}
}

task usercontrol()
{
	stopTask(liftHeight);
	startTask(autokiller);
	//Create "deadzone" variables. Adjust threshold value to increase/decrease deadzone
	int X = 0, Y1 = 0, Y2 = 0, threshold = 15;
	int direction = 0;

	//Loop Forever
	while(1 == 1)
	{
		//Create "deadzone" for Y1/Ch3
		if(abs(vexRT[Ch3]) > threshold)
			Y1 = vexRT[Ch3];
		else
			Y1 = 0;
		//Create "deadzone" for X1/Ch4
		if(abs(vexRT[Ch2]) > threshold)
			Y2 = vexRT[Ch2];
		else
			Y2 = 0;
		//Create "deadzone" for X2/Ch1
		if(abs(vexRT[Ch1]) > threshold)
			X = vexRT[Ch1];
		else
			X = 0;

		//Remote Control Commands
		leftDrive(Y1);
		rightDrive(Y2);

		if(vexRT(Btn6U) /*&& SensorValue(pot) < 2600*/)
		{
      liftPower(127);
			direction = 1;
			//if(SensorValue(pot) > 2500) {
			//	SensorValue(intake) = 0;
			//}
		}
		else if(vexRT(Btn6D))
		{
      liftPower(-127);
			direction = -1;
	  }/*
	  else if(SensorValue(pot) > 2600)
	  {
	  	liftPower(-15);
	  }*/
	  else
	  {
	  	liftPower(direction * 15);
	  }

	  if(vexRT(Btn5U) && vexRT(Btn5D))
		{
			//SensorValue(wheels) = !SensorValue(wheels);
			wait1Msec(300);
		}
		else if(vexRT(Btn5U))
		{
			SensorValue(intake) = 1;
		}
		else if(vexRT(Btn5D))
		{
			SensorValue(intake) = 0;
		}

		if (vexRT(Btn8U)) SensorValue[pusher] = 1;
		if (vexRT(Btn8D)) SensorValue[pusher] = 0;

		if (vexRT(Btn8L)) SensorValue[hangingLock] = 1;
		if (vexRT(Btn8R)) SensorValue[hangingLock] = 0;

		if (vexRT[Btn7R]) forceQuitAuton();

		if (vexRT[Btn7L])
		{


		  //PSC();
		  startTask(PSC_Driver);
		  //while (1==1);
		  while (vexRT[Btn7R] == 0)
			{
			  delay(20);
			}
		  //stopTask(PSC_Driver);

		  //AutonCubeNear(FULL_NEAR_MODE);
		  //PSC();
		  //SetLiftHeight(LIFT_HIGH_HEIGHT);
		  //PSC_HangingPushing(currentHeading);
		  //PixyAuton();
		  //currentHeading += ROTATE_LEFT / 2;
		  //driveOneWheel(currentHeading);
		  //PSC();
		  //startTask(liftHeight);
		  //currentHeading = PushTwo(currentHeading);
		  //currentHeading = PSC_HangingPushing(currentHeading);
		  //AutonCubeFar();
		  //DrivePixyStraight(1000, 60);
		  //driveHoldHeading(100, 80, 0);
		  //driveCorrectedSmoothTurn(-90, 60);
      //driveStraight(300, 80);
		}

		if(vexRT(Btn7U)) toggleSonar(FRONT_ON);
		if(vexRT(Btn7D)) toggleSonar(BACK_ON);
	}
}


#endif
