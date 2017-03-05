#ifndef __LCD_C__
#define __LCD_C__

bool redSide = true;
bool blueSide = false;
int autonMode = 0;

task LCD()
{
	string mainBattery, backupBattery;
	const short leftButton = 1;
	const short centerButton = 2;
	const short rightButton = 4;

	while(true)																												 // An infinite loop to keep the program running until you terminate it
	{
		clearLCDLine(0);																						// Clear line 1 (0) of the LCD
		clearLCDLine(1);																						// Clear line 2 (1) of the LCD

		//Display the Primary Robot battery voltage
		displayLCDString(0, 0, "Primary: ");
		sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0,'V'); //Build the value to be displayed
		displayNextLCDString(mainBattery);

		//Display the Backup battery voltage
		displayLCDString(1, 0, "Expander: ");
		sprintf(backupBattery, "%1.2f%c", SensorValue(status) * 0.00546, 'V');		//Build the value to be displayed SensorValue(status)/275.0
		displayNextLCDString(backupBattery);

		//auton selection
		if(nLCDButtons == leftButton) {
			bLCDBacklight = true;
			redSide = true;
			blueSide = false;
			clearLCDLine(0);
			clearLCDLine(1);
			displayLCDString(0, 0, "Autonomous:");
			displayLCDString(1, 0, "Red side");
		}

		else if(nLCDButtons == rightButton) {
			bLCDBacklight = true;
			redSide = false;
			blueSide = true;
			clearLCDLine(0);
			clearLCDLine(1);
			displayLCDString(0, 0, "Autonomous:");
			displayLCDString(1, 0, "Blue side");
		}

		else if(nLCDButtons == centerButton) {
			bLCDBacklight = true;
			if(autonMode == 0){
				autonMode = 1;
				clearLCDLine(0);
				clearLCDLine(1);
				displayLCDString(0, 0, "Autonomous:");
				displayLCDString(1, 0, "Disabled");
			}
			else if(autonMode == 1){
				autonMode = 0;
				clearLCDLine(0);
				clearLCDLine(1);
				displayLCDString(0, 0, "Autonomous:");
				displayLCDString(1, 0, "Enabled");
			}
			wait1Msec(400);
		}
		else
		{
			bLCDBacklight = false;
		}
		//Short delay for the LCD refresh rate
		wait1Msec(100);
	}
}

#endif
