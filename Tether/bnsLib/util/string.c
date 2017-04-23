#pragma systemFile

#if !defined(STRING_C_)
#define STRING_C_

char *toString(bool b) {
	return b ? "true" : "false";
}

char *toString(tMotor port) {
	if (port < port1 || port > port10) {
		return "";
	}
	char str[7];

	sprintf(str, "port%d", (short)(port - port1) + 1);

	return str;
}

char *toString(tSensors port) {
	short portNumber = (short)port;
	char portType[5];
	char str[7];

	if (port >= in1 && port <= in8) {
		sprintf(portType, "in");
		portNumber -= (short)in1;
	} else if (port >= dgtl1 && port <= dgtl12) {
		sprintf(portType, "dgtl");
		portNumber -= (short)dgtl1;
	} else if (port >= I2C_1 && port <= I2C_8) {
		sprintf(portType, "I2C_");
		portNumber -= (short)I2C_1;
	} else {
		return "";
	}
	sprintf(str, "%s%d", portType, portNumber + 1);

	return str;
}

#endif  // STRING_C_
