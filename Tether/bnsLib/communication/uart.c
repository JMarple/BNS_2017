#pragma systemFile

#if !defined(UART_C_)
#define UART_C_

/**
 * Get next character from UART port.
 *
 * @param 	port	UART port to get from.
 *
 * @return	Next character.
 */
unsigned char getNextChar(TUARTs port) {
	short c = getChar(port);

	// Wait for valid character.
	while (c < 0) {
		sleep(3);
		c = getChar(port);
	}
	return (unsigned char)c;
}

/**
 * Get next word from UART port.
 *
 * @param 	port	UART port to get from.
 *
 * @return	Next word from UART port.
 */
unsigned short getNextWord(TUARTs port) {
	// This routine assumes little-endian.
	return getNextChar(port) | ((unsigned short)getNextChar(port) << 8);
}

/**
 * Get next word from UART port.
 *
 * @param 	port	UART port to get from.
 *
 * @return	Next word from UART port.
 */
short getNextSignedWord(TUARTs port) {
	// This routine assumes little-endian.
	return getNextChar(port) | ((short)getNextChar(port) << 8);
}

/**
 * Get next int from UART port.
 *
 * @param 	port	UART port to get from.
 *
 * @return	Next int from UART port.
 */
int getNextSignedInt(TUARTs port) {
	// This routine assumes little-endian.
	return getNextWord(port) | ((int)getNextWord(port) << 16);
}

/**
 * Send character array to UART port.
 *
 * @param 	port	Port to send to.
 * @param 	data	Character array to send.
 * @param 	len 	Array length.
 *
 * @return	Array length.
 */
short sendChars(TUARTs port, unsigned char *data, short len) {
	for (short i = 0; i < len; i++) {
		// Wait for transmit buffer to be empty or timeout.
		for (short timeout = 0; !bXmitComplete(port) && timeout < 20; timeout++) {
		}
		sendChar(port, data[i]);
	}
	return len;
}

/**
 * Send word to UART port.
 *
 * @param 	port	Port to send to.
 * @param 	w   	Word to send.
 */
void sendWord(TUARTs port, unsigned short w) {
	unsigned char data[2] = {w & 0xff, w >> 8};
	sendChars(port, data, sizeof(data) / sizeof(data[0]));
}

/**
 * Send word to UART port.
 *
 * @param 	port	Port to send to.
 * @param 	w   	Word to send.
 */
void sendSignedWord(TUARTs port, short w) {
	unsigned char data[2] = {w & 0xff, w >> 8};
	sendChars(port, data, sizeof(data) / sizeof(data[0]));
}

/**
 * Send int to UART port.
 *
 * @param 	port	Port to send to.
 * @param 	i   	Int to send.
 */
void sendSignedInt(TUARTs port, int i) {
	unsigned char data[4] = {i & 0xff, (i >> 8) & 0xff, (i >> 16) & 0xff, i >> 24};
	sendChars(port, data, sizeof(data) / sizeof(data[0]));
}

#endif  // UART_C_
