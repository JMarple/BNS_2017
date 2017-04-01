#pragma systemFile

#if !defined(PIXY_C_)
#define PIXY_C_

// Communication/miscellaneous parameters.
#define PIXY_ARRAY_SIZE     	30
#define PIXY_START_WORD     	0xaa55
#define PIXY_START_WORD_CC  	0xaa56
#define PIXY_START_WORDX    	0x55aa
#define PIXY_MAX_SIGNATURE  	7

// Pixy x-y position values.
#define PIXY_MIN_X          	0
#define PIXY_MAX_X          	319
#define PIXY_MIN_Y          	0
#define PIXY_MAX_Y          	199

// RC-servo values.
#define PIXY_RCS_MIN_POS    	0
#define PIXY_RCS_MAX_POS    	1000
#define PIXY_RCS_CENTER_POS 	((PIXY_RCS_MAX_POS - PIXY_RCS_MIN_POS) / 2)

/**
* Get next character from UART port.
*
* @param 	port	UART port to get from.
*
* @return	Next character.
*/
unsigned char getNextChar(TUARTs port) {
	short c;

	// Wait for valid character.
	while ((c = getChar(port)) < 0) {
		sleep(3);
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

	char n1 = getNextChar(port);
	char n2 = getNextChar(port);

	return n1 | (n2 << 8);
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

typedef enum PixyBlockType {
	NORMAL_BLOCK,
	CC_BLOCK  // Color code block.
} PixyBlockType;

typedef struct {
	unsigned short signature;
	unsigned short x;
	unsigned short y;
	unsigned short width;
	unsigned short height;
	unsigned short angle;  // Angle is only available for color coded blocks.
} PixyBlock;

typedef struct {
	TUARTs port;
	bool skipStart;
	PixyBlockType blockType;
	unsigned short blockCount;
	PixyBlock blocks[PIXY_ARRAY_SIZE];
} Pixy;

PixyBlock* pixyGetLargestBlock(Pixy* this, int sig)
{
	PixyBlock* res = (PixyBlock*)0;
	int largest = 0;

	for (int i = 0; i < this->blockCount; i++)
	{
		//if (this->blocks[i].signature != sig) continue;

		//filter out part of the fov
		if (this->blocks[i].y < 110
			&& this->blocks[i].x > ((-2.0/9.0) * this->blocks[i].y + 90.)
			&& this->blocks[i].x < ((2.0/9.0) * this->blocks[i].y + 260.))
		{
			int val = this->blocks[i].width * this->blocks[i].height;
			if (val > largest)
			{
				res = &this->blocks[i];
				largest = val;
			}
		}
	}

	return res;
}

PixyBlock* pixyGetLargestBlockInClaw(Pixy* this, int sig)
{
	PixyBlock* res = (PixyBlock*)0;
	int largest = 0;

	for (int i = 0; i < this->blockCount; i++)
	{
		//if (this->blocks[i].signature != sig) continue;

		//filter out part of the fov
		if (this->blocks[i].y > 110
			&& this->blocks[i].x > 100
			&& this->blocks[i].x < 250)
		{
			int val = this->blocks[i].width * this->blocks[i].height;
			if (val > largest)
			{
				res = &this->blocks[i];
				largest = val;
			}
		}
	}

	return res;
}

/**
* Initialize Pixy.
*
* @param 	this    	Pointer to Pixy struct.
* @param 	port    	UART port the Pixy is plugged into.
* @param 	baudRate	Baud rate the Pixy is set to use.
*
* @return	Pointer to Pixy struct.
*/
Pixy *newPixy(Pixy *this, TUARTs port, TBaudRate baudRate) {
	if (this) {
		setBaudRate(port, baudRate);

		this->port = port;
		this->skipStart = false;
		this->blockCount = 0;
	}
	return this;
}

/**
* Initialize Pixy.
*
* @param 	this	Pointer to Pixy struct.
* @param 	port	UART port the Pixy is plugged into.
*
* @return	Pointer to Pixy struct.
*/
Pixy *newPixy(Pixy *this, TUARTs port) {
	return newPixy(this, port, baudRate19200);
}

/**
* Initialize Pixy.
*
* @param 	this	Pointer to Pixy struct.
*
* @return	Pointer to Pixy struct.
*/
Pixy *newPixy(Pixy *this) {
	if (this) {
		this->port = (TUARTs)-1;
		this->skipStart = false;
		this->blockCount = 0;
	}
	return this;
}

/**
* Get start word from Pixy.
*
* @param 	this	Pointer to Pixy struct.
*
* @return	true if start code found, false otherwise.
*/
bool getPixyStart(Pixy *this) {
	if (this == NULL) {
		return false;
	}
	TUARTs port = this->port;
	unsigned short lastW = 0xffff;  // Some inconsequential initial value.
	unsigned short w;

	while (true) {
		w = getNextWord(port);
		if (w == 0 && lastW == 0) {
			return false;  // No start code.
			} else if (w == PIXY_START_WORD && lastW == PIXY_START_WORD) {
			this->blockType = NORMAL_BLOCK;
			return true;  // Code found!
			} else if (w == PIXY_START_WORD_CC && lastW == PIXY_START_WORD_CC) {
			this->blockType = CC_BLOCK;  // Found color code block.
			return true;
			} else if (w == PIXY_START_WORDX) {  // This is important, we might be juxtaposed.
			getChar(port);  // We're out of sync (backwards)!
		}
		lastW = w;
	}
}

/**
* Update block data for Pixy.
*
* @param 	this	Pointer to Pixy struct.
*
* @return	Number of blocks found.
*/
unsigned short pixyUpdate(Pixy *this) {
	if (this == NULL) {
		return 0;
	}
	TUARTs port = this->port;
	unsigned short blockCount = 0;
	unsigned short w, checksum, sum;
	PixyBlock *block;

	if (!this->skipStart) {
		if (!getPixyStart(this)) {
			this->blockCount = 0;
			return 0;
		}
		} else {
		this->skipStart = false;
	}
	while (blockCount < PIXY_ARRAY_SIZE) {
		checksum = getNextWord(port);
		if (checksum == PIXY_START_WORD) {
			// We've reached the beginning of the next frame.
			this->skipStart = true;
			this->blockType = NORMAL_BLOCK;
			this->blockCount = blockCount;
			break;
			} else if (checksum == PIXY_START_WORD_CC) {
			this->skipStart = true;
			this->blockType = CC_BLOCK;
			this->blockCount = blockCount;
			break;
			} else if (checksum == 0) {
			this->blockCount = blockCount;
			break;
		}
		block = &this->blocks[blockCount];

		block->signature = w = getNextWord(port);
		sum = w;

		block->x = w = getNextWord(port);
		sum += w;

		block->y = w = getNextWord(port);
		sum += w;

		block->width = w = getNextWord(port);
		sum += w;

		block->height = w = getNextWord(port);
		sum += w;

		// No angle for regular block.
		block->angle = w = (this->blockType == NORMAL_BLOCK) ? 0 : getNextWord(port);
		sum += w;

		// Check checksum.
		//if (checksum == sum) {
		blockCount++;
		//} else {
		//	writeDebugStream("Pixy update checksum error!\n");
		//}
		w = getNextWord(port);
		if (w == PIXY_START_WORD) {
			this->blockType = NORMAL_BLOCK;
			} else if (w == PIXY_START_WORD_CC) {
			this->blockType = CC_BLOCK;
			} else {
			this->blockCount = blockCount;
			break;
		}
	}
	return blockCount;
}

/**
* Get number of blocks last recorded by Pixy via update().
*
* @param 	this	Pointer to Pixy struct.
*
* @return	Number of blocks.
*/
unsigned short getBlockCount(Pixy *this) {
	return this ? this->blockCount : 0;
}

/**
* Set Pixy camera brightness.
*
* @param 	this      	Pointer to Pixy struct.
* @param 	beightness	Brightness value.
*
* @return	Number of characters sent via sendChars().
*/
short setBrightness(Pixy *this, unsigned char brightness) {
	if (this == NULL) {
		return 0;
	}
	unsigned char outBuf[3] = {0x00, 0xfe, brightness};

	return sendChars(this->port, outBuf, sizeof(outBuf) / sizeof(outBuf[0]));
}

/**
* Set Pixy LED color.
*
* @param 	this	Pointer to Pixy struct.
* @param 	r   	Red intensity.
* @param 	g   	Green intensity.
* @param 	b   	Blue intensity.
*
* @return	Number of characters sent via sendChars().
*/
short setLED(Pixy *this, unsigned char r, unsigned char g, unsigned char b) {
	if (this == NULL) {
		return 0;
	}
	unsigned char outBuf[5] = {0x00, 0xfd, r, g, b};

	return sendChars(this->port, outBuf, sizeof(outBuf) / sizeof(outBuf[0]));
}

/**
* Set pan/tilt servo positions.
*
* @param 	this	Pointer to Pixy struct.
* @param 	s0  	Servo 0 (pan) position.
* @param 	s1  	Servo 1 (tilt) position.
*
* @return	Number of characters sent via sendChars().
*/
short setServos(Pixy *this, unsigned short s0, unsigned short s1) {
	if (this == NULL) {
		return 0;
	}
	unsigned char outBuf[6];

	outBuf[0] = 0x00;
	outBuf[1] = 0xff;
	outBuf[2] = s0;
	outBuf[4] = s1;

	return sendChars(this->port, outBuf, sizeof(outBuf) / sizeof(outBuf[0]));
}

/**
* Print single Pixy block data to debug stream.
*
* @param 	this	Pointer to PixyBlock struct.
*/
void pixyPrint(PixyBlock *this) {
	if (this == NULL) {
		return;
	}
	unsigned short signature = this->signature;

	if (signature > PIXY_MAX_SIGNATURE) {  // Color code (CC)!
		writeDebugStream(
		"CC block! sig: %o (%d decimal) x: %d y: %d width: %d height: %d angle: %d\n",
		signature, signature, this->x, this->y, this->width, this->height, this->angle);
		} else {  // Regular block. Note, angle is always zero, so no need to print.
		writeDebugStream("sig: %d x: %d y: %d width: %d height: %d\n", signature,
		this->x, this->y, this->width, this->height);
	}
}

/**
* Print all Pixy block data to debug stream.
*
* @param 	this	Pointer to Pixy struct.
*/
void pixyPrint(Pixy *this) {
	if (this == NULL) {
		return;
	}
	unsigned short blockCount = this->blockCount;

	writeDebugStream("Detected %d:\n", blockCount);

	for (unsigned short i = 0; i < blockCount; i++)
	{
		if (this->blocks[i].signature == 1)
		{
			writeDebugStream("\tblock %d: ", i);

			pixyPrint(&this->blocks[i]);
		}
	}
}

#endif
