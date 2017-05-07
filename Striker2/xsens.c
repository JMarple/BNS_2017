void flushTX(TUARTs uartPort)
{
  while (1==1)
  {
    short c = getChar(uartPort);
    if (c == -1) return;
  }
}

struct Quaternion
{
  // 0 = w, 1 = x, 2 = y, 3 = z
  float data[4];
}

// t = q * r
void mulQuaternions(Quaternion* t, Quaternion* q, Quaternion* r)
{
  float q0 = q->data[0]; float q1 = q->data[1];
  float q2 = q->data[2]; float q3 = q->data[3];
  float r0 = r->data[0]; float r1 = r->data[1];
  float r2 = r->data[2]; float r3 = r->data[3];

  t->data[0] = (r0*q0 - r1*q1 - r2*q2 - r3*q3);
  t->data[1] = (r0*q1 + r1*q0 - r2*q3 + r3*q2);
  t->data[2] = (r0*q2 + r1*q3 + r2*q0 - r3*q1);
  t->data[3] = (r0*q3 - r1*q2 + r2*q1 + r3*q0);
}

float getYaw(Quaternion* q)
{
  float qw = q->data[0];
  float qx = q->data[1];
  float qy = q->data[2];
  float qz = q->data[3];
  float yaw = 57.2958 * atan2(2 * (qx*qy + qz*qw), qx*qx - qy*qy - qz*qz + qw*qw);

  return yaw;
}

float castIntToFloat(int a)
{
  int sign = (a & 0x80000000);
  int exponent = ((a & 0x7F800000) >> 23) - 127;
  float mantissa = 1 + ((a & 0x007FFFFF) / pow(2, 23));

  float value = pow(2, exponent) * mantissa;

  if (sign == 0x80000000)
  {
    value = -value;
  }

  return value;
}

struct MTMessage
{
  int mid;
  int len;
  char data[256];
}

struct MT2Data
{
  Quaternion dQ;
  float mag[3];
  float rateOfTurn[3];
  float dV[4];
  int sampleTimeFine;
  int packetCounter;
  int statusWord;
}

char getCharFromUART(TUARTs uartPort)
{
  while (1==1)
  {
    short c = getChar(uartPort);
    if (c != -1)
    {
      return c;
    }
  }
}

void getFloatsFromBytes(float* output, char* data, int len)
{
  for (int j = 0; j < len; j+=4)
  {
    int a = data[j+3] | (data[j+2] << 8) | (data[j+1] << 16) | (data[j] << 24);
    float value = castIntToFloat(a);
    output[j/4] = value;
  }
}

void getIntsFromBytes(int* output, char* data, int len)
{
  for (int j = 0; j < len; j+= 4)
  {
    output[j/4] = data[j+3] | (data[j+2] << 8) | (data[j+1] << 16) | (data[j] << 24);
  }
}

void sendMTMessage(struct MTMessage* msg, TUARTs uartPort)
{
  char checksum = 0xFF;
  sendChar(uartPort, 0xFA);
  sendChar(uartPort, 0xFF);

  checksum += msg->mid;
  sendChar(uartPort, msg->mid);
  checksum += msg->len;
  sendChar(uartPort, msg->len);

  for (int i = 0; i < msg->len; i++)
  {
    checksum += msg->data[i];
    sendChar(uartPort, msg->data[i]);
  }

  char checksumResult = 0x00 - checksum;
  sendChar(uartPort, checksumResult);
}

void getMTMessage(struct MTMessage* msg, TUARTs uartPort)
{
  while (1==1)
  {
    char checksum = 0;

    // Wait for preamble
    while (getCharFromUART(uartPort) != 0xFA){;}

    // BID
    checksum += getCharFromUART(uartPort);

    // MID
    msg->mid = getCharFromUART(uartPort);
    checksum += msg->mid;

    // Payload length
    msg->len = getCharFromUART(uartPort);
    checksum += msg->len;

    // Payload
    for (int i = 0; i < msg->len; i++)
    {
      msg->data[i] = getCharFromUART(uartPort);
      checksum += msg->data[i];
    }

    // Checksum (ignoring for now)
    checksum += getCharFromUART(uartPort);

    if (checksum != 0)
    {
      writeDebugStreamLine("Bad Checksum!");
    }
    else
    {
      break;
    }
  }
}

#define XDI_PACKETCOUNTER 0x1020
#define XDI_SAMPLETIMEFINE 0x1060
#define XDI_DELTAQ 0x8030
#define XDI_RATEOFTURN 0x8020
#define XDI_MAGNETICFIELD 0xC020
#define XDI_DELTAV 0x4010
#define XDI_STATUSWORD 0xE020

void parseMT2Data(struct MT2Data* out, char* data, int len)
{
  for (int i = 0; i < len; )
  {
    // Get data code
    int x = (data[i] << 8) | data[i+1];
    i+=2;

    // Get Length
    int len = data[i];
    i+=1;

    // Interpret different types of packets
    if (x == XDI_DELTAQ)
    {
      getFloatsFromBytes(out->dQ.data, &data[i], len);
      out->dQ.data[1] += 0.000000;
      out->dQ.data[2] += 0.000001;
      out->dQ.data[3] -= 0.000005; // gyro bias
    }
    else if (x == XDI_PACKETCOUNTER) out->packetCounter = data[i+1] | (data[i] << 8);
    else if (x == XDI_SAMPLETIMEFINE) getIntsFromBytes(&out->sampleTimeFine, &data[i], len);
    else if (x == XDI_RATEOFTURN) getFloatsFromBytes(out->rateOfTurn, &data[i], len);
    else if (x == XDI_MAGNETICFIELD) getFloatsFromBytes(out->mag, &data[i], len);
    else if (x == XDI_DELTAV) getFloatsFromBytes(out->dV, &data[i], len);
    else if (x == XDI_STATUSWORD) getIntsFromBytes(&out->statusWord, &data[i], len);
    else
    {
      writeDebugStreamLine("Unknown code %x", x);
    }
    i += len;
  }
}

/*void PrintMTConfigurations(TUARTs uartPort)
{
	MTMessage msg, sendMsg;

	// Ensure there are no random messages from previous sessions
	// still in the tx buffer.
	flushTX(uartPort);

	// Go into config mode.
	sendMsg.mid = 0x30;
	sendMsg.len = 0x00;
	sendMTMessage(&sendMsg, uartPort);

	wait1Msec(500);

	flushTX(uartPort);

	// Get device serial number
	sendMsg.mid = 0x00;
	sendMsg.len = 0x00;
	sendMTMessage(&sendMsg, uartPort);

	writeDebugStreamLine("Getting MTi Device..");

	getMTMessage(&msg, uartPort);
	writeDebugStream("Serial Number: ");
	for (int i = 0; i < msg.len; i++) writeDebugStream("%x%x", (msg.data[i]>>4)&0xF, msg.data[i]&0xF);
	writeDebugStreamLine("");

	// Get product code
	sendMsg.mid = 0x1C;
	sendMsg.len = 0x00;
	sendMTMessage(&sendMsg, uartPort);

	getMTMessage(&msg, uartPort);
	writeDebugStream("Product Code: ");
	for (int i = 0; i < msg.len; i++) writeDebugStream("%c", msg.data[i]);
	writeDebugStreamLine("");
}
*/
float gyroCalibYaw = 0;
float gyroYaw = 0;
float gyroYawRate = 0;
float previousGyroYaw = 0;
MT2Data mtData;
int packetCount = 0;
Quaternion heading;

task xsens()
{
  TUARTs uartPort = UART1;

  wait1Msec(500);

  //PrintMTConfigurations(uartPort);

  heading.data[0] = 1;
  heading.data[1] = 0;
  heading.data[2] = 0;
  heading.data[3] = 0;

  MTMessage sendMsg;

  // Go into measurement
  sendMsg.mid = 0x10;
  sendMsg.len = 0x00;
  sendMTMessage(&sendMsg, uartPort);

  writeDebugStreamLine("IMU going into measurement mode");

  while (1==1)
  {
    MTMessage msg;
    getMTMessage(&msg, uartPort);

    // Message
    if (msg.mid == 0x36)
    {
      parseMT2Data(&mtData, msg.data, msg.len);
      //static float firstCompass = 1;
      //static float compassYawOffset = 0;

      //float compassYaw = 57.2958 * atan2(data.mag[0], data.mag[1]);
      //if (firstCompass)
      //{
      //firstCompass = 0;
      //compassYawOffset = compassYaw;
      //}

      mulQuaternions(&heading, &heading, &mtData.dQ);

      //static float complementaryYaw = 0;
      gyroYaw = getYaw(&heading) - gyroCalibYaw*packetCount;

      gyroYawRate = (gyroYaw - previousGyroYaw);
      previousGyroYaw = gyroYaw;

      //float alpha = 0.98;

      //complementaryYaw = alpha * (complementaryYaw + dGyro) + (1-alpha) * (compassYaw - compassYawOffset);
      //static float velocity = 0;
      //velocity += data.dV[0] + 0.00156;

      //writeDebugStreamLine("mag = %f %f %f", data.mag[0], data.mag[1], data.mag[2]);
      //writeDebugStreamLine("%f %f, %f %f", minX, maxX, minY, maxY);
      //writeDebugStreamLine("Q = %f %f %f %f", heading.data[0], heading.data[1], heading.data[2], heading.data[3]);
      //writeDebugStreamLine("Vel = %f, %f %f %f", velocity, data.dV[0], data.dV[1], data.dV[2]);
      //writeDebugStreamLine("Result = %d %d, %f %f %f", data.packetCounter, data.sampleTimeFine, gyroYaw, compassYaw - compassYawOffset, complementaryYaw);
      packetCount++;
    }
  }
}
