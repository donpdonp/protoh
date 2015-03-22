// SUPER Basic proof of conecpet code that
// triggers the lowest LED on one of 4 corners
// ttosi - 3/20/15

include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define DEBUG

#define FRONT_RIGHT_LED 3
#define FRONT_LEFT_LED 4
#define BACK_RIGHT_LED 5
#define BACK_LEFT_LED 6

enum ledLocation
{
	NONE,
	FRONT_RIGHT,
	FRONT_LEFT,
	BACK_RIGHT,
	BACK_LEFT
};

MPU6050 mpu;
Quaternion q;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

volatile bool mpuInterrupt = false;

void dmpDataReady() 
{
	mpuInterrupt = true;
}

void setup()
{
	pinMode(FRONT_RIGHT_LED, OUTPUT);
	pinMode(FRONT_LEFT_LED, OUTPUT);
	pinMode(BACK_RIGHT_LED, OUTPUT);
	pinMode(BACK_LEFT_LED, OUTPUT);

	Wire.begin();
	TWBR = 24;

	#ifdef DEBUG
		Serial.begin(19200);
	#endif

	mpu.initialize();
	devStatus = mpu.dmpInitialize();

	if (devStatus == 0) 
	{
		mpu.setDMPEnabled(true);
		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		dmpReady = true;
		packetSize = mpu.dmpGetFIFOPacketSize();
	} 
}

void setCornerLed(ledLocation led = NONE)
{
	digitalWrite(FRONT_RIGHT_LED, 0);
	digitalWrite(FRONT_LEFT_LED, 0);
	digitalWrite(BACK_RIGHT_LED, 0);
	digitalWrite(BACK_LEFT_LED, 0);
	
	switch (led)
	{
		case FRONT_RIGHT:
			digitalWrite(FRONT_RIGHT_LED, 1);
			break;
		case FRONT_LEFT:
			digitalWrite(FRONT_LEFT_LED, 1);
			break;
		case BACK_RIGHT:
			digitalWrite(BACK_RIGHT_LED, 1);
			break;
		case BACK_LEFT:
			digitalWrite(BACK_LEFT_LED, 1);
			break;
	}
}

void loop() 
{
	if (!dmpReady) return;

	while (!mpuInterrupt && fifoCount < packetSize) { }

	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();
	fifoCount = mpu.getFIFOCount();

	if (mpuIntStatus & 0x02) 
	{
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		fifoCount -= packetSize;
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		
		if((q.x >= -.03 && q.x <= .03) && (q.y >= -.03 && q.y <= .03))
		{
			setCornerLed(NONE);
		}
		else if(q.x <= .03)
		{
			if(q.y >= .03)
			{
				setCornerLed(FRONT_RIGHT);
			}
			else
			{
				setCornerLed(FRONT_LEFT);
			}
		}
		else if(q.x >= .03)
		{
			if(q.y >= .03)
			{
				setCornerLed(BACK_RIGHT);
			}
			else
			{
				setCornerLed(BACK_LEFT);
			}
		}

		#ifdef DEBUG
			Serial.print("quat\t");
			Serial.print(q.w);
			Serial.print("\t");
			Serial.print(q.x);
			Serial.print("\t");
			Serial.print(q.y);
			Serial.print("\t");
			Serial.println(q.z);
		#endif
	}
}
