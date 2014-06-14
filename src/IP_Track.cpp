#include "arminno.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#define TRACK_FLAG

/*************/
/* COMPONENT */
/*************/
MotorRunnerC myMotorL(31);
MotorRunnerC myMotorR(30);
SonarA mySonar(7);	
CompassB myCompass(5);
LCD2X16A myLCD(3);

/************/
/* FUNCTION */
/************/
void TrackControl();
void PIDControl();
void SetMotor(short ControlValue);
void StopMotor();
void Transmit(char* ch);
void Transmit(int num, bool end);

/*******************/
/* PUBLIC VARIABLE */
/*******************/
char TxBuffer[1024];
enum {
	TRACK_NORMAL,      // WW
	TRACK_TURN_LEFT,   // BW
	TRACK_TURN_RIGHT,  // WB
	VALLEY_TURN_LEFT,  // WW, Distance too large
	VALLEY_TURN_RIGHT, // WW, Distance too small
	COMPASS_NORMAL,
	COMPASS_TURN_LEFT,
	COMPASS_TURN_RIGHT,

	DUMMY
} TrackStatus = TRACK_NORMAL;
short IPEncoderRef = 0, MotorEncoderRef = 0;
short IPEncoder, MotorEncoder;
short IRValue[5];
unsigned short SonarStatus, Distance;
unsigned short Angle;	
unsigned short T = 0,    PrevT = 0, Multiplier = 0;
float          Time = 0, PrevTime = 0, TimeStep;

int main(void) {
	/******************/
	/* INITIALIZATION */
	/******************/
	Pause(10000);
	SetUart0(115200, 2);
	InitialGpioState(PD8, 0, 2);
	InitialGpioState(PD9, 0, 2);
	
	SetTm2Decoder(1, 0, 65535); // IP
	SetTm3Decoder(3, 0, 65535); // WHEEL
	SetTm4Counter(0, 0, 7199, 10000);
	
	SetAdc(PA0);		
	SetAdc(PA1);		
	SetAdc(PA2);		
	SetAdc(PA3);		
	SetAdc(PA4);		
	
	mySonar.SetFloorLevel(0);
	
	myLCD.BacklightOn(0);
	myLCD.SetBacklight(255);

	while(true) {
		Transmit("Header");
		
		/******************/
		/* TIME ACQUIRING */
		/******************/
		PrevT = T;
		GetTm4CounterValue(T);
		if(T < PrevT)
			Multiplier++;
		PrevTime = Time;
		Time = Multiplier + float(T) / 10000;
		TimeStep = Time - PrevTime;
		
		/***********/
		/* CONTROL */
		/***********/
		GetTm2DecoderValue(IPEncoder);
		GetTm3DecoderValue(MotorEncoder);
	#ifdef TRACK_FLAG
		for(int i = 0; i < 5; i++)
			IRValue[i] = GetAdc(i);
		mySonar.Ranging();
		SonarStatus = mySonar.GetDistance(1, Distance);
		myCompass.GetAngle(Angle);
		
		TrackControl();
	#endif
		PIDControl();
		
		/*************/
		/* SEND DATA */
		/*************/
		// Transmit(int(Time*10000), false);
		// Transmit(int(IPEncoder), false);
		// Transmit(int(MotorEncoder), true);
		
		/*******************/
		/* CATCH DEVIATION */
		/*******************/
		if( ((IPEncoder > 200) || (IPEncoder < -200)) )
			break;
		Pause(50);
	}
	StopMotor();
	return 0;
}

#define BLACK          900
#define CONST_SPEED    850 // in mark/s
#define TIME_OUT       0.3 // in second 

#define BACKWARD_RATIO 0.2
#define TURN_RATIO     (-1)
#define VALLEY_RATIO   0.5
#define COMPASS_RATIO  0   // TODO

short isLeftWhite, isRightWhite;
short isInSplit = 0, splitCount = 0;
float lastSplitTime = 0;

void TrackControl() {
	isLeftWhite  = (IRValue[0] > BLACK);
	isRightWhite = (IRValue[4] > BLACK);

	if(~isLeftWhite && ~isRightWhite) {
		isInSplit = 1;
		switch(splitCount) {
			case 0:
				MotorEncoderRef += TimeStep * CONST_SPEED;
				TrackStatus = TRACK_NORMAL;
				break;
			case 1:
				MotorEncoderRef += TimeStep * CONST_SPEED;
				TrackStatus = TRACK_TURN_LEFT;
				break;
			case 2:
				MotorEncoderRef += TimeStep * CONST_SPEED;
				TrackStatus = TRACK_NORMAL;
				break;
			default:
				// TODO
				// You should write compass rules here
				break;
		}
		lastSplitTime = Time;
	}
	else if(Time - lastSplitTime > TIME_OUT) {
		isInSplit = 0;
		splitCount++;
	}

	if(isLeftWhite && isRightWhite && !isInSplit) {
		if(Time > 40) {
			MotorEncoderRef -= TimeStep * BACKWARD_RATIO * CONST_SPEED;
			TrackStatus = TRACK_NORMAL;
		}
		else if( SonarStatus && (Distance < 15) ) {
			if(Distance < 10) {
				MotorEncoderRef += TimeStep * VALLEY_RATIO * CONST_SPEED;
				TrackStatus = VALLEY_TURN_RIGHT;
			}
			if(Distance > 12) {
				MotorEncoderRef += TimeStep * CONST_SPEED;
				TrackStatus = VALLEY_TURN_LEFT;
			}
		}
		else {
			MotorEncoderRef += TimeStep * CONST_SPEED;
			TrackStatus = TRACK_NORMAL;
		}	
	}
	
	if(isLeftWhite && ~isRightWhite && !isInSplit) {
		MotorEncoderRef += TimeStep * TURN_RATIO * CONST_SPEED;
		TrackStatus = TRACK_TURN_RIGHT;
	}

	if(~isLeftWhite && isRightWhite && !isInSplit) {
		MotorEncoderRef += TimeStep * CONST_SPEED;
		TrackStatus = TRACK_TURN_LEFT;
	}	
}

#define ENCODER_RANGE 444
#define SCALE_T 0.85
#define KP_T    2.2
#define KI_T    3.6
#define KD_T    0.0055
#define SCALE_X 0.8
#define KP_X    0.25
#define KI_X    0.045
#define KD_X    0.02

short init = 0;
float IPEncoderErrorLast, IPEncoderErrorDiff, IPEncoderErrorInt = 0;
float MotorEncoderErrorLast, MotorEncoderErrorDiff, MotorEncoderErrorInt = 0;
float IPEncoderError, MotorEncoderError;
float ControlT, ControlX, ControlValue;

void PIDControl() {
	if(~init) {
		MotorEncoderRef = MotorEncoder;
		init = 1;
	}

	IPEncoderError = IPEncoderRef - IPEncoder;
	if((IPEncoderError < 3) && (-3 < IPEncoderError))
		IPEncoderError = 0;
	IPEncoderErrorDiff = (IPEncoderError - IPEncoderErrorLast) / TimeStep;
	IPEncoderErrorInt += IPEncoderError * TimeStep;
	IPEncoderErrorLast = IPEncoderError;

	MotorEncoderError = MotorEncoderRef - MotorEncoder;
	MotorEncoderErrorDiff = (MotorEncoderError - MotorEncoderErrorLast) / TimeStep;
	MotorEncoderErrorInt += MotorEncoderError * TimeStep;
	MotorEncoderErrorLast = MotorEncoderError;
	
	ControlT = KP_T * IPEncoderError + KI_T * IPEncoderErrorInt + KD_T * IPEncoderErrorDiff;
	ControlX = KP_X * MotorEncoderError + KI_X * MotorEncoderErrorInt + KD_X * MotorEncoderErrorDiff;
	ControlValue = SCALE_T * ControlT + SCALE_X * ControlX;
	
	SetMotor(-short(ControlValue));
}

#define MAP_OFFSET 15
#define MAX_SPEED  168

void SetMotor(short ControlValue) {
	short ControlValueAbs, ControlValueSign;
	ControlValueAbs = (ControlValueSign = ControlValue > 0)? ControlValue:-ControlValue;
	
	/* INVERSE SPEED MAPPING */
	if(ControlValueAbs != 0)
		ControlValueAbs += MAP_OFFSET;
		
	/* MAX SPEED CONTROL */
	if(ControlValueAbs > MAX_SPEED)
		ControlValueAbs = MAX_SPEED;

	/* LEFT-RIGHT REGULATION */
	float Ratio;
	short ControlValueLeft;
	short ControlValueLeftAbs, ControlValueLeftSign;

	switch(TrackStatus) {
		case TRACK_NORMAL:
			Ratio = 1;
			break;
		case TRACK_TURN_LEFT:
			Ratio = TURN_RATIO;
			break;
		case TRACK_TURN_RIGHT:
			Ratio = 1/TURN_RATIO;
			break;
		case VALLEY_TURN_LEFT:
			Ratio = VALLEY_RATIO;
			break;
		case VALLEY_TURN_RIGHT:
			Ratio = 1/VALLEY_RATIO;
			break;
		case COMPASS_NORMAL:
			Ratio = 1;
			break;
		case COMPASS_TURN_LEFT:
			Ratio = COMPASS_RATIO; 
		case COMPASS_TURN_RIGHT:
			Ratio = 1/COMPASS_RATIO; 
		default:
			break;
	}
	ControlValueLeft = Ratio * (ControlValueSign? ControlValueAbs:-ControlValueAbs);
	ControlValueLeftAbs  = (ControlValueLeftSign = ControlValueLeft > 0)? ControlValueLeft:-ControlValueLeft;

	if(ControlValueSign) {
		myMotorR.Forward(ControlValueAbs);
	}
	else {
		myMotorR.Backward(ControlValueAbs);
	}
	
	if(ControlValueLeftSign) {
		myMotorL.Forward(ControlValueLeftAbs);
	}
	else {
		myMotorL.Backward(ControlValueLeftAbs);
	}
}

void StopMotor() {
	myMotorL.Stop();
	myMotorR.Stop();
}

void Transmit(char* ch) {
	sprintf(TxBuffer, "%s", ch);
	SendUart0Data(TxBuffer, strlen(TxBuffer));
	while(GetUart0TxState() != 1);
	Pause(1);
}

void Transmit(int num, bool end) {
	if(!end)
		sprintf(TxBuffer, "%d\t", num);
	else
		sprintf(TxBuffer, "%d\r\n", num);
	SendUart0Data(TxBuffer, strlen(TxBuffer));
	while(GetUart0TxState() != 1);
	Pause(1);
}
