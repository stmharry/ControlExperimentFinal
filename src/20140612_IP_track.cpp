#include "arminno.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#define TRACK_FLAG

/*************/
/* COMPONENT */
/*************/
LCD2X16A     myLCD(3);
CompassB     myCompass(5);
SonarA       mySonar(7);
MotorRunnerC myMotorL(31);
MotorRunnerC myMotorR(30);

/*******************/
/* PUBLIC VARIABLE */
/*******************/
#define IP_ENCODER_RANGE 444
char TxBuffer[1024];
enum {
	MOTOR_NORMAL,
	MOTOR_TURN_LEFT,
	MOTOR_TURN_RIGHT,

	MOTOR_DUMMY
} MotorStatus = MOTOR_NORMAL;
short IPEncoderRef = IP_ENCODER_RANGE/2 , MotorEncoderRef = 0;
short IPEncoder, MotorEncoder;
short IRValue[5];
unsigned short SonarStatus, Distance;
unsigned short Angle;
unsigned short T = 0,    PrevT = 0, Multiplier = 0;
float          Time = 0, PrevTime = 0, TimeStep;

/************/
/* FUNCTION */
/************/
void SwingControl();
#define SWING_RUN_TIME  0.3
#define SWING_STAB_TIME 1.0
#define SWING_POS_SPD   255
#define SWING_NEG_SPD   -64
#define SWING_MARGIN    60

void TrackControl();
#define TRACK_BLACK     600
#define TRACK_SPD       100
#define TRACK_TIME_OUT  3
enum {
	TRACK_SWING_UP,
	TRACK_SWING_STAB,
	STRAIGHT_MODE,
	TURN_LEFT_MODE,
	TURN_RIGHT_MODE,
	SPLITTING_MODE,

	TRACK_DUMMY
} TrackStatus = TRACK_SWING_UP;
short isLeftWhite, isRightWhite;
short splitCount = 0;
float lastSplitTime = 0;

void PIDControl();
#define SCALE_T 0.95
#define KP_T    2.2
#define KI_T    3.6
#define KD_T    0.007
#define SCALE_X 0.75
#define KP_X    0.35
#define KI_X    0.06
#define KD_X    0.02
short init = 0;
float IPEncoderErrorLast    = 0, \
      IPEncoderErrorDiff    = 0, \
      IPEncoderErrorInt     = 0, \
      MotorEncoderErrorLast = 0, \
      MotorEncoderErrorDiff = 0, \
      MotorEncoderErrorInt  = 0;
float IPEncoderError, MotorEncoderError;
float ControlT, ControlX, ControlValue;

void SetMotor(short ControlValue);
#define MOTOR_MAP_OFFSET 15
#define MOTOR_MAX_SPEED  168
#define TURN_POS   2
#define TURN_NEG   0.8

void StopMotor();


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
		#endif

		if(TrackStatus == TRACK_SWING_UP)
			SwingControl();
		else {
			#ifdef TRACK_FLAG
			TrackControl();
			#endif
			PIDControl();
		}

		/*******************/
		/* CATCH DEVIATION */
		/*******************/
		if( (MotorStatus != TRACK_SWING_UP) && ((IPEncoder > 400) || (IPEncoder < 40)) )
			break;

	}
	StopMotor();
	return 0;
}

void SwingControl() {
	if(Time < SWING_RUN_TIME)
		SetMotor(SWING_POS_SPD);
	else if(IPEncoder > IPEncoderRef - SWING_MARGIN)
		TrackStatus = TRACK_SWING_STAB;
	else
		SetMotor(SWING_NEG_SPD);
}

void TrackControl0() {
	MotorEncoderRef += TimeStep * TRACK_SPD;
	MotorStatus = MOTOR_TURN_LEFT;
}

void TrackControl() {
	isLeftWhite  = (IRValue[4] > TRACK_BLACK);
	isRightWhite = (IRValue[0] > TRACK_BLACK);

	switch(TrackStatus) {
		case SPLITTING_MODE:
			switch(splitCount) {
			case 0:
				MotorStatus = MOTOR_NORMAL;
				break;
			case 1:
				MotorStatus = MOTOR_NORMAL;
				break;
			case 2:
				MotorStatus = MOTOR_TURN_LEFT;
				break;
			default:
				// TODO
				// You should write compass rules here
				break;
			}
			break;
		case STRAIGHT_MODE:
			MotorStatus = MOTOR_NORMAL;
			break;
		case TURN_LEFT_MODE:
			MotorStatus = MOTOR_TURN_LEFT;
			break;
		case TURN_RIGHT_MODE:
			MotorStatus = MOTOR_TURN_RIGHT;
			break;
	}
	MotorEncoderRef += TimeStep * TRACK_SPD;

	switch(TrackStatus) {
		case SPLITTING_MODE:
			if(Time - lastSplitTime < TRACK_TIME_OUT) {
				if(!isLeftWhite && !isRightWhite)
					lastSplitTime = Time;
				TrackStatus = SPLITTING_MODE;
				break;
			}
			else
				splitCount++;
		case STRAIGHT_MODE:
		case TURN_LEFT_MODE:
		case TURN_RIGHT_MODE:
			if(!isLeftWhite && !isRightWhite) {
				TrackStatus = SPLITTING_MODE;
				lastSplitTime = Time;
			}
			else if(isLeftWhite && isRightWhite)
				TrackStatus = STRAIGHT_MODE;
			else if(!isLeftWhite && isRightWhite)
				TrackStatus = TURN_LEFT_MODE;
			else if(isLeftWhite && !isRightWhite && (splitCount >= 3))
				TrackStatus = TURN_RIGHT_MODE;
	}
}

void PIDControl() {
	if( (TrackStatus == TRACK_SWING_STAB) && (Time > SWING_STAB_TIME) )
		TrackStatus = STRAIGHT_MODE;
	if( (TrackStatus != TRACK_SWING_STAB) && (init == 0) ) {
		MotorEncoderRef = MotorEncoder;
		MotorEncoderErrorInt = 0;
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

	if(TrackStatus == TRACK_SWING_STAB)
		ControlValue = SCALE_T * ControlT;
	else
		ControlValue = SCALE_T * ControlT + SCALE_X * ControlX;

	SetMotor(-short(ControlValue));
}

void SetMotor(short ControlValue) {
	short ControlValueAbs, ControlValueSign;
	ControlValueAbs = (ControlValueSign = ControlValue > 0)? ControlValue:-ControlValue;

	/* INVERSE SPEED MAPPING */
	if(ControlValueAbs != 0)
		ControlValueAbs += MOTOR_MAP_OFFSET;

	/* MAX SPEED CONTROL */
	if(ControlValueAbs > MOTOR_MAX_SPEED)
		ControlValueAbs = MOTOR_MAX_SPEED;

	ControlValue = ControlValueSign? ControlValueAbs:-ControlValueAbs;

	/* LEFT-RIGHT REGULATION */
	float Ratio;
	short ControlValueLeft, ControlValueRight;
	short ControlValueLeftAbs, ControlValueRightAbs, \
	      ControlValueLeftSign, ControlValueRightSign;

	switch(MotorStatus) {
		case MOTOR_NORMAL:
			ControlValueLeft = ControlValueRight = ControlValue;
			break;
		case MOTOR_TURN_LEFT:
			if(ControlValueSign) {
				ControlValueLeft  = -TURN_NEG*ControlValueAbs;
				ControlValueRight = TURN_POS*ControlValueAbs;
			}
			else {
				ControlValueLeft  = -TURN_POS*ControlValueAbs;
				ControlValueRight = TURN_NEG*ControlValueAbs;
			}
			break;
		case MOTOR_TURN_RIGHT:
			if(ControlValueSign) {
				ControlValueLeft  = TURN_POS*ControlValueAbs;
				ControlValueRight = -TURN_NEG*ControlValueAbs;
			}
			else {
				ControlValueLeft  = TURN_NEG*ControlValueAbs;
				ControlValueRight = -TURN_POS*ControlValueAbs;
			}
			break;
		default:
			break;
	}
	ControlValueLeftAbs  = (ControlValueLeftSign  = ControlValueLeft  > 0)? ControlValueLeft: -ControlValueLeft;
	ControlValueRightAbs = (ControlValueRightSign = ControlValueRight > 0)? ControlValueRight:-ControlValueRight;

	/* HARDWARE FIX */
	ControlValueLeftAbs  = 0.85 * ControlValueLeftAbs;
	ControlValueRightAbs = 1.1 * ControlValueRightAbs;

	if(ControlValueLeftSign)
		myMotorL.Forward(ControlValueLeftAbs);
	else
		myMotorL.Backward(ControlValueLeftAbs);

	if(ControlValueRightSign)
		myMotorR.Forward(ControlValueRightAbs);
	else
		myMotorR.Backward(ControlValueRightAbs);
}

void StopMotor() {
	myMotorL.Stop();
	myMotorR.Stop();
}
