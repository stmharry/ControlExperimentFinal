#include "arminno.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define  WORKING_MODE 3
/****************************************************/
/* 2: WORKING_IP                                    */
/* 3: WORKING_SWING + WORKING_IP                    */
/* 4: WORKING_TRACK                                 */
/* 6: WORKING_IP    + WORKING_TRACK                 */
/* 7: WORKING_SWING + WORKING_IP    + WORKING_TRACK */
/****************************************************/

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
enum {
	WORKING_SWING = 1,
	WORKING_IP    = 2,
	WORKING_TRACK = 4,

	WORKING_DUMMY
} WorkingMode;
enum {
	MOTOR_NORMAL,
	MOTOR_TURN_LEFT,
	MOTOR_TURN_RIGHT,

	MOTOR_DUMMY
} MotorStatus;
char TxBuffer[1024];
short IPEncoderRef,    \
      IPEncoder,       \
      MotorEncoderRef, \
	  MotorEncoder;
short IRValue[5];
unsigned short SonarStatus, Distance;
unsigned short Angle;
unsigned short T,     \
               PrevT, \
               Multiplier;
float Time,     \
      PrevTime, \
      TimeStep;

/************/
/* FUNCTION */
/************/
// UTILITY
void Init();
void Acquire();
#define CHECK_RANGE 180
bool CheckFail();

// CONTROL
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
	TRACK_STRAIGHT,
	TRACK_TURN_LEFT,
	TRACK_TURN_RIGHT,
	TRACK_SPLIT,

	TRACK_DUMMY
} TrackStatus;

void PIDControl();
#define SCALE_T 0.95
#define KP_T    2.2
#define KI_T    3.6
#define KD_T    0.007
#define SCALE_X 0.75
#define KP_X    0.35
#define KI_X    0.06
#define KD_X    0.02
bool init;
float IPEncoderError,        \
	  IPEncoderErrorLast,    \
      IPEncoderErrorDiff,    \
      IPEncoderErrorInt,     \
	  MotorEncoderError,     \
      MotorEncoderErrorLast, \
      MotorEncoderErrorDiff, \
      MotorEncoderErrorInt;

void SimpleControl();

// MOTOR
void SetMotor(short ControlValue);
#define MOTOR_MAP_OFFSET 15
#define MOTOR_MAX_SPEED  168
#define MOTOR_TURN_POS   2
#define MOTOR_TURN_NEG   0.8
#define MOTOR_LEFT_FIX   0.85
#define MOTOR_RIGHT_FIX  1.1
void StopMotor();

/********/
/* MAIN */
/********/
int main() {
	Init();
	while(!CheckFail()) {
		Acquire();
		if(TrackStatus == TRACK_SWING_UP)
			SwingControl(); // WORKING_MODE = 3 / 7
		else {
			if(WORKING_MODE | WORKING_TRACK)
				TrackControl(); // WORKING_MODE = 4 / 6 / 7
			if(WORKING_MODE | WORKING_IP)
				PIDControl(); // WORKING_MODE = 2 / 3 / 6
			else
				SimpleControl(); // WORKING_MODE = 4
		}
	}
	StopMotor();
	return 0;
}

void Init() {
	// SENSOR INIT
	Pause(10000);
	SetUart0(115200, 2);
	InitialGpioState(PD8, 0, 2);
	InitialGpioState(PD9, 0, 2);

	SetTm2Decoder(1, 0, 65535); // IP
	SetTm3Decoder(3, 0, 65535); // WHEEL
	SetTm4Counter(0, 0, 7199, 10000);

	myLCD.BacklightOn(0);
	myLCD.SetBacklight(255);

	if(WORKING_MODE | WORKING_TRACK) {
		SetAdc(PA0);
		SetAdc(PA1);
		SetAdc(PA2);
		SetAdc(PA3);
		SetAdc(PA4);

		mySonar.SetFloorLevel(0);
	}
	// VARIABLE INIT
	if(WORKING_MODE | WORKING_SWING) {
		TrackStatus = TRACK_SWING_UP;
		IPEncoderRef = IP_ENCODER_RANGE / 2;
	}
	else {
		TrackStatus = TRACK_STRAIGHT;
		IPEncoderRef = 0;
	}
	MotorStatus = MOTOR_NORMAL;
	MotorEncoderRef = 0;

	T = PrevT = Multiplier = 0;
	Time = PrevTime = 0;

	init = false;
	IPEncoderError = IPEncoderErrorLast = IPEncoderErrorDiff = IPEncoderErrorInt = 0;
	MotorEncoderError = MotorEncoderErrorLast = MotorEncoderErrorDiff = MotorEncoderErrorInt = 0;
}

void Acquire() {
	// TIME
	PrevT = T;
	GetTm4CounterValue(T);
	if(T < PrevT)
		Multiplier++;
	PrevTime = Time;
	Time = Multiplier + float(T) / 10000;
	TimeStep = Time - PrevTime;

	// SENSOR
	GetTm2DecoderValue(IPEncoder);
	GetTm3DecoderValue(MotorEncoder);

	if(WORKING_MODE | WORKING_TRACK) {
		for(int i = 0; i < 5; i++)
			IRValue[i] = GetAdc(i);
		myCompass.GetAngle(Angle);
		mySonar.Ranging();
		SonarStatus = mySonar.GetDistance(1, Distance);
	}
}

bool CheckFail() {
	return ( !(WORKING_MODE | WORKING_SWING) && \
	         ((IPEncoderError > CHECK_RANGE) || (IPEncoderError < -CHECK_RANGE)) ) ||
		   ( (WORKING_MODE | WORKING_SWING)  && (MotorStatus != TRACK_SWING_UP) && \
	         ((IPEncoderError > CHECK_RANGE) || (IPEncoderError < -CHECK_RANGE)) );
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
	// TODO
	short isLeftWhite, isRightWhite;
	short splitCount = 0;
	float lastSplitTime = 0;

	isLeftWhite  = (IRValue[4] > TRACK_BLACK);
	isRightWhite = (IRValue[0] > TRACK_BLACK);

	switch(TrackStatus) {
		case TRACK_SPLIT:
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
			// TODO
			// You should write compass rules here
			default:
				break;
			}
			break;
		case TRACK_STRAIGHT:
			MotorStatus = MOTOR_NORMAL;
			break;
		case TRACK_TURN_LEFT:
			MotorStatus = MOTOR_TURN_LEFT;
			break;
		case TRACK_TURN_RIGHT:
			MotorStatus = MOTOR_TURN_RIGHT;
			break;
	}
	MotorEncoderRef += TimeStep * TRACK_SPD;

	switch(TrackStatus) {
		case TRACK_SPLIT:
			if(Time - lastSplitTime < TRACK_TIME_OUT) {
				if(!isLeftWhite && !isRightWhite)
					lastSplitTime = Time;
				TrackStatus = TRACK_SPLIT;
				break;
			}
			else
				splitCount++;
		case TRACK_STRAIGHT:
		case TRACK_TURN_LEFT:
		case TRACK_TURN_RIGHT:
			if(!isLeftWhite && !isRightWhite) {
				TrackStatus = TRACK_SPLIT;
				lastSplitTime = Time;
			}
			else if(isLeftWhite && isRightWhite)
				TrackStatus = TRACK_STRAIGHT;
			else if(!isLeftWhite && isRightWhite)
				TrackStatus = TRACK_TURN_LEFT;
			else if(isLeftWhite && !isRightWhite && (splitCount >= 3))
				TrackStatus = TRACK_TURN_RIGHT;
	}
}

void PIDControl() {
	float ControlT, ControlX, ControlValue;

	// GO TO PID MODE
	if( (TrackStatus == TRACK_SWING_STAB) && (Time > SWING_STAB_TIME) )
		TrackStatus = TRACK_STRAIGHT;

	// INIT PID MODE
	if( (TrackStatus != TRACK_SWING_STAB) && !init ) {
		MotorEncoderRef = MotorEncoder;
		MotorEncoderErrorInt = 0;
		init = true;
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

void SimpleControl() {
	// TODO
}

void SetMotor(short ControlValue) {
	short ControlValueAbs, ControlValueSign;
	ControlValueAbs = (ControlValueSign = ControlValue > 0)? ControlValue:-ControlValue;

	// INVERSE SPEED MAPPING
	if(ControlValueAbs != 0)
		ControlValueAbs += MOTOR_MAP_OFFSET;

	// MAX SPEED CONTROL
	if(ControlValueAbs > MOTOR_MAX_SPEED)
		ControlValueAbs = MOTOR_MAX_SPEED;

	ControlValue = ControlValueSign? ControlValueAbs:-ControlValueAbs;

	// LEFT-RIGHT REGULATION
	float RatioLeft, RatioRight;
	short ControlValueLeft,     \
		  ControlValueLeftAbs,  \
	      ControlValueLeftSign, \
		  ControlValueRight,    \
		  ControlValueRightAbs, \
		  ControlValueRightSign;

	switch(MotorStatus) {
		case MOTOR_NORMAL:
			ControlValueLeft = ControlValueRight = ControlValue;
			break;
		case MOTOR_TURN_LEFT:
			if(ControlValueSign) {
				RatioLeft  = - MOTOR_TURN_NEG;
				RatioRight = + MOTOR_TURN_POS;
			}
			else {
				RatioLeft  = - MOTOR_TURN_POS;
				RatioRight = + MOTOR_TURN_NEG;
			}
			break;
		case MOTOR_TURN_RIGHT:
			if(ControlValueSign) {
				RatioLeft  = + MOTOR_TURN_POS;
				RatioRight = - MOTOR_TURN_NEG;
			}
			else {
				RatioLeft  = + MOTOR_TURN_NEG;
				RatioRight = - MOTOR_TURN_POS;
			}
			break;
		default:
			break;
	}

	ControlValueLeft  = RatioLeft  * ControlValueAbs;
	ControlValueRight = RatioRight * ControlValueAbs;
	ControlValueLeftAbs  = (ControlValueLeftSign  = ControlValueLeft  > 0)? ControlValueLeft: -ControlValueLeft;
	ControlValueRightAbs = (ControlValueRightSign = ControlValueRight > 0)? ControlValueRight:-ControlValueRight;

	// HARDWARE FIX
	ControlValueLeftAbs  = MOTOR_LEFT_FIX  * ControlValueLeftAbs;
	ControlValueRightAbs = MOTOR_RIGHT_FIX * ControlValueRightAbs;

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
