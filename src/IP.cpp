#include "arminno.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define ENCODER_RANGE 444

MotorRunnerC myMotorL(31);
MotorRunnerC myMotorR(30);
LCD2X16A myLCD(3);

char TxBuffer[1024];

void SwingControl(short IPEncoder, float Time, short& State);
void PIDControl(short IPEncoder, short MotorEncoder, float TimeStep);

void SetMotor(short ControlValue);
void StopMotor();
void Transmit(char* ch);
void Transmit(int num, bool end);

int main(void) {
	/********/
	/* INIT */
	/********/
	Pause(10000);
	SetUart0(115200, 2);
	InitialGpioState(PD8, 0, 2);
	InitialGpioState(PD9, 0, 2);
	
	myLCD.BacklightOn(0);
	myLCD.SetBacklight(255);
	
	SetTm2Decoder(1, 0, 65535); // IP
	SetTm3Decoder(3, 0, 65535); // WHEEL
	SetTm4Counter(0, 0, 7199, 10000);
	
	unsigned short T = 0,    PrevT = 0, Multiplier = 0;
	float          Time = 0, PrevTime = 0;
	short IPEncoder, MotorEncoder, ControllerState = 1;
	
	while(true) {
		Transmit("0");
		
		/********/
		/* TIME */
		/********/
		PrevT = T;
		GetTm4CounterValue(T);
		if(T < PrevT)
			Multiplier += 1;	
		PrevTime = Time;
		Time = Multiplier + float(T) / 10000;
		
		/*********/
		/* BLOCK */
		/*********/
		GetTm2DecoderValue(IPEncoder);
		GetTm3DecoderValue(MotorEncoder);	
		if(ControllerState == 0)
			SwingControl(IPEncoder, Time, ControllerState);
		else
			PIDControl(IPEncoder, MotorEncoder, Time - PrevTime);
		
		/********/
		/* SEND */
		/********/
		Transmit(int(Time*10000), false);
		Transmit(int(IPEncoder), false);
		Transmit(int(MotorEncoder), true);
		
		/*********/
		/* CATCH */
		/*********/
		short IPEncoderError = IPEncoder - ENCODER_RANGE / 2;
		if( (ControllerState == 1) && ((IPEncoder > 200) || (IPEncoder < -200)) )
			break;
		Pause(50);
	}
	StopMotor();	
	return 0;
}

void SwingControl(short IPEncoder, float Time, short& State) {
	if(Time < 0.5) {
		SetMotor(152);
		State = 0;
	}
	else if(IPEncoder < 180) {
		SetMotor(0);
		State = 0;
	}
	else {
		SetMotor(0);
		State = 1;
	}
}

void PIDControl(short IPEncoder, short MotorEncoder, float TimeStep) {
	static short init = 0;

	//static short IPEncoderRef = ENCODER_RANGE / 2, MotorEncoderRef = 0;
	static short IPEncoderRef = 0, MotorEncoderRef = 0;
	static float ScaleT = 0.85, KPT = 2.2,  KIT = 3.6,   KDT = 0.0055, \
               ScaleX = 0.8, KPX = 0.25, KIX = 0.045, KDX = 0.02;
	
	static float IPEncoderErrorLast, IPEncoderErrorDiff, IPEncoderErrorInt = 0;
	static float MotorEncoderErrorLast, MotorEncoderErrorDiff, MotorEncoderErrorInt = 0;
	static float IPEncoderError, MotorEncoderError;
	static float ControlT, ControlX, ControlValue;

	if(init == 0) {
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
	
	ControlT = KPT * IPEncoderError + KIT * IPEncoderErrorInt + KDT * IPEncoderErrorDiff;
	ControlX = KPX * MotorEncoderError + KIX * MotorEncoderErrorInt + KDX * MotorEncoderErrorDiff;
	ControlValue = ScaleT * ControlT + ScaleX * ControlX;
	
	//printf("PID\n");
	//printf("IP: %d %f %f %f\n", IPEncoder, IPEncoderError, IPEncoderErrorDiff, IPEncoderErrorInt);
	//printf("Motor: %d %f %f %f\n", MotorEncoder, MotorEncoderError, MotorEncoderErrorDiff, MotorEncoderErrorInt);
	SetMotor(-short(ControlValue));
}

#define MAP_OFFSET 15
#define MAP_THRESH 5
#define MAX_SPEED 168
void SetMotor(short ControlValue) {
	Transmit(int(ControlValue), false);	
	
	short ControlValueAbs, ControlValueSign;
	ControlValueAbs  = (ControlValueSign = ControlValue > 0)? ControlValue:-ControlValue;
	
	/* INVERSE SPEED MAPPING */
	if(ControlValueAbs == 0)
		ControlValueAbs = 0;
	if(ControlValueAbs < MAP_THRESH)
		ControlValueAbs = MAP_OFFSET;
	else
		ControlValueAbs += MAP_OFFSET - MAP_THRESH;
		
	/* MAX SPEED CONTROL */
	if(ControlValueAbs > MAX_SPEED)
		ControlValueAbs = MAX_SPEED;

	if(ControlValueSign) {
		myMotorL.Forward(ControlValueAbs);
		myMotorR.Forward(ControlValueAbs);
	}
	else {
		myMotorL.Backward(-ControlValueAbs);
		myMotorR.Backward(-ControlValueAbs);
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
