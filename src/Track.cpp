#include "arminno.h"
#include <vector>
#include <string.h>

#define BLACK 900
#define ConstantSpeed 60
#define ValleyRatio 0.5

unsigned short Value[5];
u8 RxFifo;
int i;
MotorRunnerC myMotorL(31);		
MotorRunnerC myMotorR(30);
SonarA mySonarL(7);	
u8 StatusL;
unsigned short DistanceL;
CompassB myComp(5);				
unsigned short Angle;
LCD2X16A myLCD(3);
			
int main(void)  {
	
	SetAdc(PA0);		
	SetAdc(PA1);		
	SetAdc(PA2);		
	SetAdc(PA3);		
	SetAdc(PA4);		
	
	mySonarL.SetFloorLevel(0);
	
	myLCD.BacklightOn(0);
	myLCD.SetBacklight (255);
	
	unsigned short 	T = 0, \
					PrevT = 0, \
                 	Multiplier = 0;
	float Time = 0, \
	      PrevTime = 0;
	SetTm4Counter(0, 0, 7199, 10000);
	
	
	while(1) 
	{
		mySonarL.Ranging();
			
		for(i = 0 ; i < 5 ; i++) 
			Value[i] = GetAdc(i);

		StatusL = mySonarL.GetDistance(1, DistanceL);
		myComp.GetAngle(Angle);

		GetTm4CounterValue(T);
		if(T < PrevT)
			Multiplier = Multiplier + 1;
		PrevT = T;
		Time = Multiplier + float(T) / 10000;

		myLCD.Clear();		
		myLCD.Display("time:");
		myLCD.Display(int(Time));
		
		if( (Value[0] > BLACK) && (Value[4] > BLACK) )	// Left:W  Right:W
		{
			if ( time > 40)									// if there is no track in front of: move back
			{
				myMotorR.Backward(ConstantSpeed*0.2);
				myMotorL.Backward(ConstantSpeed*0.2);
			}
			else if ( (StatusL==1) && (DistanceL<15) )		// if Sonar has a value & in the valley: Valley mode 
			{
				myLCD.Clear();
				myLCD.Display("Valley Mode  distance: ");
				myLCD.Display(DistanceL);
										
			 	if( DistanceL<10 )
				{
					myMotorR.Forward(ConstantSpeed*ValleyRatio);
					myMotorL.Forward(ConstantSpeed);
					return;
				}
				if( DistanceL>12 )
				{
					myMotorR.Forward(ConstantSpeed);
					myMotorL.Forward(ConstantSpeed*ValleyRatio);
					return;
				}						
			}
			else											// right on the track: go straight
			{
				myMotorR.Forward(ConstantSpeed);
				myMotorL.Forward(ConstantSpeed);
			}
		}
		
		else if( (Value[0] < BLACK) && (Value[4] > BLACK) )		// Left:W  Right:B
		{
			myMotorR.Backward(ConstantSpeed);
			myMotorL.Forward(ConstantSpeed);
		}
			
		else if( (Value[0] > BLACK) && (Value[4] < BLACK) )	// Left:B  Right:W
		{
			myMotorR.Forward(ConstantSpeed);
			myMotorL.Backward(ConstantSpeed);
		}
		
		else if( (Value[0] < BLACK) && (Value[4] < BLACK) )	// Right:B Left:B
		{
			switch (time)
			{
				case 0 ... 10 :								// before the third crosspoint: go straight
					myMotorR.Forward(ConstantSpeed);
					myMotorL.Forward(ConstantSpeed);
					pause(500);
					break;
					
				case 11 ... 15 :							// before go into the velly: go left
					myMotorR.Forward(ConstantSpeed);
					myMotorL.Backward(ConstantSpeed);
					pause(500);
					break;
					
				case 16 ... 35 :							// before using compass: go straight
					myMotorR.Forward(ConstantSpeed);
					myMotorL.Forward(ConstantSpeed);
					pause(500);
					break;
					
				default :									// compass mode
					myLCD.Clear();
					myLCD.Display("Compass Mode   Angle: ");
					myLCD.Display(Angle);
					
					while( (Angle>9) || (Angle<7) )
					{
						myMotorR.Backward(ConstantSpeed*0.2);
						myMotorL.Forward(ConstantSpeed*0.2);
					}
					pause(1000);
						
					while( (Value[1]>BLACK) || (Value[2]>BLACK) || (Value[3]>BLACK) )
					{
						myMotorR.Forward(ConstantSpeed*0.5);
						myMotorL.Forward(ConstantSpeed*0.5);
					}
					pause(500);
					myMotorL.stop();
					myMotorR.stop();
					break;
			}
		}
	}
