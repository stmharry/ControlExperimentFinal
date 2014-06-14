#include "arminno.h"
#include <vector>
#include <string.h>

#define BLACK 950
#define ConstantSpeed 50
#define DistanceToWallStandard 25
#define valleyDifferenceTolerance 5
#define COMMAND_TTL  2000

#define BackRatio -1
#define ValleyRatio 0.3
#define LIB_SIZE 16
#define BLACK_SUM 5

unsigned short Value[5];
int i;
MotorRunnerC myMotorL(31);
MotorRunnerC myMotorR(30);
SonarA mySonarL(7);
CompassB myComp(5);
unsigned short Angle;
LCD2X16A myLCD(3);
unsigned short DistanceL;
unsigned short StatusL;

int black_count;

void MotorControl(int left ,int right)
{
	if( left == right )
	{
		if (left>0)
		{
			myMotorR.Forward(ConstantSpeed);
			myMotorL.Forward(ConstantSpeed);
		}
		else if (left<0)
		{
			myMotorR.Backward(ConstantSpeed);
			myMotorL.Backward(ConstantSpeed);
		}
		else
		{
			myMotorL.Stop();
			myMotorR.Stop();
		}


	}
	else if( left > right )
	{
		myMotorR.Backward(ConstantSpeed*1.6);
		myMotorL.Forward(ConstantSpeed*1.6);
	}
	else
	{
		myMotorR.Forward(ConstantSpeed*1.6);
		myMotorL.Backward(ConstantSpeed*1.6);
	}



}
enum ControlStyle
{
	track_tracing,
	valleyNav,
	blindMode,
	followCompassStyle,
	none
};
enum SteeringCommand
{
	GoLeft,
	GoRight,
	GoStraight,
	FollowCompass,
	Goal,
	neutral
};

enum compassStage
{
	first_charge,
	turning,
	second_charge,
	final_charge
};

void trackTracing(short ,short ,int&,int&,SteeringCommand *commandLib,int &whi,SteeringCommand &myS,int &cu,int &c,int &,ControlStyle &,float,float&);
void ValleyNavigation(unsigned short,short,int&,int&);
void compassNavigation(short,int&,int&,short ,short ,short ,short ,short ,compassStage &,ControlStyle &);
void stop(int &a,int& b){a=0;b=0;}


void TimeBasedCommand(float , SteeringCommand &);
void BlindMode(int &a,int& b)
{
	a=ConstantSpeed;
	b=ConstantSpeed;
	myLCD.Clear();
	myLCD.Display("Blind Mode");
}
void commandSurvival(SteeringCommand,float,float&);

int main(void)  {

	SetAdc(PA0);
	SetAdc(PA1);
	SetAdc(PA2);
	SetAdc(PA3);
	SetAdc(PA4);
	mySonarL.SetFloorLevel(0);

	myLCD.BacklightOn(0);
	myLCD.SetBacklight (255);


	Pause(1);
	int LeftSpeed  = ConstantSpeed;
	int RightSpeed = ConstantSpeed;


	ControlStyle myStyle=none;
	SteeringCommand mySteeringCommand=neutral;
	int currentStep=0;
	int commandExpiration=0;
	SteeringCommand commandLib[LIB_SIZE+1];

	commandLib[0]=GoStraight;
	commandLib[1]=GoLeft;
	commandLib[2]=GoLeft;
	commandLib[3]=GoLeft;
	commandLib[4]=GoRight;
	commandLib[5]=GoLeft;
	commandLib[6]=GoRight;
	commandLib[7]=GoLeft;
	commandLib[8]=GoRight;
	/////////////////////
	//
	/////////////////////
	commandLib[9]=GoLeft;
	commandLib[10]=GoLeft;
	commandLib[11]=GoLeft;
	commandLib[12]=GoStraight;
	//////////////////////
	//valley
	//////////////////////
	commandLib[13]=GoStraight;
	commandLib[14]=GoLeft;
	commandLib[15]=FollowCompass;
	commandLib[16]=Goal;
	int whichOrderToTake=1;

	//black_count=0;

	compassStage myStage=first_charge;
	/////////////////////////
	unsigned short 	T = 0, \
			    PrevT = 0, \
			    Multiplier = 0;
	float Time = 0, \
		     PrevTime = 0;
	SetTm4Counter(0, 0, 7199, 10000);
	//////////////////


	myStyle = blindMode;
	float  commandExpTime=0;
	////////////////////
	//initial charge
	///////////////////
	if (whichOrderToTake<=1)
	{
		myLCD.Display("initial charge");
		myMotorR.Forward(ConstantSpeed);
		myMotorL.Forward(ConstantSpeed);
		Pause(10000);
	}
	/////////////////////////////
	while(1)
	{
		currentStep++;
		mySonarL.Ranging();
		myComp.GetAngle(Angle);
		Angle = Angle % 360;

		for(i = 0 ; i < 5 ; i++)
		{
			Value[i] = GetAdc(i);
		}

		if( (Value[0] < BLACK) && (Value[4] < BLACK) )
			black_count++;

		StatusL = mySonarL.GetDistance(1, DistanceL);


		////////////////////////////////////////////////
		GetTm4CounterValue(T);
		if(T < PrevT)
			Multiplier = Multiplier + 1;
		PrevT = T;
		Time = Multiplier + float(T) / 10000;
		///////////////////////////////////////////////

		if (myStyle!=followCompassStyle)
		{
			if( (Value[2]<BLACK) && ( (Value[1]<BLACK) || (Value[3]<BLACK) ) )
				myStyle=track_tracing;
			else if((StatusL==1)&&(DistanceL<DistanceToWallStandard))
				myStyle=valleyNav;
			else
				myStyle=blindMode;
		}

		int pauseTime=1;
		switch(myStyle)
		{
			case track_tracing :
				trackTracing(Value[0],Value[4],LeftSpeed,RightSpeed,commandLib,whichOrderToTake,mySteeringCommand,currentStep,commandExpiration,pauseTime,myStyle,Time,commandExpTime);
				break;

			case valleyNav:
				ValleyNavigation(StatusL,DistanceL,LeftSpeed,RightSpeed);
				pauseTime=1;
				break;

			case followCompassStyle:
				compassNavigation(Angle,LeftSpeed,RightSpeed,Value[0],Value[1],Value[2],Value[3],Value[4],myStage,myStyle);
				break;

			default:
				BlindMode(LeftSpeed,RightSpeed);
				break;
		}

		MotorControl(LeftSpeed,RightSpeed);
		Pause(pauseTime);

	}

}
void trackTracing(short rightSensor,short leftSensor,int&LeftSpeed,int&RightSpeed,SteeringCommand* commandLib,int &whichOrderToTake,SteeringCommand &mySteeringCommand,int &currentStep,int &commandExpiration,int &pauseTime ,ControlStyle &myStyle,float current_time,float & commandExpTime)
{
	myLCD.Clear();
	//    myLCD.Display("black ");
	//		myLCD.Display(black_count);
	myLCD.Display(" Order:");
	myLCD.Display(char(whichOrderToTake-1));
	Pause(50);


	switch(mySteeringCommand)
	{
		case neutral:
			myLCD.Display(" neutral");
			if( (rightSensor < BLACK) && (leftSensor > BLACK) )
			{
				RightSpeed = ConstantSpeed*BackRatio;
				LeftSpeed  = ConstantSpeed;
				return;
			}

			if( (rightSensor > BLACK) && (leftSensor < BLACK) )
			{
				LeftSpeed  = ConstantSpeed*BackRatio;
				RightSpeed = ConstantSpeed;
				return;
			}

			if( (rightSensor > BLACK) && (leftSensor > BLACK) )
			{
				LeftSpeed  = ConstantSpeed;
				RightSpeed = ConstantSpeed;
				return;
			}

			if( (rightSensor < BLACK) && (leftSensor < BLACK) )
			{

				//if (black_count>BLACK_SUM=1)
				if(1)
				{
					if(whichOrderToTake>LIB_SIZE)whichOrderToTake=LIB_SIZE;
					mySteeringCommand	= commandLib[whichOrderToTake];
					//if (mySteeringCommand==FollowCompass)myLCD.Display(" I should follow compass");

					//commandExpiration=currentStep+COMMAND_TTL;
					//commandExpTime=current_time+1;
					commandSurvival(commandLib[whichOrderToTake],current_time,commandExpTime);
					whichOrderToTake++;
				}
				else
				{
					mySteeringCommand=GoStraight;
					commandExpTime=current_time+0.01;
				}
			}
			break;

		case GoLeft :
			myLCD.Display(" GoLeft");
			if( leftSensor < BLACK )
				LeftSpeed = ConstantSpeed*BackRatio;
			else LeftSpeed = ConstantSpeed;
			RightSpeed  = ConstantSpeed;
			break;

		case GoRight :
			myLCD.Display(" GoRight");
			if( rightSensor < BLACK )
				RightSpeed = ConstantSpeed*BackRatio;
			else RightSpeed = ConstantSpeed;
			LeftSpeed   = ConstantSpeed;
			break;

		case GoStraight :
			myLCD.Display(" GoStraight");
			RightSpeed = ConstantSpeed;
			LeftSpeed  = ConstantSpeed;
			break;

		case FollowCompass:
			myLCD.Display(" Follow Compass");
			myStyle = followCompassStyle;
			RightSpeed = 0;
			LeftSpeed  = 0;
			break;

		case Goal:
			RightSpeed = 0;
			LeftSpeed  = 0;

			myMotorR.Forward(ConstantSpeed);
			myMotorL.Forward(ConstantSpeed);
			Pause(20000);

			myLCD.Clear();
			myLCD.Display("Goal");
			Pause(100000);
			return;
			break;

	}

	if( commandExpTime < current_time )
		mySteeringCommand=neutral;

	pauseTime=1;


}
void ValleyNavigation(unsigned short StatusL,short DistanceL,int&LeftSpeed,int&RightSpeed)
{
	//if(Status!=1)return;


	myLCD.Clear();
	myLCD.Display("Valley Mode");
	myLCD.Display("Left: ");
	myLCD.Display(DistanceL);
	//Pause(100);

	if( DistanceL<10 )
	{
		//printf("Invalley , turn right\n");
		LeftSpeed  = ConstantSpeed;
		RightSpeed = ConstantSpeed*ValleyRatio;
		return;
	}
	if( DistanceL>12 )
	{
		//printf("Invalley , turn left\n");
		LeftSpeed  = ConstantSpeed*ValleyRatio;
		RightSpeed = ConstantSpeed;
		return;
	}
	LeftSpeed  = ConstantSpeed;
	RightSpeed = ConstantSpeed;

}

void commandSurvival(SteeringCommand order,float current_time,float &commandExpTime)

{
	switch(order)
	{

		case GoStraight:
			commandExpTime=current_time+0.2;
			break;
		default :
			commandExpTime=current_time+1;
			break;

	}
}
void TimeBasedCommand(float time, SteeringCommand &mySteeringCommand)
{
	/*if(time<=10.5)
	  mySteeringCommand=GoStraight;
	  else if(10.5<time<=15)
	  mySteeringCommand=GoLeft;
	  else if (15<time<=35)
	  mySteeringCommand=GoStraight;*/

}
void compassNavigation(short angle,int &left ,int &right,short val_0 ,short val_1,short val_2 ,short val_3,short val_4,compassStage &myStage,ControlStyle &myStyle)
{
	myLCD.Clear();
	//myLCD.Display("Compass Mode !! Stage:");

#define TARGET_ANGLE 37

	switch(myStage)
	{
		case first_charge:
			myLCD.Display("first_cahrge");
			myMotorR.Forward(ConstantSpeed);
			myMotorL.Forward(ConstantSpeed);
			Pause(2000);
			myStage = turning;
			break;

		case turning:
			myLCD.Clear();
			myLCD.Display("turning");
			myLCD.Display(" Angle: ");
			myLCD.Display(Angle);
			if ( (Angle < TARGET_ANGLE-3 ) || (Angle > TARGET_ANGLE+3 ) )
			{
				left=ConstantSpeed*0.8;
				right=ConstantSpeed*-0.8;
			}
			else
				myStage=second_charge;
			break;

		case second_charge:
			myLCD.Display("second_cahrge");
			myMotorR.Forward(ConstantSpeed);
			myMotorL.Forward(ConstantSpeed);
			Pause(5000);
			myStage = final_charge;
			break;

		case final_charge:
			myLCD.Display("final_cahrge");

			if((val_1<BLACK)&&(val_2<BLACK)&&(val_3<BLACK))
			{
				myMotorR.Forward(ConstantSpeed);
				myMotorL.Forward(ConstantSpeed);
				Pause(30000);

				myLCD.Clear();
				myLCD.Display("Goal");
				myMotorR.Stop();
				myMotorL.Stop();
				Pause(65530);
				Pause(65530);
				Pause(65530);
				Pause(65530);
				return;
			}
			else if (Angle<TARGET_ANGLE-3)
			{
				left=ConstantSpeed*0.8;
				right=ConstantSpeed*-0.8;
			}
			else if(Angle>TARGET_ANGLE+3)
			{
				left=ConstantSpeed*-0.8;
				right=ConstantSpeed*0.8;
			}

			else
			{
				left=ConstantSpeed*0.8;
				right=ConstantSpeed*0.8;
			}
	}
}
