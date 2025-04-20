#pragma config(Sensor, in1,    ClawArmPotL,    sensorPotentiometer)
#pragma config(Sensor, in2,    ClawArmPotR,    sensorPotentiometer)
#pragma config(Sensor, in3,    ArmPot,         sensorPotentiometer)
#pragma config(Sensor, in4,    MobilePotL,     sensorPotentiometer)
#pragma config(Sensor, in5,    MobilePotR,     sensorPotentiometer)
#pragma config(Sensor, dgtl2,  Switch2,        sensorTouch)
#pragma config(Sensor, dgtl3,  BumpTouch2,     sensorTouch)
#pragma config(Sensor, dgtl5,  BumpTouch1,     sensorTouch)
#pragma config(Sensor, dgtl9,  EncRight,       sensorQuadEncoder)
#pragma config(Sensor, dgtl11, EncLeft,        sensorQuadEncoder)
#pragma config(Motor,  port1,           Left2,        tmotorVex393_HBridge, openLoop, reversed, driveRight)
#pragma config(Motor,  port2,           Left1,         tmotorVex393_MC29, openLoop, driveLeft)
#pragma config(Motor,  port3,           MobileGoalL,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           ClawArmL,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           TR,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           TL,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           ClawArmR,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           MobileGoalR,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           Right1,        tmotorVex393_MC29, openLoop, reversed, driveRight)
#pragma config(Motor,  port10,          Right2,         tmotorVex393_HBridge, openLoop, reversed, driveLeft)

int moptimalL;
int moptimalR

task PID_MG()
{
	float mkp = 0.3; //tune
	float mki = 0.00001; //tune
	float mkd = 0.2; //tune

	float mcurrentL ;
	float merrorTl;
	float mlastErrorL = 0;
	float mproportionL;
	float mintegralL;
	float mderivativeL;
	float mcurrentR ;
	float merrorTr;
	float mlastErrorR = 0;
	float mproportionR;
	float mintegralR;
	float mderivativeR;

	while(true){


		float merrorL = MoptimalL - SensorValue[MobilePotL];
		float merrorR = MoptimalR - SensorValue[MobilePotR];
		//PidAtTargetL = abs(SensorValue[ArmPotL] - optimalL) < 50;
		//PidAtTargetR = abs(SensorValue[ArmPotR] - optimalR) < 50;


		if (abs(merrorL) < 200 && merrorL !=0)
		{
			merrorTl = merrorL;
		}
		else{
			merrorTl = 0;
		}

		if (abs(merrorR) < 200 && merrorR !=0)
		{
			merrorTr = merrorR;
		}
		else{
			merrorTr = 0;
		}

		if (abs(merrorTl)> 50 / mki){
			merrorTl = 50 / mki;
		}

		if (merrorL ==0){
			mderivativeL = 0;
		}

		if (abs(merrorTr)> 50 / mki){
			merrorTr = 50 / mki;
		}

		if (merrorR ==0){
			mderivativeR = 0;
		}




		mproportionL = merrorL * mkp;
		mproportionR = merrorR * mkp;
		mintegralL = merrorTl  * mki;
		mintegralR = merrorTr  * mki;
		mderivativeL = (merrorL - mlastErrorL) * mkd;
		mderivativeR = (merrorR - mlastErrorR) * mkd;

		mlastErrorL = merrorL;
		mlastErrorR = merrorR;

		mcurrentL = mproportionL + mintegralL + mderivativeL;
		mcurrentR = mproportionR + mintegralR + mderivativeR;

		motor[MobileGoalR] = mcurrentR;
		motor[MobileGoalL] = mcurrentL;



		wait1Msec(40);

	}
	return;
}

task main()
{
	while (true){
		if(vexRT[Btn8U] == 1)
		{
			while ((vexRT[Btn8U] == 1) && (SensorValue[BumpTouch2]==0 )){
				MobileGoal_AutoStraight (120);
	wait1Msec (50);
			}
		}
		else if(vexRT[Btn8D] == 1)
		{

			while ((vexRT[Btn8D] == 1)){
				MobileGoal_AutoStraight (-120);
	wait1Msec (50);
			}
		}
		else
		{
			motor[MobileGoalR] = 	motor[MobileGoalL] = 0;

		}
		//-------------------------------------------------------------------------------------------------TUNING
		if(vexRT[Btn6UXmtr2] == 1)
		{
			while ((vexRT[Btn6UXmtr2] == 1) && (SensorValue[BumpTouch2]==0 )){
				MobileGoal_AutoStraight (120);
			wait1Msec (50);
				}
		}
		else if(vexRT[Btn6DXmtr2] == 1)
		{
			while ((vexRT[Btn6DXmtr2] == 1)  && (SensorValue[MobilePotL]< 2100)){
				MobileGoal_AutoStraight (-120);
					wait1Msec (50);
			}
		}
		else
		{
			motor[MobileGoalR] =	motor[MobileGoalL] = 0;
		}



	}
}
