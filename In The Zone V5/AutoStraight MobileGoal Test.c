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


void MobileGoal_AutoStraight (int MobileGoal_Power){
	if ((vexRT[btn8U] ==1) || (vexRT[btn8D]==1)){
		int MG_Left = SensorValue[MobilePotL];
		int MG_Right=SensorValue[MobilePotR] + 100;
		float MobileGoal_Constant = 1.0;
		if (MG_Left < MG_Right){
			motor [MobileGoalR] = MobileGoal_Power * MobileGoal_Constant;
			motor [MobileGoalL] = MobileGoal_Power / MobileGoal_Constant;
		}
		else if (MG_Left > MG_Right){
			motor [MobileGoalL] = MobileGoal_Power * MobileGoal_Constant;
			motor [MobileGoalR] = MobileGoal_Power / MobileGoal_Constant;
		}
		else{
			motor [ MobileGoalL] = motor[MobileGoalR] = MobileGoal_Power;
		}
		wait1Msec(40);
		}
else {
	return;
}
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
