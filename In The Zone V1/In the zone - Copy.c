#pragma config(Sensor, in1,    armPot,         sensorPotentiometer)
#pragma config(Sensor, in2,    armPotL,        sensorPotentiometer)
#pragma config(Sensor, in3,    ClawArmPot,     sensorPotentiometer)
#pragma config(Sensor, in4,    ClawPot,        sensorPotentiometer)
#pragma config(Sensor, in5,    MobilePot,      sensorPotentiometer)
#pragma config(Sensor, dgtl1,  LeftMotorENC,   sensorRotation)
#pragma config(Sensor, dgtl2,  RightMotorENC,  sensorRotation)
#pragma config(Sensor, dgtl3,  BumpTouch1,     sensorTouch)
#pragma config(Sensor, dgtl4,  BumpTouch2,     sensorTouch)
#pragma config(Sensor, dgtl5,  Switch1,        sensorTouch)
#pragma config(Sensor, dgtl6,  Switch2,        sensorTouch)
#pragma config(Motor,  port1,           MobileGoalR,   tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           Left1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           Right1,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           ClawArmR,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           RLLeft,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           RLRight,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           Claw,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           Right2,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           Left2,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          MobileGoalL,   tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//BumpTouch1 is for Claw LIft
//BumpTouch2 is for Mobile Goal Intake
//Switch 1 is for RD4B

int ClawPotMin = 100;
int ClawPotMax = 400;

int LiftPotMin = 200;
int LiftPotMax = 400;

int ClawLiftMin= 100;
int ClawLiftMax= 400;

int MobileGoalMin = 100;
int MobileGoalMax = 400;

//Constant for RD4B lift PID
float KP = 0.5;

//Constant for Claw Lift PID
float KP2 = 0.8;

//Constant for Claw PID
float KP3 = 0.3;

int Ayush;
int Ayush2;
int Ayush3;



void PID (int optimal)
{
	int error = (optimal - SensorValue [armPot]);
	int output = error * KP;
	motor [RLLeft] = output;
	motor [RLRight] = output;
}
//PID for the Reverse Double four bar LIft



void PID2 (int optimal2)
{
	int error2 = (optimal2 - SensorValue [ClawArmPot]);
	int output2 = error2 * KP2;
	motor [ClawArmR] = output2;

}
//PID for claw  lift


void PID3 (int optimal3)
{
	int error3 = (optimal3 - SensorValue [ClawPot]);
	int output3 = error3 * KP3;
	motor [Claw] = output3;

}
//PID for Claw
//For Reverse Double 4 bar lift

//-----------------------------------------------------------------------------------------------------------------------------------------------//


task main()
{
while(true)
	{
		motor[Left1]  = (vexRT[Ch3] + vexRT[Ch4])*0.9;
		motor[Left2]  = (vexRT[Ch3] + vexRT[Ch4])*0.9;
		motor[Right1] = (vexRT[Ch3] - vexRT[Ch4])*0.9;
		motor[Right2] = (vexRT[Ch3] - vexRT[Ch4])*0.9;

//-----------------------------------------------------------------------------------------------------------------------------------------------//
//RD4B Lift

if ((SensorValue [armPot] > LiftPotMin)  || (SensorValue [Switch1] == 0)){
		motor[RLLeft]  = (vexRT[Ch2])*0.9;
		motor[RLRight] = (vexRT[Ch2])*0.9;

		Ayush = SensorValue[armPot];

	}
else if ((SensorValue[armPot] <LiftPotMax) || (SensorValue[Switch1]==0)){
		motor[RLLeft]  = (vexRT[Ch2])*0.9;
		motor[RLRight] = (vexRT[Ch2])*0.9;

		Ayush = SensorValue[armPot];
}
	else
{
	PID(Ayush);

}
//------------------------------------------------------------------------------------------------------------------------------------------------//

		//claw arm control
if (vexRT [Btn5U]== 1)
{
	if (SensorValue[ClawArmPot] > ClawLiftMin ||SensorValue(BumpTouch1) == 0)
		{
	motor[ClawArmR]= 120;
	Ayush2 = SensorValue[ClawArmPot];
}

	if ((SensorValue[ClawArmPot] < ClawLiftMax ) || (SensorValue(BumpTouch1) == 0)){
	motor[ClawArmR]= 120;
	Ayush2 = SensorValue[ClawArmPot];
}

else {
 PID2(Ayush2);
}
}


if (vexRT [Btn5D]== 1)
{
	if ((SensorValue[ClawArmPot] > ClawLiftMin) || (SensorValue(BumpTouch1) == 0))
		{
	motor[ClawArmR]= -127;
	Ayush2 = SensorValue[ClawArmPot];
}

if (( SensorValue[ClawArmPot] < ClawLiftMax ) || (SensorValue(BumpTouch1) ==0))
{
  motor[ClawArmR]= -127;
  Ayush2 = SensorValue[ClawArmPot];
}

else {
PID2 (Ayush2);
}
}

//----------------------------------------------------------------------------------------------------------------------------------------------------//
// Claw Control -OPEN
if (vexRT [Btn6U]== 1)
{
	if (SensorValue[ClawPot] > ClawPotMin)
		{
	motor[Claw]= 120;
	Ayush3 = SensorValue[ClawPot];
}

if (SensorValue[ClawPot] < ClawPotMax )
{
	motor[Claw]= 120;
		Ayush3 = SensorValue[ClawPot];
}

else {
	PID3 (Ayush3);
}
}

//close
if (vexRT [Btn6D]== 1)
{
	if (SensorValue[ClawPot] > ClawPotMin)
		{
	motor[Claw]= -120;
	Ayush3 = SensorValue[ClawPot];
}

	if (SensorValue[ClawPot] < ClawPotMax )
	{
	motor[Claw]= -120;
	Ayush3 = SensorValue[ClawPot];
}

else {
	PID3 (Ayush3);
}
}

//-------------------------------------------------------------------------------------------------------------------------------------------------//
//Mobilegoal Intake up

if (vexRT [Btn8L]== 1)
{
	if ((SensorValue[MobilePot] > MobileGoalMin) || (SensorValue(BumpTouch2) == 0))
		{
	motor[MobileGoalL]= 100;
	motor[MobileGoalR]=100;

}

if ((SensorValue[MobilePot] < MobileGoalMax) || (SensorValue(BumpTouch2) ==0))
{
	motor[MobileGoalL]= 100;
	motor[MobileGoalR]=100;

}

else {
		motor[MobileGoalL]= 0;
	motor[MobileGoalR]=0;
}
}


if (vexRT [Btn8D]== 1)
{
	if ((SensorValue[MobilePot] > MobileGoalMin) || ( SensorValue(BumpTouch2) == 0))
		{
	motor[MobileGoalL]= -100;
	motor[MobileGoalR]=-100;

}

if (( SensorValue[MobilePot] < MobileGoalMax) ||( SensorValue(BumpTouch2) == 0))
{
	motor[MobileGoalL]= -100;
	motor[MobileGoalR]=-100;
}

else {
	motor[MobileGoalL]= 0;
	motor[MobileGoalR]=0;
}
}

//-----------------------------------------------------------------------------------------------------------------------------------------------//
}
//Below is the main taskmain
}
