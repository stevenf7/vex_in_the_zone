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

task main()
{
	while(true){
if (vexRT [Btn5U] ==1){
motor[ClawArmL] = motor [ClawArmR] = 120;
}
else if (vexRT [Btn5D] ==1){
motor[ClawArmL] = motor [ClawArmR] = -120;
}
else{
motor[ClawArmL] = motor [ClawArmR] = 0;
}

if (vexRT [Btn8U] ==1){
motor[MobileGoalL] = motor [MobileGoalR] = 120;
}
else if (vexRT [Btn8D] ==1){
motor[MobileGoalL] = motor [MobileGoalR] = -120;
}
else{
motor[MobileGoalL] = motor [MobileGoalR] = 0;
}

		motor[Left1]  = ((Ch3) + (Ch1));
		motor[Left2]  = ((Ch3) + (Ch1));
		motor[Right1] = ((Ch3) - (Ch1));
		motor[Right2] = ((Ch3) - (Ch1));

if (vexRT [Btn6U] ==1){
motor[TL] = motor [TR] = 120;
}
else if (vexRT [Btn6D] ==1){
motor[TL] = motor [TR] = -120;
}
else{
motor[TL] = motor [TR] = 0;
}
}
}
