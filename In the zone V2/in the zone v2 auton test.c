#pragma config(Sensor, in1,    ClawArmPot,     sensorPotentiometer)
#pragma config(Sensor, in2,    ArmPot,         sensorPotentiometer)
#pragma config(Sensor, in4,    MobilePot,      sensorPotentiometer)
#pragma config(Sensor, in5,    ClawPot,        sensorPotentiometer)
#pragma config(Sensor, dgtl1,  EncLeft,        sensorQuadEncoder)
#pragma config(Sensor, dgtl2,  Switch2,        sensorTouch)
#pragma config(Sensor, dgtl3,  BumpTouch2,     sensorTouch)
#pragma config(Sensor, dgtl4,  BumpTouch1,     sensorTouch)
#pragma config(Sensor, dgtl11, EncRight,       sensorQuadEncoder)
#pragma config(Motor,  port1,           MobileGoalR,   tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           Left1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           Right1,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           Claw,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           TR,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           TL,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           ClawArmR,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           Right2,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           Left2,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          MobileGoalL,   tmotorVex393_HBridge, openLoop)





void autostraight (int leftmotor, int rightmotor) {
if ((SensorValue[EncLeft]) == (SensorValue [EncRight])){
motor[Left1]= leftmotor;
motor[Left2]= leftmotor;
motor[Right1]= rightmotor;
motor[Right2] = rightmotor;
}
else if ((SensorValue[EncLeft]) > (SensorValue [EncRight])){
motor[Left1]= leftmotor - 5;
motor[Left2]= leftmotor - 5;
motor[Right1]= 5 + rightmotor;
motor[Right2] = 5 + rightmotor;

}

else {
motor[Left1]= leftmotor + 5;
motor[Left2]= leftmotor + 5;
motor[Right1]=  rightmotor - 5;
motor[Right2] = rightmotor - 5;
}
}


task main()
{
while(true) {
while (vexRT[Btn8R] ==1) {

SensorValue [EncLeft] = 0;
SensorValue[EncRight] = 0;

while ((SensorValue[ClawArmPot]>1700 )&& (SensorValue[Switch2]==0)){
	motor[ClawArmR] = 80;
}
// Clawarm move up

while ((SensorValue[MobilePot]< 2100)){
	motor[MobileGoalL] = -80;
	motor[MobileGoalR] = -80;
}
// extent mobilegoal
while (SensorValue [EncLeft] < 500) {
autostraight(100,100)	;
}
//move forward

while (SensorValue [ClawArmPot] <500) {
motor [ClawArmR] = -80;

}

//lower clawlift

while (SensorValue [ClawPot] <500) {
motor [Claw] = 80;

}
// Release cone
while (SensorValue [ClawArmPot] >1500) {
motor [ClawArmR] = 80;

}
// raise clawlift
while ((SensorValue[BumpTouch2]==0)|| (SensorValue[MobilePot] < 5)) {
	motor[MobileGoalL] = 100;
	motor[MobileGoalR] = 100;
}
// move mobilegoal up
SensorValue [EncLeft] = 0;
SensorValue[EncRight] = 0;

while (SensorValue [EncLeft] < 500){
autostraight(-100,-100);
}
//moves back

SensorValue [EncLeft] = 0;
SensorValue[EncRight] = 0;

while (SensorValue [EncLeft] <100) {
motor[Left1]= -50;
motor[Left2]= -50;
motor[Right1]= 50;
motor[Right2] = 50;
}
//turn left
SensorValue [EncLeft] = 0;
SensorValue[EncRight] = 0;

while (SensorValue [EncLeft] < 100){
autostraight(100,100);
}
//move forward to the center position

while (SensorValue [EncLeft] <100) {
motor[Left1]= -50;
motor[Left2]= -50;
motor[Right1]= 50;
motor[Right2] = 50;
}
// turn left again
while (SensorValue [EncLeft] < 100){
autostraight(70,70);
}
// mobe forward to the one point zone

while ((SensorValue[MobilePot]< 2100)){
	motor[MobileGoalL] = -80;
	motor[MobileGoalR] = -80;
}

//lower the claw and place the goal down
SensorValue [EncLeft] = 0;
SensorValue[EncRight] = 0;
while (SensorValue [EncLeft] < 100) {
autostraight(-70,-70);
}
//move back


}
if (vexRT [Btn7D] == 1) {
	SensorValue [EncLeft] = 0;
SensorValue[EncRight] = 0;
}
else
{

}

}
}
