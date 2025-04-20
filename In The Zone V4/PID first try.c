#pragma config(Sensor, in1,    ClawArmPotL,    sensorPotentiometer)
#pragma config(Sensor, in2,    ClawArmPotR,    sensorPotentiometer)
#pragma config(Sensor, in3,    ArmPot,         sensorPotentiometer) //if you want to mount a second ArmPot, put it in in6
#pragma config(Sensor, in4,    MobilePotL,     sensorPotentiometer)
#pragma config(Sensor, in5,    MobilePotR,     sensorPotentiometer)
#pragma config(Sensor, dgtl2,  Switch2,        sensorTouch) //Claw lift botttom limit touch sensor
#pragma config(Sensor, dgtl3,  BumpTouch2,     sensorTouch) //claw lift upper limit touch sensor
#pragma config(Sensor, dgtl5,  BumpTouch1,     sensorTouch) //Mobile goal intake limit touch sensor
#pragma config(Sensor, dgtl9,  EncRight,       sensorQuadEncoder) //EncRight second wire goes to dgtl10
#pragma config(Sensor, dgtl11, EncLeft,        sensorQuadEncoder) //EncLeft second wire goes to dgtl112
#pragma config(Motor,  port1,           MobileGoalR,   tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           Left1,         tmotorVex393_MC29, openLoop, driveLeft)
#pragma config(Motor,  port3,           Right1,        tmotorVex393_MC29, openLoop, reversed, driveRight)
#pragma config(Motor,  port4,           ClawArmL,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           TR,            tmotorVex393_MC29, openLoop, reversed)//Y spitters to power expander input signal. TR, BL to output
#pragma config(Motor,  port6,           TL,            tmotorVex393_MC29, openLoop)//Y spliters to power expander input, TL, BR to output
#pragma config(Motor,  port7,           ClawArmR,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           Right2,        tmotorVex393_MC29, openLoop, reversed, driveRight)
#pragma config(Motor,  port9,           Left2,         tmotorVex393_MC29, openLoop, driveLeft)
#pragma config(Motor,  port10,          MobileGoalL,   tmotorVex393_HBridge, openLoop, reversed)
//L for claw arm lift
//R for roller
int ClawliftOptimal;

void PID_ClawLift(int optimal,int actual)
{

float kp = 1.3;
float ki = 0.1;
float kd = 0.2;
// Tuning: set the kp value first, go from 0 to the nearest value which it oscillates, then adjust kd value to prevent it from oscillating
//then increase the kp value again and repeat the procedure untill kd can no longer preventing kd from ocilating
// then find the last kp and kd value.
// for ki, find the value that smooth that out the most

float currentL = 0;
float errorTl;
float lastErrorL;
float proportionL;
float integralL;
float derivativeL;

actual = SensorValue[ClawArmPot];

while(true){
float errorL = optimal - SensorValue[ClawArmPot];
if (abs(errorL) < 1000 && errorL !=0)   //double checking: this is what you mean by inserting absolute values right?
{
	errorTl = errorL;
}
else{
errorTl = 0;
}

if (abs(errorTl)> 50 / ki){
errorTl = 50 / ki;
}

if (errorL ==0){
	derivativeL = 0;
}

proportionL = errorL * kp;
//proportionR = errorR * kp;
integralL = errorTl  * ki;
//integralR = errorTr  * ki;
derivativeL = (errorL - lastErrorL) * kd;
//derivativeR = (errorR - lastErrorR) * kd;

lastErrorL = errorL;
//lastErrorR = errorR;

currentL = proportionL + integralL + derivativeL;


motor[ClawArmL] = motor[ClawArmR] = currentL;
wait1Msec(50); //and here is the wait commend, is that correct?
}

}



int MobileGoalOptimal;

void PID_MobileGoal(int optimal_MobileGoalL,int optimal_MobileGoalR,int actual_MobileGoalL, int actual_MobileGoalR)
{

float kp = 1.3;
float ki = 0.1;
float kd = 0.2;
// Tuning: set the kp value first, go from 0 to the nearest value which it oscillates, then adjust kd value to prevent it from oscillating
//then increase the kp value again and repeat the procedure untill kd can no longer preventing kd from ocilating
// then find the last kp and kd value.
// for ki, find the value that smooth that out the most

float currentML = 0;
float errorTML;
float lastErrorML;
float proportionML;
float integralML;
float derivativeML;

actual_MobileGoalL = SensorValue[MobilePotL];
actual_MobileGoalR = SensorValue[MobilePotR];

while(true){
float errorL = optimal - SensorValue[ClawArmPot];
if (abs(errorL) < 1000 && errorL !=0)   //double checking: this is what you mean by inserting absolute values right?
{
	errorTl = errorL;
}
else{
errorTl = 0;
}

if (abs(errorTl)> 50 / ki){
errorTl = 50 / ki;
}

if (errorL ==0){
	derivativeL = 0;
}

proportionL = errorL * kp;
//proportionR = errorR * kp;
integralL = errorTl  * ki;
//integralR = errorTr  * ki;
derivativeL = (errorL - lastErrorL) * kd;
//derivativeR = (errorR - lastErrorR) * kd;

lastErrorL = errorL;
//lastErrorR = errorR;

currentL = proportionL + integralL + derivativeL;


motor[ClawArmL] = motor[ClawArmR] = currentL;
wait1Msec(50); //and here is the wait commend, is that correct?
}

}








task main() {
		// Claw Arm Control Main Driver
if(vexRT[Btn6U] == 1)
		{
			if ((vexRT[Btn6U] == 1) && (SensorValue[ClawArmPot] < 2800)&& (SensorValue[Switch2]==0) ){
				motor[ClawArmR] = motor[ClawArmL] = 100;
			}
			else if (SensorValue[Switch2]==1){
				motor[ClawArmR]=motor[ClawArmL]=0;
			}

			else {
				ClawliftOptimal = SensorValue[ClawArmPot];
			}
		}
		else if(vexRT[Btn6D] == 1)  	//Else, if button 6D is pressed...
		{
			if ((vexRT[Btn6D] == 1) && (SensorValue[ClawArmPot] > 900) && (SensorValue[BumpTouch1] ==0)){
				motor[ClawArmR] = -100; 		//...close the gripper.
				motor[ClawArmL] = -100;
				}
			else {
				ClawliftOptimal = SensorValue[ClawArmPot];
			}
		}


		else {
			if((SensorValue[ClawArmPot] < 900)||(SensorValue[ClawArmPot]>2800)||(SensorValue[Switch2]==1))
			{motor[ClawArmR]=0;
				motor[ClawArmL]=0;
			}


			else {

				PID_ClawLift(ClawliftOptimal,SensorValue[ClawArmPot]);
			}
}
}
