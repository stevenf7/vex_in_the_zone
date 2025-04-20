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
int optimalL;
int optimalR;
int arm_optimal;
int drive_optimalL;
int drive_optimalR;

task PID_Drive()
{

	float kp = 1.3; //tune
	float ki = 0.1; //tune
	float kd = 0.2; //tune
	float ks = 1.0; //tune

	//ks is the autostraight constant.
	float currentL = 0;
	float errorTl;
	float lastErrorL;
	float proportionL;
	float integralL;
	float derivativeL;
	float currentR = 0;
	float errorTr;
	float lastErrorR;
	float proportionR;
	float integralR;
	float derivativeR;
	SensorValue[EncLeft] = 0;
	SensorValue[EncRight] = 0;

	//clear the enc values first)

	while(true){


		float errorL = drive_optimalL - SensorValue[EncLeft];
		float errorR = drive_optimalR - SensorValue[EncRight];
		if (abs(errorL) < 1000 && errorL !=0)
		{
			errorTl = errorL;
		}
		else{
			errorTl = 0;
		}

		if (abs(errorR) < 1000 && errorR !=0)
		{
			errorTr = errorR;
		}
		else{
			errorTr = 0;
		}

		if (abs(errorTl)> 50 / ki){
			errorTl = 50 / ki;
		}

		if (errorL ==0){
			derivativeL = 0;
		}

		if (abs(errorTr)> 50 / ki){
			errorTr = 50 / ki;
		}

		if (errorR ==0){
			derivativeR = 0;
		}



		proportionL = errorL * kp;
		proportionR = errorR * kp;
		integralL = errorTl  * ki;
		integralR = errorTr  * ki;
		derivativeL = (errorL - lastErrorL) * kd;
		derivativeR = (errorR - lastErrorR) * kd;

		lastErrorL = errorL;
		lastErrorR = errorR;

		currentL = proportionL + integralL + derivativeL;
		currentR = proportionR + integralR + derivativeR;
		// Autostraightening
		int dr =SensorValue[EncLeft];
		int dl =SensorValue[EncRight] - 200; //tune, find difference

		if (dr > dl){
			motor[Right1] = motor [Right2] = currentR / ks;
			motor[Left1] = motor [Left2] = currentL * ks;
		}

		else if(dr <dl) {
			motor[Right1] = motor [Right2] = currentR / ks;
			motor[Left1] = motor [Left2] = currentL * ks;
		}

		else {
			motor[Right1] = motor [Right2] = currentR;
			motor[Left1] = motor [Left2] = currentL;
		}


		wait1Msec(40);
	}
}




task PID_ClawLift()
{

	float kp = 1.3; //tune
	float ki = 0.1; //tune
	float kd = 0.2; //tune
	float ks = 1.0; //tune

	//ks is the autostraight constant.
	float currentL = 0;
	float errorTl;
	float lastErrorL;
	float proportionL;
	float integralL;
	float derivativeL;
	float currentR = 0;
	float errorTr;
	float lastErrorR;
	float proportionR;
	float integralR;
	float derivativeR;

	while(true){


		float errorL = optimalL - SensorValue[ClawArmPotL];
		float errorR = optimalR - SensorValue[ClawArmPotR];
		if (abs(errorL) < 1000 && errorL !=0)
		{
			errorTl = errorL;
		}
		else{
			errorTl = 0;
		}

		if (abs(errorR) < 1000 && errorR !=0)
		{
			errorTr = errorR;
		}
		else{
			errorTr = 0;
		}

		if (abs(errorTl)> 50 / ki){
			errorTl = 50 / ki;
		}

		if (errorL ==0){
			derivativeL = 0;
		}

		if (abs(errorTr)> 50 / ki){
			errorTr = 50 / ki;
		}

		if (errorR ==0){
			derivativeR = 0;
		}



		proportionL = errorL * kp;
		proportionR = errorR * kp;
		integralL = errorTl  * ki;
		integralR = errorTr  * ki;
		derivativeL = (errorL - lastErrorL) * kd;
		derivativeR = (errorR - lastErrorR) * kd;

		lastErrorL = errorL;
		lastErrorR = errorR;

		currentL = proportionL + integralL + derivativeL;
		currentR = proportionR + integralR + derivativeR;
		// Autostraightening
		int r =SensorValue[ClawArmPotR];
		int l =SensorValue[ClawArmPotL] - 200; //tune, find difference

		if (r > l){
			motor[ClawArmR] = currentR / ks;
			motor[ClawArmL] = currentL * ks;
		}

		else if(r <l) {
			motor[ClawArmR] = currentR * ks;
			motor[ClawArmL] = currentL / ks;
		}

		else {
			motor[ClawArmR] = currentR;
			motor[ClawArmL] = currentL;
		}


		wait1Msec(40);
	}
}


task PID_RD4B()
{

	float kp = 1.3; //tune
	float ki = 0.1; //tune
	float kd = 0.2; //tune


	//ks is the autostraight constant.
	float currentL = 0;
	float errorTl;
	float lastErrorL;
	float proportionL;
	float integralL;
	float derivativeL;


	while(true){


		float errorL = arm_optimal - SensorValue[ArmPot];

		if (abs(errorL) < 1000 && errorL !=0)
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

		integralL = errorTl  * ki;

		derivativeL = (errorL - lastErrorL) * kd;


		lastErrorL = errorL;


		currentL = proportionL + integralL + derivativeL;

		motor[TR] = motor[TL] = currentL;



		wait1Msec(40);
	}
}

int lift_update;

/*
void RD4B(int a)
{
arm_optimal = a;
waitUntil(SensorValue[armPot]>=100); //experiment
wait1Msec (50);
return;
}

void MG(int b)
{
	while (SensorValue[MobilePotL] > b){
motor [MobileGoalL] = motor [MobileGoalR]	= 120;
wait1Msec(50);
}
waitUntil (SensorValue[MobilePotL] > b);
return;
}

void Drive(int f, int e){
SensorValue (EncLeft) = SensorValue (EncRight) = 0;
drive_optimalL = f;
drive_optimalR = e;
waitUntil (EncLeft) == f;
return;
}

void ClawLift (int d, int h){
optimalL = d;
optimalR = h;
waitUntil (SensorValue [ClawArmPotL] == d - 50;

return;
}
*/



task main()
{
startTask (PID_ClawLift);
startTask (PID_Drive);
startTask (PID_RD4B);





}
