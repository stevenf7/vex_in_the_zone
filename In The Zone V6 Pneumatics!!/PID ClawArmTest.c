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
int PidAtTargetL;
int PidAtTargetR;

task PID_ClawLift()
{

	float kp = 0.37; //tune
	float ki = 0.00001; //tune
	float kd = 0.55; //tune
	//float ks = 1.0; //tune

	//ks is the autostraight constant.
	float currentL ;
	float errorTl;
	float lastErrorL = 0;
	float proportionL;
	float integralL;
	float derivativeL;
	float currentR ;
	float errorTr;
	float lastErrorR = 0;
	float proportionR;
	float integralR;
	float derivativeR;

	while(true){


		float errorL = optimalL - SensorValue[ClawArmPotL];
		float errorR = optimalR - SensorValue[ClawArmPotR];
		PidAtTargetL = abs(SensorValue[ClawArmPotL] - optimalL) < 50;
		PidAtTargetR = abs(SensorValue[ClawArmPotR] - optimalR) < 50;
/*
		if ((		PidAtTargetL = abs(SensorValue[ClawArmPotL] - optimalL) < 50) || (PidAtTargetR = abs(SensorValue[ClawArmPotR] - optimalR) < 50)){
			motor [ClawArmL] = motor [ ClawArmR] = 5;
		}
*/

		if (abs(errorL) < 200 && errorL !=0)
		{
			errorTl = errorL;
		}
		else{
			errorTl = 0;
		}

		if (abs(errorR) < 200 && errorR !=0)
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
		//int r =SensorValue[ClawArmPotR];
		//int l =SensorValue[ClawArmPotL] - 200; //tune, find difference

		/*if (r > l){
		motor[ClawArmR] = currentR / ks;
		motor[ClawArmL] = currentL * ks;
		}

		else if(r <l) {
		motor[ClawArmR] = currentR * ks;
		motor[ClawArmL] = currentL / ks;
		}

		else {
		*/
		motor[ClawArmR] = -currentR;
		motor[ClawArmL] = -currentL;
		//	}


		wait1Msec(40);

	}
	return;
}

task main()
{
	optimalL = SensorValue[ClawArmPotL];
	optimalR = SensorValue[ClawArmPotR];

	startTask (PID_ClawLift);
	while (true){

		if(vexRT[Btn6U] == 1)
		{

			optimalL = 3632; //tune  upper limit
			optimalR = 3404; //tune  upper limit

		}
		else if(vexRT[Btn6D] == 1)
		{

			optimalL = 1891; //tune  Lower limit
			optimalR = 1700; //tune  Lower limit

		}

		else{
		}
	}




}
