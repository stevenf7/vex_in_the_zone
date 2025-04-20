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


task main()
{
	optimalL = SensorValue[ClawArmPotL];
	optimalR = SensorValue[ClawArmPotR];
	arm_optimal = SensorValue [ArmPot];
	startTask (PID_ClawLift);
	startTask (PID_RD4B);
	// Set the optimal value so the task wont be confused, and wont move. Start task


	while (true){
		if(vexRT[Btn5U] == 1)
		{
			while(vexRT[Btn5U] ==1) {
				motor [TL] = motor [TR] = 120;
				lift_update = 	SensorValue[ArmPot];
				wait1Msec (40);

			}
		}
		else if(vexRT[Btn5D] == 1)
		{
			while(vexRT[Btn5D] ==1) {
				motor [TL] = motor [TR] = -120;
				lift_update = 	SensorValue[ArmPot];
				wait1Msec (40);

			}
		}

		else {
			arm_optimal= lift_update;
		}


		float d = vexRT[Ch3Xmtr2] / 127;
		float Ch3_RD4BL = (0.4* pow(d,3)+ 0.6 * d)*127;

		if (vexRT[Ch3Xmtr2] != 0){
			while (vexRT[Ch3Xmtr2]!= 0){
				motor [TR]= motor [TL]= Ch3_RD4BL;
				lift_update = SensorValue [ArmPot];
				wait1Msec (40);
			}
		}
		else {
			arm_optimal = lift_update;
		}

		//----------------------------------------------------------------------------------------------


		if(vexRT[Btn6U] == 1)
		{

				optimalL = 2500; //tune  upper limit
				optimalR = 2500; //tune  upper limit

		}
		else if(vexRT[Btn6D] == 1)
		{

				optimalL = 1500; //tune  Lower limit
				optimalR = 1500; //tune  Lower limit

		}

		else {
		}
		if(vexRT[Btn5UXmtr2] == 1)
		{
				optimalL = 2500; //tune  upper limit
				optimalR = 2500; //tune  upper limit

		}
		else if(vexRT[Btn5DXmtr2] == 1)
		{

				optimalL = 1500; //tune  Lower limit
				optimalR = 1500; //tune  Lower limit

		}
		else {
		}

		float c = vexRT[Ch2Xmtr2] / 127;
		float Ch2_ClawArmLift = (0.641* pow(c,3)+ 0.38 * c)*127;

		while (vexRT[Ch2Xmtr2] != 0){
			motor[ClawArmL] =  motor [ClawArmR]= Ch2_ClawArmLift;

			optimalL = SensorValue[ClawArmPotL];
			optimalR = SensorValue[ClawArmPotR];
		wait1Msec(40);
			}
//-------------------------------------------------------------------------------------------------

wait1Msec(40);
	}
}
