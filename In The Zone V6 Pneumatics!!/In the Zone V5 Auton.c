#pragma config(Sensor, in1,    MobilePotL,     sensorPotentiometer)
#pragma config(Sensor, in2,    MobilePotR,     sensorPotentiometer)
#pragma config(Sensor, in3,    ArmPotL,        sensorPotentiometer)
#pragma config(Sensor, in4,    ArmPotR,        sensorPotentiometer)
#pragma config(Sensor, dgtl1,  ClawArmL,       sensorDigitalOut)
#pragma config(Sensor, dgtl2,  ClawArmR,       sensorDigitalOut)
#pragma config(Sensor, dgtl9,  EncRight,       sensorQuadEncoder)
#pragma config(Sensor, dgtl11, EncLeft,        sensorQuadEncoder)
#pragma config(Motor,  port2,           Left1,         tmotorVex393_MC29, openLoop, driveLeft, encoderPort, dgtl11)
#pragma config(Motor,  port3,           MobileGoalL,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           Left2,         tmotorVex393_MC29, openLoop, driveLeft, encoderPort, dgtl11)
#pragma config(Motor,  port5,           TR,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           TL,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           Right2,        tmotorVex393_MC29, openLoop, reversed, driveRight, encoderPort, dgtl9)
#pragma config(Motor,  port8,           MobileGoalR,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           Right1,        tmotorVex393_MC29, openLoop, reversed, driveRight, encoderPort, dgtl9)

int optimalL;
int optimalR;
int arm_optimal;

int drive_optimalL;
int drive_optimalR;

task PID_Drive()
{

	float kp = 0.7; //tune
	float ki = 0.0001; //tune
	float kd = 0.8; //tune
	//float ks = 1.0; //tune

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
	//	int dr =SensorValue[EncLeft];
	//	int dl =SensorValue[EncRight] - 200; //tune, find difference
/*
		if (dr > dl){
			motor[Right1] = motor [Right2] = currentR / ks;
			motor[Left1] = motor [Left2] = currentL * ks;
		}

		else if(dr <dl) {
			motor[Right1] = motor [Right2] = currentR / ks;
			motor[Left1] = motor [Left2] = currentL * ks;
		}

		else {
		*/
			motor[Right1] = motor [Right2] = currentR;
			motor[Left1] = motor [Left2] = currentL;
	//	}


		wait1Msec(40);
	}
}

/*



int ArmUpdateL;
int ArmUpdateR;
//int drive_optimalL;
//int drive_optimalR;

/*
void MobileGoal_AutoStraight (int MobileGoal_Power){
	if ((vexRT[Btn8U] ==1) || (vexRT[Btn8D]==1)){
		int MG_Left = SensorValue[MobilePotL];
		int MG_Right=SensorValue[MobilePotR] + 100;
		int MG_error = abs (MG_Left - MG_Right);
		float MobileGoal_Constant = 1.1;
while (true){
if (MG_error <= 10){
	motor [ MobileGoalL] = motor[MobileGoalR] = MobileGoal_Power;
}
else{
		if (MG_Left < MG_Right){
			motor [MobileGoalR] = MobileGoal_Power * MobileGoal_Constant;
			motor [MobileGoalL] = MobileGoal_Power / MobileGoal_Constant;
		}
		else if (MG_Left > MG_Right){
			motor [MobileGoalL] = MobileGoal_Power * MobileGoal_Constant;
			motor [MobileGoalR] = MobileGoal_Power / MobileGoal_Constant;
		}
		else	{
			motor [ MobileGoalL] = motor[MobileGoalR] = MobileGoal_Power;
		}
}
		wait1Msec(40);
}
		}
else {
	return;
}
}
*/
/*
int optimalML;
int optimalMR;

task PID_MG()
{
	float Mkp = 0.37; //tune
	float Mki = 0.00001; //tune
	float Mkd = 0.55; //tune

	float currentML ;
	float errorMTl;
	float lastErrorML = 0;
	float proportionML;
	float integralML;
	float derivativeML;
	float currentMR ;
	float errorMTr;
	float lastErrorMR = 0;
	float proportionMR;
	float integralMR;
	float derivativeMR;

	while(true){


		float errorML = optimalL - SensorValue[MobilePotL];
		float errorMR = optimalR - SensorValue[MobilePotR];
		//PidAtTargetL = abs(SensorValue[ArmPotL] - optimalL) < 50;
		//PidAtTargetR = abs(SensorValue[ArmPotR] - optimalR) < 50;


		if (abs(errorML) < 200 && errorML !=0)
		{
			errorMTl = errorML;
		}
		else{
			errorMTl = 0;
		}

		if (abs(errorMR) < 200 && errorMR !=0)
		{
			errorMTr = errorMR;
		}
		else{
			errorMTr = 0;
		}

		if (abs(errorMTl)> 50 / Mki){
			errorMTl = 50 / Mki;
		}

		if (errorML ==0){
			derivativeML = 0;
		}

		if (abs(errorMTr)> 50 / Mki){
			errorMTr = 50 / Mki;
		}

		if (errorMR ==0){
			derivativeMR = 0;
		}




		proportionML = errorML * Mkp;
		proportionMR = errorMR * Mkp;
		integralML = errorMTl  * Mki;
		integralMR = errorMTr  * Mki;
		derivativeML = (errorML - lastErrorML) * Mkd;
		derivativeMR = (errorMR - lastErrorMR) * Mkd;

		lastErrorML = errorML;
		lastErrorMR = errorMR;

		currentML = proportionML + integralML + derivativeML;
		currentMR = proportionMR + integralMR + derivativeMR;

		motor[MobileGoalR] = -currentMR;
		motor[MobileGoalL] = -currentML;



		wait1Msec(40);

	}
	return;
}


task PID_RD4B()
{
	float kp = 0.37; //tune
	float ki = 0.00001; //tune
	float kd = 0.55; //tune

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


		float errorL = optimalL - SensorValue[ArmPotL];
		float errorR = optimalR - SensorValue[ArmPotR];
		//PidAtTargetL = abs(SensorValue[ArmPotL] - optimalL) < 50;
		//PidAtTargetR = abs(SensorValue[ArmPotR] - optimalR) < 50;


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

		motor[TL] = -currentR;
		motor[TR] = -currentL;



		wait1Msec(40);

	}
	return;
}


void RD4B(int a, int z)
{
optimalL = a;
optimalR = z;
waitUntil(SensorValue[armPotL]>=100); //experiment
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
*/
void Drive(int f, int e){
SensorValue (EncLeft) = SensorValue (EncRight) = 0;
drive_optimalL = f;
drive_optimalR = e;
waitUntil (SensorValue[EncLeft]) = abs(f) - 50;
return;
}
/*
void ClawLift (int d, int h){
SensorValue [ClawArmL]= d;
SensorValue [ClawArmR] = h;
wait1Msec(100)

return;
}


*/

task main()
{

startTask (PID_Drive);
//startTask (PID_RD4B);

//RD4B (100,120);
//wait1Msec(50)
Drive (500,500);
}
