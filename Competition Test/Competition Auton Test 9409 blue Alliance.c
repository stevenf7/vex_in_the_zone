#pragma config(Sensor, in1,    MobilePotL,     sensorPotentiometer)
#pragma config(Sensor, in2,    MobilePotR,     sensorPotentiometer)
#pragma config(Sensor, in3,    ArmPotL,        sensorPotentiometer)
#pragma config(Sensor, in4,    ArmPotR,        sensorPotentiometer)
#pragma config(Sensor, in5,    ClawArmPotL,    sensorPotentiometer)
#pragma config(Sensor, in6,    ClawArmPotR,    sensorPotentiometer)
#pragma config(Sensor, dgtl9,  EncRight,       sensorQuadEncoder)
#pragma config(Sensor, dgtl11, EncLeft,        sensorQuadEncoder)
#pragma config(Motor,  port1,           Left2,         tmotorVex393_HBridge, openLoop, reversed, driveLeft, encoderPort, dgtl11)
#pragma config(Motor,  port2,           Left1,         tmotorVex393_MC29, openLoop, driveLeft, encoderPort, dgtl11)
#pragma config(Motor,  port3,           MobileGoalL,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           ClawArmL,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           TR,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           TL,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           ClawArmR,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           MobileGoalR,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           Right1,        tmotorVex393_MC29, openLoop, reversed, driveRight, encoderPort, dgtl9)
#pragma config(Motor,  port10,          Right2,        tmotorVex393_HBridge, openLoop, reversed, driveRight, encoderPort, dgtl9)

//Hello, This is our autonomous
//The main problem is, the functions dont run as smoothly,there is sometimes a pause inbetween (likely because alot of it is time based)
//Ideally, a PID near target value can be implemented to eliminate the pauses
//Also, the drive pid is not very precise, sometimes the robot curves to the right even with PID enabled.


int optimalL;
int optimalR;
int MoptimalL;
int MoptimalR;
int CoptimalL;
int CoptimalR;


task PID_ClawLift()
{
	float ckp = 0.24; //tune
	float cki = 0.004; //tune
	float ckd = 0.28; //tune

	float ccurrentL ;
	float cerrorTl;
	float clastErrorL = 0;
	float cproportionL;
	float cintegralL;
	float cderivativeL;
	float ccurrentR ;
	float cerrorTr;
	float clastErrorR = 0;
	float cproportionR;
	float cintegralR;
	float cderivativeR;

	while(true){


		float cerrorL = CoptimalL - SensorValue[ClawArmPotL];
		float cerrorR = CoptimalR - SensorValue[ClawArmPotR];



		if (abs(cerrorL) < 200 && cerrorL !=0)
		{
			cerrorTl = cerrorL;
		}
		else{
			cerrorTl = 0;
		}

		if (abs(cerrorR) < 200 &&cerrorR !=0)
		{
			cerrorTr = cerrorR;
		}
		else{
			cerrorTr = 0;
		}

		if (abs(cerrorTl)> 50 / cki){
			cerrorTl = 50 / cki;
		}

		if (cerrorL ==0){
			cderivativeL = 0;
		}

		if (abs(cerrorTr)> 50 / cki){
			cerrorTr = 50 / cki;
		}

		if (cerrorR ==0){
			cderivativeR = 0;
		}




		cproportionL = cerrorL * ckp;
		cproportionR = cerrorR * ckp;
		cintegralL = cerrorTl  * cki;
		cintegralR = cerrorTr  * cki;
		cderivativeL = (cerrorL - clastErrorL) * ckd;
		cderivativeR = (cerrorR - clastErrorR) * ckd;

		clastErrorL = cerrorL;
		clastErrorR = cerrorR;

		ccurrentL = cproportionL + cintegralL + cderivativeL;
		ccurrentR = cproportionR + cintegralR + cderivativeR;

		motor[ClawArmR] = -ccurrentR;
		motor[ClawArmL] = -ccurrentL;



		wait1Msec(40);

	}
	return;
}
task PID_MG()
{
	float mkp = 0.3; //tune
	float mki = 0.001; //tune
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


		float merrorL = (MoptimalL - SensorValue[MobilePotL]);
		float merrorR = (MoptimalR - SensorValue[MobilePotR]);



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

task PID_RD4B()
{
	float kp = 0.5; //tune
	float ki = 0.0001; //tune
	float kd = 0.4; //tune

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

		motor[Right1] = motor [Right2] = currentR;
		motor[Left1] = motor [Left2] = currentL;



		wait1Msec(40);
	}
}





//secondary functions to simplify the coding process
//not sure how the waitUntil funcitons work


void drive(int LeftDrive, int RightDrive){
	SensorValue (EncLeft) = SensorValue (EncRight) = 0;
	drive_optimalL = LeftDrive;
	drive_optimalR = RightDrive;
	//waitUntil (abs(SensorValue[EncLeft]) < LeftDrive - 50);
	wait1Msec (50);
	return;
}

void RD4B (int a, int b){
	optimalL = a;
	optimalR = b;
	waitUntil ( abs(SensorValue[ArmPotL]) > abs(a - 100));
}


void MG (int x, int y){
	MoptimalL = x;
	MoptimalR = y;
	waitUntil ( abs(SensorValue[MobilePotL]) > abs(x - 100));
}
void clawlift (int g, int h){
	CoptimalL = g;
	CoptimalR = h;
	waitUntil ( abs(SensorValue[ClawArmPotL]) > abs(g - 100));
}

//---------------------------------------------------------------------------------------------
task a1 (){
	RD4B (1484,985);
	wait1Msec(500);
	return;
}

task a2 () {
	wait1Msec (300);
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (1400, 1400);
wait1Msec(1600);
	return;
}
task a3 () {
	wait1Msec(300);
	MG (1450,1510);
	wait1Msec(1000);
	return;
}

void a (){
	startTask (a1 );
	startTask (a2 );
	startTask (a3 );
	wait1Msec (2300);
	stopTask (a1 );
	stopTask (a2 );
	stopTask (a3 );
	return;
}
//-----------------------------------------------------------------------------------------------
void b1 () {
	MG (4020,3822);
		wait1Msec (600);
		return;
}
task b2 () {

	RD4B (1125, 605);
	wait1Msec (300);
	return;
}
task b3 () {
	wait1Msec(500);
	clawlift (2430,2650);
	wait1Msec (400);
	return;
}
void c(){
startTask(b2);
startTask(b3);

wait1Msec(1800);
stopTask(b2);
stopTask(b3);

return;
}

void z1 (){
	RD4B (1600, 1260);
	wait1Msec (300);
	return;
}

void z2 () {
		SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (250, 250);
	wait1Msec (300);
	return;
}

void z3 () {

	RD4B (992, 572);
	wait1Msec (400);
	return;
}

void z4 () {
	RD4B (1547, 1224);
	wait1Msec (400);
	return;
}

void z5 () {
	clawlift(754,962);
	wait1Msec(400);
	return;
}

void z6 () {

	RD4B (1093,664);
	wait1Msec (600);
	return;
}
void z7 () {

	clawlift (2397,2499);
	wait1Msec (400);
	return;
}

//-------------------------------------------------------------------------------------------------------
void c1b () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1060, -1060);
	wait1Msec (1000);
	return;
}

void c2 () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (620, -620);
	wait1Msec (800);
	return;
}

void c3 (){
MG (1900,1900);
wait1Msec(400);
}

void c4(){
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-200, -200);
	wait1Msec (200);
	return;
}

//-------------------------------------------------------





//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
task main()
{
	optimalL = SensorValue [ArmPotL];
	optimalR = SensorValue [ArmPotR];
	MoptimalL = SensorValue [MobilePotL];
	MoptimalR = SensorValue [MobilePotR];
	CoptimalL = SensorValue [ClawArmPotL];
	CoptimalR = SensorValue [ClawArmPotR];
	startTask(PID_RD4B);
	startTask(PID_Drive);
	startTask(PID_MG);
	startTask(PID_ClawLift);
	a();
	b1();
	c();
	z1();
	z2();
	z3();
	z4();
	z5();
	z6();
	z7();
	c1b();
	c2();
	c3();
	c4();

	stopTask(PID_RD4B);
	stopTask(PID_Drive);
	stopTask(PID_ClawLift);
	stopTask (PID_MG);
}
