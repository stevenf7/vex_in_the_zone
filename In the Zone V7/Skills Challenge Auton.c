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
#pragma config(Motor,  port5,           TR,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           TL,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           ClawArmR,      tmotorVex393_MC29, openLoop, reversed, driveRight)
#pragma config(Motor,  port8,           MobileGoalR,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           Right1,        tmotorVex393_MC29, openLoop, reversed, driveRight, encoderPort, dgtl9)
#pragma config(Motor,  port10,          Right2,        tmotorVex393_HBridge, openLoop, reversed)

int optimalL;
int optimalR;
int MoptimalL;
int MoptimalR;
int CoptimalL;
int CoptimalR;

task PID_ClawLift()
{

	float ckp = 0.37; //tune
	float cki = 0.00001; //tune
	float ckd = 0.55; //tune
	//float ks = 1.0; //tune

	//ks is the autostraight constant.
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
		//	PidAtTargetL = abs(SensorValue[ClawArmPotL] - CoptimalL) < 50;
		//	PidAtTargetR = abs(SensorValue[ClawArmPotR] - CoptimalR) < 50;
		/*
		if ((		PidAtTargetL = abs(SensorValue[ClawArmPotL] - optimalL) < 50) || (PidAtTargetR = abs(SensorValue[ClawArmPotR] - optimalR) < 50)){
		motor [ClawArmL] = motor [ ClawArmR] = 5;
		}
		*/

		if (abs(cerrorL) < 200 && cerrorL !=0)
		{
			cerrorTl = cerrorL;
		}
		else{
			cerrorTl = 0;
		}

		if (abs(cerrorR) < 200 && cerrorR !=0)
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
		motor[ClawArmR] = ccurrentR;
		motor[ClawArmL] = ccurrentL;
		//	}


		wait1Msec(40);

	}
	return;
}

task PID_MG()
{
	float kp = 0.3; //tune
	float ki = 0.00001; //tune
	float kd = 0.2; //tune

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


		float errorL = MoptimalL - SensorValue[MobilePotL];
		float errorR = MoptimalR - SensorValue[MobilePotR];
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

		motor[MobileGoalR] = currentR;
		motor[MobileGoalL] = currentL;



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

//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------




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
void MG_mid () {
	MoptimalL = 2000;
	MoptimalR = 2100;

}
void MG_up (){
	MoptimalL = 3756;
	MoptimalR = 3850;
	waitUntil ( abs(SensorValue[MobilePotL]) > abs(3756 - 100));
}

void MG_down() {
	MoptimalL = 1400;
	MoptimalR = 1463;
	waitUntil ( abs(SensorValue[MobilePotL]) > abs(1400 - 100));
}
//---------------------------------------------------------------------------------------------
task a1 (){
RD4B (1279,906);
return;
}

task a2 () {
wait1Msec (500);
SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive (1300, 1300);
return;
}
task a3 () {
wait1Msec(500);
MG (1400,1463);
return;
}

void a (){
startTask (a1 );
startTask (a2 );
startTask (a3 );
wait1Msec (3000);
stopTask (a1 );
stopTask (a2 );
stopTask (a3 );
}
//-----------------------------------------------------------------------------------------------
task b1 () {
MG (3756, 3850);
}



void b (){
startTask (b1 );
startTask (b2 );

wait1Msec (2500);
stopTask (b1 );
stopTask (b2 );

}
//-------------------------------------------------------------------------------------------------------
void c () {
SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive (-1185, -1185);
wait1Msec (2000);
return;
}
void c1 () {
SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive (130, -150);
wait1Msec (700);
return;
}
void c2 (){
SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive (-650, -650);
wait1Msec (1000);
return;
}
void c3 () {
SensorValue[EncLeft] = SensorValue [EncRight] = 0;
drive (400, -400);
wait1Msec (500);
return;
}
//---------------------------------------------------


void e (){

motor[Left1] = motor [Left2] = motor[Right1] = motor [Right2] = 120;
wait1Msec (2000);

}
//-------------------------------------------------------





task f1a() {
wait1Msec(500);
stopTask(PID_Drive);
motor[Left1] = motor [Left2] = motor[Right1] = motor [Right2] = -120;
wait1Msec(600);
startTask(PID_Drive);
return;
}

task f4a () {
MG (1400,1487);
wait1Msec(800);
return;
}

void f2 () {
startTask (f1a);
startTask (f4a);
wait1Msec(1000);
stopTask (f1a);
stopTask(f4a);
}

void f3() {
	wait1Msec(1500);
MG (3756, 3850);
}

void f4 () {

motor[Left1] = motor [Left2] = motor[Right1] = motor [Right2] = -120;
wait1Msec (2000);
}
//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
//------------------------------------------------------
//----------------------Standard Auton Ends, Skills Challenge Auton begin
//turn 90 degree counterclockwise
void e1 () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

// drive back and turn 45 wish degrees
void e2 () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

// drive back till it hit the wall
void e3 () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

//turn if necessary
void e3a () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

task e4a (){
	MG_down();
	return;
}
//drive forward
task e4b (){
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

void e4 () {
	startTask (e4a);
	startTask (e4b);
	wait1Msec(1500);
	stopTask (e4a);
	stopTask (e4b);
	return;
}

void e5 () {
	wait1Msec(300);
	MG_up(); //myabe consider halfway
	wait1Msec(200);
	return;
}

//turn 90 degrees to the right
void g1 () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

// drive forward to the 10 point zone
void g2 () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

void g3 () {
	MG_down ();
	return;
}

//drive back slightly to release
task g4a (){
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

task g4b (){
	wait1Msec (500);
	MG_up ();
	return;
}

//release 1 mg in the 10 point zone
void g4 () {
	startTask (g4a);
	startTask (g4b);
	wait1Msec(1500);
	stopTask (g4a);
	stopTask (g4b);
	return;
}

//Turn 180 degrees
void h1 () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

task h2a () {
	MG_mid();
	return;
}
//drive forward to MG2
task h2b() {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}
void h2 () {
	startTask (h2a);
	startTask (h2b);
	wait1Msec (1500);
	stopTask (h2a);
	stopTask (h2b);
	return;
}
///turn 90 degrees to the right
void h3 () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

task h4a (){
	MG_down();
	return;
}
task h4b () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}


void h4 (){
	startTask (h4a);
	startTask (h4b);
	wait1Msec (1000);
	stopTask (h4a);
	stopTask (h4b);
	return;
}

//turn 90 degrees to the right
void h5 (){
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}


task h6a () {
	wait1Msec (700);
	MG_mid();
	return;
}

//drive up to teh 10 point zone bar to release
task h6b () {
	wait1Msec (400);
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

void h6 () {
	startTask (h6a);
	startTask (h6b);
	wait1Msec (1400);
	stopTask (h6a);
	stopTask (h6b);
	return;
}

void h7 () {
	MG_down();
	return;
}

void i1 () {
	MG_mid();
	return;
}

//turn 180 degrees
void i2 () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

void i3a (){
	MG_down();
	return;
}

//drive forward to the 4th Mobile goal
void i3b () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}
/*
void i3 () {
startTask (i3a);
startTask (i3b);
wait1Msec (3000);
stopTask (i3a);
stopTask (i3b);
return;
}
*/
void i4a (){
	MG_mid();
	return;
}

//drive forward to the blue 10 point bar
void i4b () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}
/*
void i4 () {
startTask (i4a);
startTask (i4b);
wait1Msec (3000);
stopTask (i4a);
stopTask (i4b);
return;
}
*/
//place teh 4th mobile goal in the 10 point zone
void i5 () {
	MG_down();
	return;
}

void i6 () {
	MG_mid();
	return;
}

//turn 90 degrees to the left
void j1 () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}
///drive forward
void j2 () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}


// turn left to line up with the fifth mobile goal
void j3 (){
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

void j4a (){
	MG_down();
	return;
}

//drive forward to the fifth mobile goal
void j4b () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}


/*
void j4 () {
startTask (j4a);
startTask (i4b);
wait1Msec (1500);
stopTask (j4a);
stopTask (j4b);
return;
}
*/

void j5 () {
	MG_up();
	return;
}

//180 degrees turn
void j6 () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

//drive forward to the white line
void j7 () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

//90 degrees turn to the right
void j8 () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

//drive forward
void j9 () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

//90 degrees turn to the right
void j10 () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

void j11 () {
	MG_mid ();

}

//drive forward to teh 20 point zone
void j12 () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}
void j13 () {
	MG_down ();
}

void j14 () {
startTask (g4a); //mobile goal up
//Drive back
SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
wait1Msec(1500);
stopTask (g4a);

}

//Turn clockwise  90 degrees
void l1 () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

//drive back to hit the right fence
void l2 () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

//turn 90 degrees to the right
void l3 () {
		SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

void l4 (){
MG_down();
return;
}

//drive forward to the sixth mobilegoal
void l5 () {
		SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

void l6() {
MG_up ();
return;
}

//turn 180 degrees
void l7 () {
		SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

void l8 () {
MG_mid ();
return;
}

//drive to the 10 point zone
void l9 () {
		SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

void l10 () {
MG_down ();
return;
}

//drive back to get parking bonus?
void l11 () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1000, -1000);
	return;
}

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
	b();
	c();
	c1();
	c2();
	c3();
	stopTask (PID_Drive);
	e();
	startTask(PID_Drive);
	//f1();
	f2();
	//f3();
	f4();

}
