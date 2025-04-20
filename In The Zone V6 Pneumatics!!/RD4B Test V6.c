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
int ArmUpdateL;
int ArmUpdateR;


task PID_RD4B()
{
	float kp = 0.02; //tune
	float ki = 0.00001; //tune
	float kd = 0.01; //tune

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

		motor[TL] = currentR;
		motor[TR] = -currentL;



		wait1Msec(40);

	}
	return;
}

task main {

optimalL = SensorValue [ArmPotL];
optimalR = SensorValue [ArmPotR];
	startTask (PID_RD4B);
	// Set the optimal value so the task wont be confused, and wont move. Start task


	while (true){
		if(vexRT[Btn5U] == 1)
		{
			while(vexRT[Btn5U] ==1) {
				motor [TL] = motor [TR] = 120;
			ArmUpdateL = 	SensorValue[ArmPotL];
			ArmUpdateR = SensorValue[ArmPotR];
				wait1Msec (40);

			}
		}
		else if(vexRT[Btn5D] == 1)
		{
			while(vexRT[Btn5D] ==1) {
				motor [TL] = motor [TR] = -120;
			ArmUpdateL = 	SensorValue[ArmPotL];
			ArmUpdateR = SensorValue[ArmPotR];
				wait1Msec (40);

			}
		}

		else {
		optimalL = ArmUpdateL;
		optimalR = ArmUpdateR;
		}
}
}
