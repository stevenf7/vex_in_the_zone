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





task main()
{

	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	startTask(PID_Drive);
	;
while (true){
if (vexRT[Btn5U] ==1) {
drive_optimalL = drive_optimalR = 1248;
}

else if (vexRT[Btn5D] ==1) {
drive_optimalL =drive_optimalR = -1248;
}
else{
//motor [Left1] = motor [Left2] = motor [Right1] = motor [Right2] = 0;
}

if (vexRT[Btn8D] ==1) {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive_optimalL =drive_optimalR = -1248;

}
else {
}
wait1Msec(40);
}
}
