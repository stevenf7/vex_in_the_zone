
#ifndef MecDrive.c
#define MecDrive.c
#define JOYSTICK_DEADZONE  6
#include "Utils.c"
#include "PIDController.h"
#include "GyroLib.c"
#include "Constants.h"

const float LEFT_RIGHT_OFFSET = 1;
const float AUTO_MAX_POW = 80;
typedef struct {
	tMotor fl;
	tMotor fr;
	tMotor bl;
	tMotor br;
	tMotor ml;
	tMotor mr;
	tSensors encLeft;
	tSensors encRight;
	tSensors gyro;
	PID master;
	PID slave;
	PID gyroPID;
	float netDistance;
	bool pidEnabled;
} DriveBase;
DriveBase mec;

float mec_getDriveErr(){
	return mec.master.error;
}

void zeroDriveEncoders(){
	SensorValue[mec.encLeft] = 0;
	SensorValue[mec.encRight] = 0;
}

void _setLeftDrivePow(int power){
	if(power == 0){
		motor[mec.fl] = 0;
		motor[mec.bl] = 0;
		motor[mec.ml] = 0;
		return;
	}
	setLinMotorPow(mec.fl,power);
	setLinMotorPow(mec.bl,power);
	setLinMotorPow(mec.ml,power);
}

void _setRightDrivePow(int power){
	if(power == 0){
		motor[mec.fr] = 0;
		motor[mec.br] = 0;
		motor[mec.mr] = 0;
		return;
	}
	setLinMotorPow(mec.br,power);
	setLinMotorPow(mec.fr,power);
	setLinMotorPow(mec.mr,power);
}

float _getLeftEnc(){
	return SensorValue[mec.encLeft];
}

float _getRightEnc(){
	return -SensorValue[mec.encRight];
}

float GYRO_KP = 6.3 * 0.6;//6.5/1.7;
float GYRO_KI = 1.2 / 2.0;//2.8/2.0;
float GYRO_KD = 1.5 / 3.2;//3.3/8.0;
float GYRO_INTLIM = 1270;
float GYRO_ERROR_THRESH = 2;

void mec_GyroTurnAbs(int degrees, bool escapable){
	pidInit(mec.gyroPID, GYRO_KP,GYRO_KI,GYRO_KD,0,GYRO_INTLIM);
	pidInit(mec.slave, 	0.03,0,0,0,1270);
	pidReset(mec.gyroPID);
	pidReset(mec.slave);

	mec.pidEnabled = false;
	degrees = degrees % 360;

	bool targetReached = false;
	int setPoint = degrees;

	long timeInit = nPgmTime - 20;
	long atTargetTime = nPgmTime;

	float initTicksL = _getLeftEnc();
	float initTicksR = _getRightEnc();

	int integralLimit = 17 / mec.gyroPID.kI;
	int escapeThresh = 30;

	float lastLTicks = initTicksL;
	long initTime = nPgmTime;
	while(!targetReached){

		timeInit = nPgmTime;

		//printPIDDebug(mec.gyroPID);
		//deal with input values
		float error = setPoint - GyroGetAngle();
		if(abs(error) > abs( setPoint - (GyroGetAngle() + 360))){
			error = setPoint - (GyroGetAngle() + 360) ;
		}
		else if(abs(error) > abs( setPoint - (GyroGetAngle() - 360))){
			error = setpoint - (GyroGetAngle() - 360);
		}

		float slaveErr = (-(_getLeftEnc() - initTicksL)) - (_getRightEnc() - initTicksR);
		float slaveOut = pidExecute(mec.slave, slaveErr);
		float driveOut = pidExecute(mec.gyroPID,error);

		writeDebugStreamLine("%f\t%f", nPgmTime - initTime, error);



		if(sgn(mec.gyroPID.error) != sgn(mec.gyroPID.lastError)){
			mec.gyroPID.errorSum = 0;
		}

		if(abs(mec.gyroPID.errorSum) > integralLimit){
			mec.gyroPID.errorSum = integralLimit * mec.gyroPID.errorSum/abs(mec.gyroPID.errorSum);
		}

		int minVal = 0;
		if(abs(error) > GYRO_ERROR_THRESH){
			atTargetTime = nPgmTime;
			if(abs(driveOut) < minVal){
				if(driveOut != 0){
					driveOut += minVal * driveOut / abs(driveOut);
				}
			}
		}

		_setLeftDrivePow((driveOut + slaveOut));
		_setRightDrivePow(-(driveOut - slaveOut));

		if(nPgmTime - atTargetTime > 200){
			targetReached = true;
			_setLeftDrivePow(0);
			_setRightDrivePow(0);
			break;
		}
		wait1Msec(20);
	}
}

void mec_GyroTurnAbs(float degrees){
	mec_GyroTurnAbs(degrees, false);
}

void mec_GyroTurnRel(int degrees){
	if(degrees < 0){
		degrees += 360;
	}
	if(degrees > 360){
		degrees -= 360;
	}
	mec_GyroTurnAbs(GyroGetAngle() + degrees);
}

const int DRIVE_SAT_TIME = 200;

float _DRIVE_KP = 0.9 * 0.6;//0.7/1.7;
float _DRIVE_KI = 0.880 / 2.0;//0.2/2.0;
float _DRIVE_KD = 0.880 / 8.0;//0.6/8.0;
float _DRIVE_SLEW = 1270;
float _SLAVE_KP = 0.4;//0.4;
float _SLAVE_KI = 0.01;
float _SLAVE_KD = 0.04;//0.880 / 8.0;//0.01;

void mec_driveInches(float inches, int maxSpeed,int expiryms, float turnRatio = 1){
	//def constants
	pidInit(mec.master, _DRIVE_KP,_DRIVE_KI,_DRIVE_KD,0,_DRIVE_SLEW);
	pidInit(mec.slave, _SLAVE_KP,_SLAVE_KI,_SLAVE_KD,0,1270);
	pidReset(mec.master);
	pidReset(mec.slave);

	float integralLimit = 20 / _DRIVE_KI;

	//turn off pid task
	mec.pidEnabled = false;
	bool targetReached = false;

	float setPoint = inches * TICKS_PER_INCHES;
	long timeInit = nPgmTime;
	long atTargetTime = nPgmTime;
	long expTime = nPgmTime + expiryms;
	float errorThreshold = 2 * TICKS_PER_INCHES; //tolerance of 0.2 inches

	int initDriveL = _getLeftEnc();
	int initDriveR = _getRightEnc();

	writeDebugStreamLine("starting");
	float initAngle = GyroGetAngle();
	while(!targetReached){
		if(timeInit + 400 > nPgmTime){
			mec.master.slewRate = 400;
		} else{
			mec.master.slewRate = 1270;
		}

		//left encoder is master
		float error = setPoint - (( _getLeftEnc() - initDriveL) + (_getRightEnc() - initDriveR)) / 2;
		float slaveErr = ((_getLeftEnc() - initDriveL) / turnRatio) - ((_getRightEnc()- initDriveR) * turnRatio);

		writeDebugStreamLine("%f",slaveErr);


		if(sgn(mec.master.error) != sgn(mec.master.errorSum)){
			mec.master.errorSum = 0;
		}
		if(abs(mec.master.errorSum) > integralLimit){
			mec.master.errorSum = mec.master.errorSum > 0 ?  integralLimit : -integralLimit;
		}
		float driveOut = pidExecute(mec.master,error);
		float slaveOut = pidExecute(mec.slave, slaveErr);

		//we need to add a special case in case driveOut is saturated
		if(turnRatio > 1){
			if(abs(driveOut * turnRatio) > maxSpeed){
				driveOut = maxSpeed * (driveOut/abs(driveOut));
			}
		} else{
			if(abs(driveOut / turnRatio) > maxSpeed){
				driveOut = maxSpeed * (driveOut/abs(driveOut));
			}
		}

		float diffRatio = 1;
		float leftPow = driveOut / turnRatio  - slaveOut;
		float rightPow = driveOut * turnRatio + slaveOut ;
		if(abs(leftPow) > 127 || abs(rightPow) > 127){
			//get ratio of right to left pow
			diffRatio =  rightPow / leftPow;

			if(leftPow > rightPow){
				leftPow = leftPow > 0 ? maxSpeed : -maxSpeed;
				rightPow = leftPow * diffRatio;
			} else{
				rightPow = rightPow > 0 ? maxSpeed : -maxSpeed;
				leftPow = rightPow / diffRatio;

			}
		}

		_setLeftDrivePow(leftPow);
		_setRightDrivePow(rightPow);

		if(abs(error) > errorThreshold){
			atTargetTime = nPgmTime;
		}
		if(nPgmTime - atTargetTime > DRIVE_SAT_TIME){
			targetReached = true;
			_setLeftDrivePow(0);
			_setRightDrivePow(0);
			break;
		}
		if(nPgmTime >= expTime){
			writeDebugStreamLine("TARGET NOT REACHED");
			break;
		}
		wait1Msec(20);
	}
	mec.pidEnabled = true;
}

void mec_driveInches(float inches){
	mec_driveInches(inches,110,99999);
}

//prints encoder values for left and right
void printDriveEncoders(){
	writeDebugStreamLine("LEFT ENCODER : %f RIGHT ENCODER: %f",
		SensorValue[mec.encLeft], SensorValue[mec.encRight]);
}

//prints encoder values for left and right
void printPIDDriveDebug(){
	writeDebugStreamLine("LEFT ENCODER : %f RIGHT ENCODERmecf",
		SensorValue[mec.encLeft], SensorValue[mec.encRight]);
	printPIDDebug(mec.master);
	writeDebugStreamLine("SLAVE")
	printPIDDebug(mec.slave);
}

void printGyroPIDDebug(){
	writeDebugStreamLine("LEFT ENCODER : %f RIGHT ENCODER: %f",
		SensorValue[mec.encLeft], SensorValue[mec.encRight]);
	printPIDDebug(mec.gyroPID);
}

float STRETCH_FACT = 0.7;
int getMappedVal(float x){
	float newX = x/127;
	return 127 * ((STRETCH_FACT * pow(newX,3) + (1-0.7) * newX));
}


void _mecDrive(){
	int x1 = vexRT[Ch4];
		int y1 = 0;
		int x2 = vexRT[Ch1];
		int y2 = vexRT[Ch2];

		x1 = threshold(x1,JOYSTICK_DEADZONE);
		y1 = threshold(y1,JOYSTICK_DEADZONE);
		x2 = threshold(x2,JOYSTICK_DEADZONE);
		y2 = threshold(y2,JOYSTICK_DEADZONE);


		int oldSign = x1;
		x1 = getMappedVal(x1);



			float leftOut = y2 + x1 + y1;
			float rightOut = y2 - x1 - y1;

			_setLeftDrivePow(leftOut);
			_setRightDrivePow(rightOut);
		}

		task _PIDmecDrive(){
			zeroDriveEncoders();
			mec.pidEnabled = true;
			bool running = true;
			while(running){
				while(mec.pidEnabled){
					_mecDrive();
					wait1Msec(20);
				}
				wait1Msec(50);
			}
		}



		void initMecDrive(DriveBase db){
			mec.fl = db.fl;
			mec.fr = db.fr;
			mec.ml = db.ml;
			mec.mr = db.mr;
			mec.bl = db.bl;
			mec.br = db.br;
			mec.encLeft = db.encLeft;
			mec.encRight = db.encRight;
			mec.master = db.master;
			mec.slave = db.slave;
			mec.gyro = db.gyro;
		}

		void mec_StartTeleop(){
			startTask(_PIDmecDrive, 9);
		}


		void mec_StopTeleop(){
			stopTask(_PIDmecDrive);
		}
#endif
