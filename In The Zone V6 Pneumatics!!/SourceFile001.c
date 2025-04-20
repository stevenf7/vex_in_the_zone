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

task main()
{
	while (true){
if (vexRT [Btn5U] ==1) {
	motor [TR] = motor[TL] = 120;
}
 else
 {
   motor [TR] = motor[TL] = 120;
}


}
}
