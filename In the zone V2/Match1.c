#pragma config(Motor,  port1,           leftWheel,     tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           leftGoal,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           leftArm,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           wrist,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           leftWheel2,    tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           rightWheel2,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           claw,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           rightArm,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           rightGoal,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          rightWheel,    tmotorVex393_HBridge, openLoop)
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VEX EDR                      */
/*                                                                           */
/*---------------------------------------------------------------------------*/

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton()
{
  // Set bStopTasksBetweenModes to false if you want to keep user created tasks
  // running between Autonomous and Driver controlled modes. You will need to
  // manage all user created tasks if set to false.
  bStopTasksBetweenModes = true;

	// Set bDisplayCompetitionStatusOnLcd to false if you don't want the LCD
	// used by the competition include file, for example, you might want
	// to display your team name on the LCD in this function.
	// bDisplayCompetitionStatusOnLcd = false;

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task autonomous()
{
  // ..........................................................................
  // Insert user code here.
  // ..........................................................................

  // Remove this function call once you have "real" code.
  AutonomousCodePlaceholderForTesting();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task usercontrol()
{
  // User control code here, inside the loop

  while (true)
  {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    motor[leftWheel]  = -(vexRT[Ch3] + vexRT[Ch4])/2;
    motor[leftWheel2]  = -(vexRT[Ch3] + vexRT[Ch4])/2;
    motor[rightWheel] = -(vexRT[Ch3] - vexRT[Ch4])/2;
    motor[rightWheel2] = -(vexRT[Ch3] - vexRT[Ch4])/2;


    if(vexRT[Btn6U] == 1)    //Arm Forward
    {
        motor[leftArm] = 127;
        motor[rightArm] = 127;

    }
    else if(vexRT[Btn6D] == 1) //Arm Backward
    {
        motor[leftArm] = -127;
        motor[rightArm] = -127;

    }
    else //Arm Stopped
    {
        motor[leftArm] = 0;
        motor[rightArm] = 0;
    }

    if(vexRT[Btn5U] == 1) //Claw Grab
    {
        motor[claw] = 63;
    }
    else if(vexRT[Btn5D] == 1) //Claw Released
    {
        motor[claw] = -63;
    }
    else //Claw Idle
    {
        motor[claw] = 0;
    }

    if(vexRT[Btn7L] == 1) //Goal Up
    {
        motor[leftGoal] = 127;
        motor[rightGoal] = 127;
    }
    else if(vexRT[Btn7D] == 1) //Goal Down
    {
        motor[leftGoal] = -60;
        motor[rightGoal] = -60;
    }
    else //Goal Idle
    {
        motor[leftArm] = 0;
        motor[rightArm] = 0;
    }

    if(vexRT[Btn8L] == 1) //Wrist Up
    {
        motor[wrist] = 30;
    }
    else if(vexRT[Btn8D] == 1) //Wrist Down
    {
        motor[wrist] = -30;
    }
    else //Wrist Idle
    {
        motor[wrist] = 15;
    }
  }
}
