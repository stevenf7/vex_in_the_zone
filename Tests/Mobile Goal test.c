#pragma config(Sensor, in4,    MobilePot,      sensorPotentiometer)
#pragma config(Sensor, dgtl3,  Switch2,     sensorTouch)
#pragma config(Motor,  port1,           MobileGoalR,   tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port10,          MobileGoalL,   tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main()
{
while(true){


//Button Mode
if(vexRT[Btn7U] == 1)       	//If Button 6U is pressed...
		{
			while ((vexRT[Btn7U] == 1) && (SensorValue[Switch2]==0 )){
			motor[MobileGoalL] = 80;
				motor[MobileGoalR] = 80;
				//	motor[BL] = 50;
					//	motor[BR] = 50;// open

		}
	}
		//}
		else if(vexRT[Btn7D] == 1)  	//Else, if button 6D is pressed...
		{
			while ((vexRT[Btn7D] == 1)  && (SensorValue[MobilePot]< 2100)){
		motor[MobileGoalL] = -80;
				motor[MobileGoalR] = -80;
			//		motor[BL] = -50;
					//	motor[BR] = -50;		//...close the gripper.

		//}
	//}
}
}
		else                      		//Else (neither button is pressed)...
		{
		motor[MobileGoalL] = 0;
				motor[MobileGoalR] = 0;
			//		motor[BL] = 0;
				//		motor[BR] = 0;
			//
	//	}



		}

}




/*	if ((SensorValue [MobilePot] > 0)){
			motor[MobileGoalR]  = (vexRT[Ch2])*0.8;
			motor[MobileGoalL] = (vexRT[Ch2])*0.8;



		}
		else if ((SensorValue[MobilePot && (SensorValue (BumpTouch2)==0)] <100)){
			motor[MobileGoalR]  = (vexRT[Ch2])*0.8;
			motor[MobileGoalL] = (vexRT[Ch2])*0.8;


		}
		else
		{
			motor[MobileGoalR]  = 0;
			motor[MobileGoalL] = 0;

		}
*/

}
