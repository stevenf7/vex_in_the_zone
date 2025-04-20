void pid{
float pidExecute(PID &pid, float error){
	pid.lastError = pid.error;
	pid.error = error;

	pid.dT = (nPgmTime - pid.timer)/1000; //delta time in seconds
	pid.timer = nPgmTime;

	delay(10);

	float rate;
	if(abs(pid.dT) > 0){
		rate = (pid.error - pid.lastError)/pid.dT;
	}
	else{
		rate = 0;
	}


	pid.output = error * pid.kP
		+ rate * pid.kD;

	if(abs(pid.output) < 127){
		pid.errorSum += error*pid.dT;
	}

	pid.output +=   pid.errorSum * pid.kI;

	return pidFilteredOutput(pid);
}
}
}
task main()
{



}
