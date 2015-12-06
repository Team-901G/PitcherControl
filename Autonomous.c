//Possible Todo:
//Movement commands given as array of motor indices, array of motor values, the limited factor, and the value to stop at (doUntil(motor array, value array, stopWhenThis, equalsThis))
//Manuevers should be in an array
//Movement should run separately from launcher PID

int loopDelay = 10;
int cycles = 0;

//Launcher Configuration Variables
float launchSpeed = 19.5;
float errorThreshold = 1;
int playedTone = 0;
pidController *left_fly_pid;
pidController *right_fly_pid;

//Drive Configuration Variables
typedef struct {
	int moveDelaySec;//time until cross-field manuever
	int moveDelayCyc;
	int firstRotationTicks;
	int forwardDriveTicks;
	int secondRotationTicks;
	int movementEncoder;
	int turnEncoder;
	int turnSpeed;
	int driveSpeed;
	bool completed;
} Maneuver;

Maneuver crossManeuver;

void runLCDSpeedInfo(float leftSpeedVal, float rightSpeedVal) {
	clearLCDLine(0);
	clearLCDLine(1);
	displayLCDString(0, 1, "L Spd ");
	displayLCDString(1, 1, "R Spd ");
	string leftSpeedStr;
	string rightSpeedStr;
	sprintf(leftSpeedStr,"%1.2f",leftSpeedVal);
	sprintf(rightSpeedStr,"%1.2f",rightSpeedVal);
	displayLCDString(0, 7, leftSpeedStr);
	displayLCDString(1, 7, rightSpeedStr);
}

void runLauncher() {
	/*float tone = 400+((10*(-SensorValue[RIGHT_FLY_QUAD_1])));
	int toneInt = (int)10*(round(tone/10));
	writeDebugStreamLine("tone value %d",tone);
	clearSounds();
	playImmediateTone(toneInt,2);*/

	PidControllerUpdate(left_fly_pid);
	PidControllerUpdate(right_fly_pid);

	motor[LEFT_FLY_MOTOR] =  left_fly_pid -> drive_cmd;
	motor[RIGHT_FLY_MOTOR] =  right_fly_pid -> drive_cmd;

	left_fly_pid -> target_value = launchSpeed;
	right_fly_pid -> target_value = launchSpeed;

	motor[LIFT_MOTOR] = 127;
	motor[INTAKE_MOTOR] = 127;
	SensorValue[INTAKE_SOLENOID] = 1;

	if(abs(left_fly_pid->error) < errorThreshold && abs(right_fly_pid->error) < errorThreshold) {
		if(playedTone == 0) {
			playTone(500,100);
		}
		playedTone = 1;
	}
	else if(abs(left_fly_pid->last_error) > errorThreshold && abs(right_fly_pid->last_error) > errorThreshold){
		clearSounds();
		playedTone = 0;
	}

	runLCDSpeedInfo(left_fly_pid->sensor_value,right_fly_pid->sensor_value);
}

void runDrive(int left, int right) {

	int frontRightMotorSpeed = right;
	int frontLeftMotorSpeed = left;
	int backRightMotorSpeed = right;
	int backLeftMotorSpeed = left;

	motor[FRONT_RIGHT_BOT_DRIVE_MOTOR] = frontRightMotorSpeed;
	motor[FRONT_RIGHT_TOP_DRIVE_MOTOR] = frontRightMotorSpeed;
	motor[BACK_RIGHT_DRIVE_MOTOR] = backRightMotorSpeed;
	motor[FRONT_LEFT_TOP_DRIVE_MOTOR] = frontLeftMotorSpeed;
	motor[FRONT_LEFT_BOT_DRIVE_MOTOR] = frontLeftMotorSpeed;
	motor[BACK_LEFT_BOT_DRIVE_MOTOR] = backLeftMotorSpeed;

}

void runManeuever (Maneuver *s) {
	SensorValue[s->turnEncoder] = 0;
	writeDebugStreamLine("Starting Maneuver");
	while(abs(SensorValue[s->turnEncoder]) < abs(s->firstRotationTicks)) {
		int sign = sgn(s->firstRotationTicks);
		runDrive(sign*s->turnSpeed,-1*sign*s->turnSpeed);
		writeDebugStreamLine("First Turn Maneuver");
	}
	runDrive(0,0);

	SensorValue[s->movementEncoder] = 0;

	while(abs(SensorValue[s->movementEncoder]) < abs(s->forwardDriveTicks)) {
		int sign = sgn(s->forwardDriveTicks);
		runDrive(sign*s->driveSpeed,sign*s->driveSpeed);
		writeDebugStreamLine("First Drive Maneuver");
	}
	runDrive(0,0);

	SensorValue[s->turnEncoder] = 0;

	while(abs(SensorValue[s->turnEncoder]) < abs(s->secondRotationTicks)) {
		int sign = sgn(s->secondRotationTicks);
		runDrive(sign*s->turnSpeed,-1*sign*s->turnSpeed);
		writeDebugStreamLine("Second Turn Maneuver");
	}
	runDrive(0,0);
	writeDebugStreamLine("Ending Maneuver");
	crossManeuver.completed = true;
}


void manueverUpdateLoop() {
	if (cycles > crossManeuver.moveDelayCyc && !crossManeuver.completed) {
		runManeuever(&crossManeuver);
	}
}

void AutonControlLoop() {

 	left_fly_pid = PidControllerInit( 0.32, 0.0011, 0.0, LEFT_FLY_QUAD_1, 0 );
 	right_fly_pid = PidControllerInit( 0.32, 0.011, 0.0, RIGHT_FLY_QUAD_1, 1 );

	crossManeuver.moveDelaySec=30000; // time until cross-field manuever
	crossManeuver.moveDelayCyc=300;
	crossManeuver.firstRotationTicks=388;
	crossManeuver.forwardDriveTicks=-3500;
	crossManeuver.secondRotationTicks=-388-900;
	crossManeuver.movementEncoder=FRONT_RIGHT_BOT_DRIVE_IME;
	crossManeuver.turnEncoder=GYRO;
	crossManeuver.turnSpeed=70;
	crossManeuver.driveSpeed=100;
	crossManeuver.completed = false;

	while(1) {

		runLauncher();
		//manueverUpdateLoop();
		cycles ++;
		wait1Msec(loopDelay);

	}
}

void AutonControlInit () {

	AutonControlLoop();

}
