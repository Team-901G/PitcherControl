void UserControlLoop() {
 	pidController *left_fly_pid;
 	pidController *right_fly_pid;

 	//Speeds: Tile 19.5
 	//left_fly_pid = PidControllerInit( 0.017, 0.0011, 0.03, LEFT_FLY_IME, 0 );
 	//right_fly_pid = PidControllerInit( 0.017, 0.0011, 0.03, RIGHT_FLY_IME, 1 );
 	left_fly_pid = PidControllerInit( 0.32, 0.0011, 0.0, LEFT_FLY_QUAD_1, 0 );
 	right_fly_pid = PidControllerInit( 0.32, 0.011, 0.0, RIGHT_FLY_QUAD_1, 1 );


	float launchSpeed = 19.5;//19.60;
	int wasPressed = 0;
	float errorThreshold = 1;
	int playedTone = 0;

  float UBtnSpeed = 19.5;
  float DBtnSpeed = 16;
  float LBtnSpeed = 17.5;
  float RBtnSpeed = 17.5;

	while(1) {

		int frontRightMotorSpeed = ((vexRT[Ch3] + vexRT[Ch4]) + vexRT[Ch1]);
		int frontLeftMotorSpeed = ((vexRT[Ch3] - vexRT[Ch4]) - vexRT[Ch1]);
		int backRightMotorSpeed = ((vexRT[Ch3] - vexRT[Ch4]) + vexRT[Ch1]);
		int backLeftMotorSpeed = ((vexRT[Ch3] + vexRT[Ch4]) - vexRT[Ch1]);

		motor[FRONT_RIGHT_BOT_DRIVE_MOTOR] = frontRightMotorSpeed;
		motor[FRONT_RIGHT_TOP_DRIVE_MOTOR] = frontRightMotorSpeed;
		motor[BACK_RIGHT_DRIVE_MOTOR] = backRightMotorSpeed;
		motor[FRONT_LEFT_TOP_DRIVE_MOTOR] = frontLeftMotorSpeed;
		motor[FRONT_LEFT_BOT_DRIVE_MOTOR] = frontLeftMotorSpeed;
		motor[BACK_LEFT_BOT_DRIVE_MOTOR] = backLeftMotorSpeed;

		PidControllerUpdate(left_fly_pid);
		PidControllerUpdate(right_fly_pid);

		if(vexRT[Btn8D] == 1) {
			motor[LEFT_FLY_MOTOR] =  left_fly_pid -> drive_cmd;
			motor[RIGHT_FLY_MOTOR] =  right_fly_pid -> drive_cmd;

			left_fly_pid -> target_value = launchSpeed;
			right_fly_pid -> target_value = launchSpeed;

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
		}
		else {
			motor[LEFT_FLY_MOTOR] = 0;
			motor[RIGHT_FLY_MOTOR] = 0;
		}

		if (vexRT[Btn7U] == 1) {
			launchSpeed = UBtnSpeed;
		}

		else if (vexRT[Btn7D] == 1) {
			launchSpeed = DBtnSpeed;
		}

    else if (vexRT[Btn7L] == 1) {
       launchSpeed = LBtnSpeed;
    }

    else if (vexRT[Btn7R] == 1) {
       launchSpeed = RBtnSpeed;
    }

		if(vexRT[Btn8R] == 1 && wasPressed == 0)
		{
			wasPressed = 1;
			launchSpeed +=0.25;
			playedTone = 0;
		}
		else if (vexRT[Btn8L] == 1 && wasPressed == 0)
		{
			wasPressed = 1;
			launchSpeed -= 0.25;
			playedTone = 0;
		}
		else if (vexRT[Btn8L] == 0 && vexRT[Btn8R] == 0)
		{
			wasPressed = 0;
		}

		if(vexRT[Btn5U] == 1) {
			motor[LIFT_MOTOR] = 127;
		}

		else if(vexRT[Btn5D] == 1) {
			motor[LIFT_MOTOR] = -127;
		}

		else {
				motor[LIFT_MOTOR] = 0;
		}

		if(vexRT[Btn6U] == 1) {
			motor[INTAKE_MOTOR] = 127;
		}
		else {
			motor[INTAKE_MOTOR] = 0;
		}

		if(vexRT[Btn6D] == 1) {
			SensorValue[INTAKE_SOLENOID] = 1;
		}

		else {
			SensorValue[INTAKE_SOLENOID] = 0;
		}


		wait1Msec(10);

	}

}

void UserControlInit () {

	UserControlLoop();

}
