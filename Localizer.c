//TODO: update sensor function calls to ROBOTC

//custom data types

struct pile {
	int xPos;
	int yPos;
};

struct line {
	int slope;
	int yIntercept;
	bool isWall; //tape or wall
};
struct SensorData {
	int leftEncoder;
	int rightEncoder;
	int liftEncoder;
	int leftSonar;
	int rightSonar;
	int lineTracker1;
	int lineTracker2;
	int lineTracker3;
	float gyro;
};
#define PI 3.14

//external sensors
extern Ultrasonic leftSonar;
extern Ultrasonic rightSonar;
extern Gyro gyro;
extern Encoder liftEncoder;

//sensor constants
#define ULTRASONIC_NOISE 0.5
#define ENCODER_NOISE 0.5
#define LINE_TRACKER_NOISE 0.5
#define POTENTIOMETER_NOISE 0.5

//movement constants
#define MOVE_PER_TICK 0.03757f // (18/24 * 10 * PI) / 627.1
#define ROBOT_WIDTH 10.0f
#define DEGREES_TO_RADIANS 0.01745f

//map data - no grid map because that's too large
#define NUM_LINES 8
#define NUM_PILES 10
const struct cube piles[NUM_PILES];
const struct line lines[NUM_LINES];
#define MAP_SIZE 100

struct Particle START_POSITIONS[] = {
		{.x = 0, .y = 0, .heading = 0 },
		{.x = 0, .y = 0, .heading = 0},
		{.x = 0, .y = 0, .heading = 0},
		{.x = 0, .y = 0, .heading = 0}

};

//Particles
#define NUM_PARTICLES 10
struct Particle particles[NUM_PARTICLES];
struct SensorData sensorValues;
float moveDistance;


//filter parameters

//implementation
void initialize_filter() {
	for (int i = 0; i < NUM_PARTICLES; i++) {
		initialize_particle(&particles[i], &START_POSITIONS[ROBOT_START_POS]);
	}

}

void localizer(void* ignore) {
	printf("Started Localizer Task");
	initialize_filter();

	//encoderReset(liftEncoder);
	//imeReset(LEFT_MOTOR_IME);
	//imeReset(RIGHT_MOTOR_IME);

	while (1) {

		if(!isAutonomous()){
					taskDelete(NULL); //exit this task if not in autonomous
		}

		sensorValues = sense();
		moveDistance = calculateMovement(sensorValues.leftEncoder,sensorValues.rightEncoder);
		update_state(sensorValues.liftEncoder);
		update_filter(moveDistance, sensorValues.gyro);
		delay(30);
	}
}

//TODO: weight updating method and resampling method
void update_filter(float distance, float rotation) {

	//move Particles
	for (int i = 0; i< NUM_PARTICLES; i++) {
		move_particle(&particles[i], distance, rotation);
	}

	//update the weights
	for (int i = 0; i< NUM_PARTICLES; i++) {
		//mes_prob_particle(&particles[i], distance, rotation);
	}

	//resample Particles
	//Particles = resample_particles(Particles);
}

void update_state(int liftEncoderValue) {
	liftHeight += liftEncoderValue * cm_per_tick;
}

struct SensorData sense() {
	struct SensorData values;
	//imeGet(LEFT_MOTOR_IME, &values.leftEncoder);
	//imeGet(RIGHT_MOTOR_IME, &values.rightEncoder);
	values.gyro = -1 * gyroGet(gyro) * DEGREES_TO_RADIANS;
	values.leftSonar = ultrasonicGet(leftSonar);
	values.rightSonar = ultrasonicGet(rightSonar);
	values.liftEncoder = encoderGet(liftEncoder);

	encoderReset(liftEncoder);
	//imeReset(LEFT_MOTOR_IME);
	//imeReset(RIGHT_MOTOR_IME);
	gyroReset(gyro);
	return values;
}

void initialize_particle(struct Particle * particle, struct Particle * startPos) {
	particle->x = startPos->x;
	particle->y = startPos->y;
	particle->heading = startPos->heading;
	particle->weight = 1.0f/NUM_PARTICLES;
}

//simplified motion model assuming robot turns then moves
void move_particle(struct Particle * particle, float distance, float direction) {
	distance += gaussianNoise(0,distance/5);
	float newHeading = particle->heading + direction + gaussianNoise(0,direction/5);
	particle->x += sin(newHeading) * distance;
	particle->y += cos(newHeading) * distance;
	particle->heading = fmodf(newHeading, (2 * PI)); //keep within -2pi to 2pi
}

void mes_prob_particle(struct Particle * particle) {

}

float calculateMovement(int leftEncoderValue, int rightEncoderValue) {
	//TODO: more accurate movement model
	//struct Particle translation;
	float SL = -leftEncoderValue * MOVE_PER_TICK;
	float SR = rightEncoderValue * MOVE_PER_TICK;
	float meanS = (SL + SR) / 2;
	return meanS;
}

float gaussianNoise (int mu, int sigma) {
	//taken from wikiipedia
	//TODO: more efficient polar method
	static int haveSpareRandoms = 0;
	static double rand1, rand2;

	//use the other equation of box muller transformation
	if(haveSpareRandoms) {
		haveSpareRandoms = 0;
		return (sqrt(rand1) * sin(rand2) * sigma) + mu;
	}
	haveSpareRandoms = 1;
	rand1 = rand() / (double) RAND_MAX;
	//printf("RAND1: %f \n", rand1);
	//if(rand1 < 1e-6) rand1 = 1e-6;
	rand1 = -2 * log(rand1);
	//printf("Log RAND1: %f \n", rand1);
	rand2 = (rand() / ((double) RAND_MAX)) * 2 * PI;

	return (sqrt(rand1) * cos(rand2) * sigma) + mu;
}

struct Particle Localizer_getWeightedAverage(){
	struct Particle average;
	float avgX=0;
	float avgY=0;
	float avgT=0;
	for (int i = 0; i < NUM_PARTICLES; i++) {
		float weight = particles[i].weight;
		avgX += particles[i].x*weight;
		avgY += particles[i].y*weight;
		avgT += particles[i].heading*weight;
	}
	average.x = (int)avgX;
	average.y = (int)avgY;
	average.heading = fmodf(avgT,(2*PI));

	return average;
}

int Localizer_getLiftHeight() {
	return liftHeight;
}

#endif /* LOCALIZER_C_ */
