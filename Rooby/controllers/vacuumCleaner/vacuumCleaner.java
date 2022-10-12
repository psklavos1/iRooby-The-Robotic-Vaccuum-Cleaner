// File:          vaccumController.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
//  import com.cyberbotics.webots.controller.Motor;
import java.util.Random;

import com.cyberbotics.webots.controller.Accelerometer;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.Receiver;
import com.cyberbotics.webots.controller.TouchSensor;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class vacuumCleaner extends Robot {

	/* Touch Sensors */
	protected final int BUMPERS_NUMBER = 2;
	protected final int BUMPER_LEFT = 0;
	protected final int BUMPER_RIGHT = 1;
	protected TouchSensor[] bumpers = new TouchSensor[BUMPERS_NUMBER];
	protected final String[] bumperNames = { "bumper_left", "bumper_right" };

	/* Distance Sensors to the ground to spot cliffs */
	protected final int CLIFF_SENSORS_NUMBER = 4;
	protected final int CLIFF_SENSOR_LEFT = 0;
	protected final int CLIFF_SENSOR_FRONT_LEFT = 1;
	protected final int CLIFF_SENSOR_FRONT_RIGHT = 2;
	protected final int CLIFF_SENSOR_RIGHT = 3;
	protected DistanceSensor[] cliffSensors = new DistanceSensor[CLIFF_SENSORS_NUMBER];
	protected final String[] cliffSensorNames = { "cliff_left", "cliff_front_left", "cliff_front_right",
			"cliff_right" };

	/* Led Sensors */
	protected final int LEDS_NUMBER = 3;
	protected LED[] leds = new LED[3];
	protected final String[] ledNames = { "led_on", "led_play", "led_step" };

	/* Receiver */
	protected Receiver receiver;
	protected final String receiverName = "receiver";

	/* Motors */
	protected Motor leftMotor, rightMotor;
	/* Position Sensors (position of the Wheels) */
	protected PositionSensor left_position_sensor, right_position_sensor;

	/* Constants and wider range variables */
	protected final int MAX_SPEED = 16;
	protected final int ZERO_SPEED = 0;
	protected final int HALF_SPEED = 8;
	protected final int MIN_SPEED = 2;
	protected final double WHEEL_RADIUS = 0.031;
	protected final double AXLE_LENGTH = 0.271756;
	protected final double ENCODER_RESOLUTION = 507.9188;
	protected final static int timeStep = 35;
	protected final double PI = Math.PI;
	protected final int RAND_MAX = 100;
	protected final int CLIFF_THRESHOLD = 100;
	protected final double minTime = 0;
	/*
	 * Set max time so that each algorithm is executed for a maximum of two minutes
	 * in real time. That is highligted because the timer that decides when to
	 * change among algorithms is using the currentTimeMillis(); function, so time
	 * is calculated in real life, and not the simulation timing. For example for a
	 * max of 2 min per algo: if average simulation speed is 20, set maxTime 6 so
	 * that 6*20 = 120 sec
	 */
	protected final double maxTime = 22;
	protected static int count = 0;
	protected long start, end;
	protected double sec;
	private static Robot rooby;
	private int numOfCycles = 0;

	/* Constructor Function for vaccum */
	public vacuumCleaner() {
		receiver = getReceiver("receiver");
		receiver.enable(timeStep);
		leftMotor = getMotor("left wheel motor");
		rightMotor = getMotor("right wheel motor");
		leftMotor.setPosition(Double.POSITIVE_INFINITY);
		rightMotor.setPosition(Double.POSITIVE_INFINITY);
		leftMotor.setVelocity(0.0);
		rightMotor.setVelocity(0.0);
		left_position_sensor = getPositionSensor("left wheel sensor");
		right_position_sensor = getPositionSensor("right wheel sensor");
		left_position_sensor.enable(timeStep);
		right_position_sensor.enable(timeStep);
		/* initialize touch sensors */
		for (int i = 0; i < BUMPERS_NUMBER; i++) {
			bumpers[i] = getTouchSensor(bumperNames[i]);
			bumpers[i].enable(timeStep);
		}
		/* initialize LEDS */
		for (int i = 0; i < LEDS_NUMBER; i++) {
			leds[i] = getLED(ledNames[i]);
		}
		/* initialize distance sensors */
		for (int i = 0; i < CLIFF_SENSORS_NUMBER; i++) {
			cliffSensors[i] = getDistanceSensor(cliffSensorNames[i]);
			cliffSensors[i].enable(timeStep);
		}
	}

	/* Detect Obstacles */
	public boolean collisionLeft() {
		if (bumpers[BUMPER_LEFT] != null) {
			if ((bumpers[BUMPER_LEFT].getValue() != 0.0)) {
				System.out.println("Collison Left");
				return true;
			}
		}
		return false;
	}

	public boolean collisionRight() {
		if (bumpers[BUMPER_RIGHT] != null) {
			if ((bumpers[BUMPER_RIGHT].getValue() != 0.0)) {
				System.out.println("Collison Right");
				return true;
			}
		}
		return false;
	}

	public boolean cliffLeft() {
		if (cliffSensors[CLIFF_SENSOR_LEFT].getValue() < CLIFF_THRESHOLD) {
			System.out.println("Cliff Left");
			return true;
		}
		return false;
	}

	public boolean cliffRight() {
		if (cliffSensors[CLIFF_SENSOR_RIGHT].getValue() < CLIFF_THRESHOLD) {
			System.out.println("Cliff Right");
			return true;
		}
		return false;
	}

	public boolean checkFront() {
		return (cliffSensors[CLIFF_SENSOR_FRONT_LEFT].getValue() < CLIFF_THRESHOLD
				|| cliffSensors[CLIFF_SENSOR_FRONT_RIGHT].getValue() < CLIFF_THRESHOLD);
	}

	public boolean cliffFront() {
		if (checkFront()) {
			// Extra check for miscalculated cliff.
			goForward();
			doStep();
			if (checkFront()) {
				System.out.println("Cliff front");
				return true;
			}
		}
		return false;
	}

	/* Manage movement */
	public void goForward() {
		leftMotor.setVelocity(MAX_SPEED);
		rightMotor.setVelocity(MAX_SPEED);
	}

	public void goBackward() {
		leftMotor.setVelocity(-HALF_SPEED);
		rightMotor.setVelocity(-HALF_SPEED);
	}

	public void stop() {
		leftMotor.setVelocity(ZERO_SPEED);
		rightMotor.setVelocity(ZERO_SPEED);
	}

	public void turnSlightLeft() {
		leftMotor.setVelocity(MAX_SPEED);
		rightMotor.setVelocity(HALF_SPEED / 2);
	}

	public void turnLeft() {
		leftMotor.setVelocity(HALF_SPEED);
		rightMotor.setVelocity(-HALF_SPEED);
	}

	public void turnSlightRight() {
		leftMotor.setVelocity(HALF_SPEED / 2);
		rightMotor.setVelocity(MAX_SPEED);
	}

	public void turnRight() {
		leftMotor.setVelocity(-HALF_SPEED);
		rightMotor.setVelocity(HALF_SPEED);
	}

	public void turnAngle(double angle) {
		stop(); // stop to get initial position of wheels
		double l_offset = left_position_sensor.getValue();
		double r_offset = right_position_sensor.getValue();
		doStep();

		/* determine the direction if angle>0 turn left else right */
		int sign = (angle < 0.0) ? -1 : 1;
		/* Turn towards the right direction */
		leftMotor.setVelocity(sign * HALF_SPEED);
		rightMotor.setVelocity(-sign * HALF_SPEED);
		double orientation;

		/* gradually turn until the given angle is achived */
		do {
			double l = left_position_sensor.getValue() - l_offset;
			double r = right_position_sensor.getValue() - r_offset;
			double dl = l * WHEEL_RADIUS; /* Distance covered by left wheel */
			double dr = r * WHEEL_RADIUS; /* Distance covered by right wheel */
			orientation = sign * (dl - dr) / AXLE_LENGTH;
			doStep();
		} while (orientation < sign * angle);
		stop();
		doStep();
	}

	/* Helper Functions */
	public void doStep() {
		step(timeStep);
	}

	public void doStop() {
		stop();
		doStep();
	}

	/* Random double generator */
	public double randDouble() {
		Random random = new Random();
		return random.nextDouble();
	}

	public double randDoubleRange(double rangeMin, double rangeMax) {
		Random r = new Random();
		double randomValue = rangeMin + (rangeMax - rangeMin) * r.nextDouble();
		return randomValue;
	}

	/* Random int generator */
	public int randIntRange(int min, int max) {
		Random random = new Random();
		return random.nextInt(max - min) + min;
	}

	/*
	 * This function executes steps as many times as it is possible in the time
	 * given as argument.
	 */
	public void passive_wait(double sec) {
		double start_time = this.getTime();
		do {
			doStep();
		} while (start_time + sec > this.getTime());
	}

	public boolean hasObstacle() {
		return (collisionLeft() || collisionRight() || cliffFront() || cliffLeft() || cliffRight());
	}

	/*
	 * Algorithm for spiral movvement. This function turns left sided, so the right
	 * motor moves with max speed while the right motor with minimum at start. Along
	 * the way, gradually decrease the angle of turn as the left wheel velocity
	 * increases in respect to the left motor's speed which is constantly max.
	 */
	public void spiralAlgorithm() {

		leftMotor.setVelocity(MIN_SPEED);
		rightMotor.setVelocity(MAX_SPEED);
		double tourTime = 0.6; // The period of time that a specific velocity is used for the right wheel
		double increaseTime = tourTime / 100; // As the velocity increases the time for a specific velocity value neeeds
		// to be increased in order to maintain the vaccum on a steadily
		// increasing, controlled form.
		double minSpeed = MIN_SPEED + 3; // The speed of the left wheel
		double increaseSpeed = (MAX_SPEED - minSpeed) / 65; // The velocity is increased after a period of tourTime.

		start = System.currentTimeMillis();
		sec = randDoubleRange(minTime, maxTime);
		end = start + (int) (sec * 1000);

		/* As long as there is no obstacle and there is still time for execution */
		while (!hasObstacle() && System.currentTimeMillis() < end) {
			System.out.println(
					"========================================= Spiral =========================================");
			// current
			passive_wait(tourTime);

			// The speed cannot be increased mor than the max value.
			if ((minSpeed + increaseSpeed) <= MAX_SPEED)
				minSpeed += increaseSpeed;

			leftMotor.setVelocity(minSpeed);

			// setup for next iteration
			tourTime += increaseTime;
			increaseSpeed -= increaseSpeed / 65;
			increaseTime += increaseTime / 100;
		}
	}

	/* Algorithm to follow the left wall. */
	public void leftWallFolllow() {
		start = System.currentTimeMillis();
		sec = randDoubleRange(minTime, maxTime / 2);
		end = start + (int) (sec * 1000);
		while (System.currentTimeMillis() < end) {
			System.out.println(
					"========================================= Left Wall Follow =========================================");
			/* Move Forward */
			goForward();
			doStep();

			/* If there is an obstacle */
			if (hasObstacle()) {
				System.out.println("Obstacle");
				/* Go back just so there is no colission while turning. */
				goBackward();
				passive_wait(0.1);

				/*
				 * Turn right gradually until the obstacle is put to the left side of the vaccum
				 * and it no longer stops it from moving
				 */
				do {
					System.out.println("Turning Right");
					turnAngle(-PI / 10);
				} while (hasObstacle() && System.currentTimeMillis() < end);

				/* With no obstacle, limiting the movement, move the vaccum forward */
				goForward();
				passive_wait(0.5);
				double totalAngle = 0;

				/* Turn left until an obstacle is found to keep following. */
				while (!(hasObstacle()) && System.currentTimeMillis() < end && totalAngle < 2 * PI) {
					System.out.println("Turning Left");
					turnAngle(PI / 10);
					totalAngle += PI / 10;
					goForward();
					passive_wait(0.2);
				}
			}
		}
	}

	/* Algorithm to follow the right wall. Same as above */
	public void rightWallFollow() {
		start = System.currentTimeMillis();
		sec = randDoubleRange(minTime, maxTime / 2);
		end = start + (int) (sec * 1000);
		while (System.currentTimeMillis() < end) {
			System.out.println(
					"========================================= Right Wall Follow =========================================");
			goForward();
			doStep(); /* 1 */
			if (hasObstacle()) { /* 2 */
				System.out.println("Obstacle");
				goBackward();
				passive_wait(0.1);

				do {
					System.out.println("Turning Left");
					turnAngle(PI / 10);
				} while (hasObstacle() && System.currentTimeMillis() < end);

				goForward();
				passive_wait(0.5); /* 4 */
				double totalAngle = 0;

				while (!(hasObstacle()) && System.currentTimeMillis() < end && totalAngle < 2 * PI) {
					System.out.println("Turning Right");
					turnAngle(-PI / 10); /* 5 */
					goForward();
					passive_wait(0.2);
					totalAngle += PI / 10;
				}
			}
		}
	}

	/*
	 * This algorithm combines the two previous algorithms. Specifically, the vaccum
	 * does a spiral movement until it find an obstacle. Then the left Wall Follow
	 * algorithm is used to move along this obstacle.
	 */
	public void comboSpiralAndWallFollow() {
		spiralAlgorithm();

		// int rand = randInt(0, 2);
		// if (rand == 0) {
		// if (wf_count < 15)
		rightWallFollow();
		// }

		// else
		// rightWallFollow();
	}

	/*
	 * This algorithm executes a random movement. In Specific, when the vaccum hits
	 * an obstacle, It goes backwards in order to be able to turn, and then turns at
	 * the opposite side of the obstacle, with a random angle, getting away from it
	 */
	public void randomWalk() {
		start = System.currentTimeMillis();
		sec = randDoubleRange(minTime, maxTime);
		end = start + (int) (sec * 1000);
		while (System.currentTimeMillis() < end) {
			System.out.println(
					"========================================= Random Walk =========================================");
			if (collisionLeft() || cliffLeft()) {
				System.out.println("Obstacle Left");
				goBackward();
				passive_wait(0.5);
				turnAngle(-PI * randDouble());
			} else if (collisionRight() || cliffRight() || cliffFront()) {
				System.out.println("Obstacle Right");
				goBackward();
				passive_wait(0.5);
				turnAngle(PI * randDouble());
			} else {
				System.out.println("Go Forward");
				goForward();
				doStep();
			}
		}

	}

	// Used to unblock the vaccum at some difficult spots for the boustrofedon to
	// handle.
	boolean right = false;
	boolean left = false;

	/*
	 * This Algorithm makes an "S" type movement. Specifically, The vaccum goes
	 * forword until it hits an obstacle. Then it turns 90 degrees (left the odd
	 * times - right the even times), and goes forword for a little Then it turns 90
	 * deggress again going the other direction. After that it moves forward again
	 * until it hits an obstacle.
	 */
	public void boustrophedon() {
		start = System.currentTimeMillis();
		sec = randDoubleRange(minTime, maxTime);
		end = start + (int) (sec * 1000);
		while (System.currentTimeMillis() < end) {
			System.out.println(
					"========================================= Bustrophedon =========================================");

			/* If the vaccum faces an obstacle */
			if (hasObstacle()) {
				goBackward();
				passive_wait(0.2);
				doStop();
				// used to normalize the angle of the 90 degree turn
				// 0.1804 comes from PI/17.4.
				double norm = 0.18038;

				// if even we turn left
				if (count % 2 == 0) {
					left = true;
					if (right) {
						// if previous movement was turn left, then it means the vaccum is stuck(Useful
						// in some cases, not all of the stuck situatios)
						System.out.println("Stuck");
						break;
					}
					System.out.println("Turn Left");
					turnAngle(PI / 2 + norm);
					doStop();
					goForward();
					passive_wait(0.5);
					turnAngle(PI / 2 + norm);
					doStop();
					count++;
					// if odd we turn right
				} else {
					right = true;
					// if previous movement was turn right, then it means the vaccum is stuck
					if (left) {
						System.out.println("Stuck");
						break;
					}
					System.out.println("Turn Right");
					turnAngle(-PI / 2 - norm);
					doStop();
					goForward();
					passive_wait(0.5);
					turnAngle(-PI / 2 - norm);
					doStop();
					count++;
				}

			} else {
				// In case there is no bstacle the vaccum moves forward.
				left = right = false;
				System.out.println("Go Forward");
				goForward();
				doStep();
			}
		}

	}

	public void run() {
		while (numOfCycles < 40) {
			numOfCycles++;
			// Execution of "S" movement algorithm (bousrtrophedon)
			boustrophedon();
			right = false;
			left = false;
			doStop();

			// Execution of random walk movement algorithm
			randomWalk();
			doStop();

			// Execution of spiral movement algorithm along with left wall follow
			comboSpiralAndWallFollow();
			doStop();
			turnAngle(PI / 2 + 0.18038);
		}
	}

	// This is the main function of your controller.
	// It creates an instance of your Robot instance and
	// it uses its function(s).
	// Note that only one instance of Robot should be created in
	// a controller program.
	// The arguments of the main function can be specified by the
	// "controllerArgs" field of the Robot node
	public static void main(String[] args) {
		// create the Robot instance.
		vacuumCleaner rooby = new vacuumCleaner();
		System.out.println("Everything all right");

		// Main loop:
		// - perform simulation steps until Webots is stopping the controller
		while (rooby.step(timeStep) != -1) {
			rooby.run();
		}
		;

		// Enter here exit cleanup code.
	}
}
