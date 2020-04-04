package org.firstinspires.ftc.teamcode.Test.Auto_Tests.Pseudodometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Test.Auto_Tests.AutoHardwareGalileo;

import java.util.Locale;

@Autonomous (name = "coordinateSystemTest", group = "Autonomous")
public class coordinateSystemTest extends LinearOpMode{

	AutoHardwareGalileo robot = new AutoHardwareGalileo();   //Calls Upon Robot Definitions File

	private ElapsedTime runtime = new ElapsedTime(); //Sets timer for encoders

	double step = 1; //Sets the steps for the autonomous

	//Odometry Variables
	double currentAngle = 0;
	double currentPositionX = 0;
	double currentPositionY = 0;

	@Override
	public void runOpMode() throws InterruptedException{
		robot.init(hardwareMap); //Calls Upon Robot Initialization File

		composeTelemetry(); //Gyro Telemetry Initialization

		telemetry.addData("Drive Train: ", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Payload: ", "Initialized");          // Adds telemetry to the screen to show that the payload is initialized
		telemetry.addData("Status: ", "Ready");                 // Adds telemetry to the screen to show that the robot is ready
		telemetry.addData("Press Play to Start ", "Autonomous");    // Adds telemetry to the screen to tell the drivers that the code is ready to start
		telemetry.update();                                                   // Tells the telemetry to display on the phone
		waitForStart();

		while (opModeIsActive()){

			if (step == 1) {
				//Move the slide forward, and drop lift
				//encoderSlide(1, 4);  // Move the slide forward 4 inches
				//encoderLift(1, -1.4); // Drop the lift downward 1.4 inches

				//Open the grabber
				robot.grabStone.setPosition(1); //Set the grabber to open position

				//move forward a bit
				encoderDrive(1,2);

				//Set our new position
				currentPositionY += 2;

				step++;
			}

			if (step == 2) {
				//Run to position 3
				runToCoordinate(-13,18,1,0,1);

				//Adjust with gyro
				gyroTurn(.15,0);
				gyroHold(.15,0,.1);

				step++;
			}

			if (step == 3) {
				//Move Forward
				encoderDrive(.1,8);

				//Set position
				currentPositionY += 8;

				//Grab the Skystone
				robot.grabStone.setPosition(0);
				sleep(500);

				//Move lift up
				encoderLift(2, 1);

				//Move back with the skystone
				encoderDrive(1,-10);
				currentPositionY -= 10;

				step++;
			}

			if (step == 4) {
				//Go to the foundation
				runToCoordinate(68,20,1,0,1);

				//Adjust with gyro
				gyroTurn(.2,-1);
				gyroHold(.2,-1,.1);

				step++;
			}

			if (step == 5) {
				//Lift up the skystone to avoid the foundation
				encoderLift(1,5);

				//Move forward to the foundation
				encoderDrive(.5,7);
				encoderDrive(.1,4);

				//Close the foundation movers
				robot.foundationMoverL.setPosition(1);
				robot.foundationMoverR.setPosition(1);

				step++;
			}

			if (step == 6) {
				//Move the slide forward and drop the skystone
				encoderSlide(1,3.5);

				//Drop the skystone
				robot.grabStone.setPosition(1);
				sleep(250);

				//Move the slide back
				encoderSlide(1,-3.5);

				step++;
			}

			if (step == 7) {
				//Move back with the foundation
				encoderDrive(1,-15);

				//Turn the foundation into the buildzone
				encoderTurn(1,-180);
				currentAngle = -90;

				//Move foundation into the build zone
				encoderDrive(1,10);

				currentPositionX = 70;
				currentPositionY = 22;

				step++;
			}

			if (step == 8) {
				//Unhook the foundation
				robot.foundationMoverL.setPosition(0);
				robot.foundationMoverR.setPosition(0);

				//Move Back
				encoderDrive(1,-3);

				//Drop the lift
				encoderLift(1,-5);

				//Move back
				encoderDrive(1,-40);

				step++;
			}
		}

	}





		/** Coordinate System Voids **/

		/** Run to Position **/
		public void runToCoordinate(double wantedPositionX,double wantedPositionY, double moveSpeed, double wantedAngle,double turnSpeed) {

			//Base Lengths
			double a = wantedPositionX - currentPositionX; //Leg A
			double b = wantedPositionY - currentPositionY; //Leg B
			double distance = getDistanceCalc(a,b); //Leg C (Hypotenuse)

			//Angles
			double turnAngle = getAngleError(a,b,distance); //Find angle we need to turn to to get on heading

			//Turn to required angle
			encoderTurn(turnSpeed, turnAngle);

			//Move to wanted position
			encoderDrive(moveSpeed,distance);

			//Turn to wanted angle
			double turnWantedAngle = getWantedAngleError(wantedAngle);
			encoderTurn(turnSpeed,turnWantedAngle);
			currentAngle = wantedAngle;

			//Set our current position
			currentPositionX = wantedPositionX;
			currentPositionY = wantedPositionY;
		}

		/** Finding Hypotenuse and needed angles **/
		public double getDistanceCalc(double baseA, double baseB) {
			double c = (Math.pow(baseA,2)) + (Math.pow(baseB,2)); //a^2 + b^2
			double baseC = Math.sqrt(c); //Find the square root of c

			return baseC;
		}

		public double getAngleError(double baseA,double baseB,double baseC) {
			//Get angle
			double tanTheta = baseA / baseB; //Get the tangent of theta
			double theta = Math.atan(tanTheta); //Get the arc tangent of cosTheta
			double target = Math.toDegrees(theta); //Convert theta from Radians to Degrees
			double angleError = 0;

			double ZeroError = findZeroError();

			if (baseB < 0 && baseA < 0) { //quad 3 relative
				double finalTarget = 90-(target-90);

				//Find angle error
				angleError = finalTarget-currentAngle;

				currentAngle = -finalTarget;

				return -(angleError);
			}

			if (baseB < 0 && baseA > 0) { //quad 4 relative
				double finalTarget = -90-(target+90);

				//Find angle error
				angleError = finalTarget-currentAngle;

				currentAngle = finalTarget;

				return angleError;
			}

			if (baseB >= 0){
				//Find angle error
				angleError = -(target) + ZeroError;

				currentAngle = -target;

				return angleError;
			}

			return angleError;
		}

		public double getWantedAngleError (double wantedAngle) {
			double wantedAngleError = -(currentAngle - wantedAngle);

			if (wantedAngle == 0) {
				wantedAngleError = -(wantedAngle + currentAngle);

				return wantedAngleError;
			}

			return wantedAngleError;
		}

		public double findZeroError () {
			double ZeroError = -(currentAngle);

			return ZeroError;
		}

		/** Encoder Turn **/
		public void encoderTurn(double speed, double angle) {
			//Create our target variables
			int newFrontLeftTarget;
			int newFrontRightTarget;
			int newBackLeftTarget;
			int newBackRightTarget;

			double COUNTS_PER_ANGLE = 10.405;
			double COUNTS_PER_DISTANCE = (angle * COUNTS_PER_ANGLE);

			// Ensure that the opmode is still active
			if (opModeIsActive()) {

				// Math to calculate each target position for the motors
				newFrontLeftTarget = (int) (robot.motorFrontLeft.getCurrentPosition() - COUNTS_PER_DISTANCE);
				newFrontRightTarget = (int) (robot.motorFrontRight.getCurrentPosition() + COUNTS_PER_DISTANCE);
				newBackLeftTarget = (int) (robot.motorBackLeft.getCurrentPosition() - COUNTS_PER_DISTANCE);
				newBackRightTarget = (int) (robot.motorBackRight.getCurrentPosition() + COUNTS_PER_DISTANCE);

				//Set Target Positions to respective motors
				robot.motorFrontLeft.setTargetPosition(newFrontLeftTarget);
				robot.motorFrontRight.setTargetPosition(newFrontRightTarget);
				robot.motorBackLeft.setTargetPosition(newBackLeftTarget);
				robot.motorBackRight.setTargetPosition(newBackRightTarget);

				// Turn On RUN_TO_POSITION
				robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

				// start motion
				robot.motorFrontLeft.setPower(speed);
				robot.motorFrontRight.setPower(speed);
				robot.motorBackLeft.setPower(speed);
				robot.motorBackRight.setPower(speed);

				// keep looping while we are still active, and there is time left, and both motors are running.
				// Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
				// its target position, the motion will stop.  This is "safer" in the event that the robot will
				// always end the motion as soon as possible.
				// However, if you require that BOTH motors have finished their moves before the robot continues
				// onto the next step, use (isBusy() || isBusy()) in the loop test.
				while (opModeIsActive() &&
						(robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy() && robot.motorBackLeft.isBusy() && robot.motorBackRight.isBusy())) {

					// Display it for the driver.
					telemetry.addData("Going to angle: ", angle);
					telemetry.update();
				}

				// Stop all motion
				robot.motorFrontLeft.setPower(-speed);
				robot.motorFrontRight.setPower(-speed);
				robot.motorBackLeft.setPower(-speed);
				robot.motorBackRight.setPower(-speed);

				sleep(10);

				robot.motorFrontLeft.setPower(0);
				robot.motorFrontRight.setPower(0);
				robot.motorBackLeft.setPower(0);
				robot.motorBackRight.setPower(0);

				// Turn off RUN_TO_POSITION
				robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

				currentAngle = angle;

				//  sleep(250);   // optional pause after each move
			}
		}

		/** Encoder Move **/
		public void encoderDrive(double speed, double Inches){

			//Create our target variables
			int newFrontLeftTarget;
			int newFrontRightTarget;
			int newBackLeftTarget;
			int newBackRightTarget;

			// Ensure that the opmode is still active
			if (opModeIsActive()) {

				// Math to calculate each target position for the motors
				newFrontLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int)(Inches * org.firstinspires.ftc.teamcode.Competition_Code.Autonomous.AutoHardwareGalileo.COUNTS_PER_INCH);
				newFrontRightTarget = robot.motorFrontRight.getCurrentPosition() + (int)(Inches * org.firstinspires.ftc.teamcode.Competition_Code.Autonomous.AutoHardwareGalileo.COUNTS_PER_INCH);
				newBackLeftTarget = robot.motorBackLeft.getCurrentPosition() + (int)(Inches * org.firstinspires.ftc.teamcode.Competition_Code.Autonomous.AutoHardwareGalileo.COUNTS_PER_INCH);
				newBackRightTarget = robot.motorBackRight.getCurrentPosition() + (int)(Inches * org.firstinspires.ftc.teamcode.Competition_Code.Autonomous.AutoHardwareGalileo.COUNTS_PER_INCH);

				//Set Target Positions to respective motors
				robot.motorFrontLeft.setTargetPosition(newFrontLeftTarget);
				robot.motorFrontRight.setTargetPosition(newFrontRightTarget);
				robot.motorBackLeft.setTargetPosition(newBackLeftTarget);
				robot.motorBackRight.setTargetPosition(newBackRightTarget);

				// Turn On RUN_TO_POSITION
				robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

				// start motion
				robot.motorFrontLeft.setPower(speed);
				robot.motorFrontRight.setPower(speed);
				robot.motorBackLeft.setPower(speed);
				robot.motorBackRight.setPower(speed);

				// keep looping while we are still active, and there is time left, and both motors are running.
				// Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
				// its target position, the motion will stop.  This is "safer" in the event that the robot will
				// always end the motion as soon as possible.
				// However, if you require that BOTH motors have finished their moves before the robot continues
				// onto the next step, use (isBusy() || isBusy()) in the loop test.
				while (opModeIsActive() &&
						(robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy() && robot.motorBackLeft.isBusy() && robot.motorBackRight.isBusy())) {

					// Display it for the driver.
					telemetry.addData("Motor Paths",  "Running at %7d : %7d : %7d : %7d", //Tells us where we are
							robot.motorFrontLeft.getCurrentPosition(), //Front Left Position
							robot.motorFrontRight.getCurrentPosition(), //Front Right Position
							robot.motorBackLeft.getCurrentPosition(), //Back Left Position
							robot.motorBackRight.getCurrentPosition()); //Back Right Position
					telemetry.update();
				}

				// Stop all motion
				robot.motorFrontLeft.setPower(-speed);
				robot.motorFrontRight.setPower(-speed);
				robot.motorBackLeft.setPower(-speed);
				robot.motorBackRight.setPower(-speed);

				sleep(20);

				robot.motorFrontLeft.setPower(0);
				robot.motorFrontRight.setPower(0);
				robot.motorBackLeft.setPower(0);
				robot.motorBackRight.setPower(0);

				// Turn off RUN_TO_POSITION
				robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

				//  sleep(250);   // optional pause after each move
			}
		}






		/** Other Autonomous Voids **/

		public void encoderLift(double liftSpeed, double Inches) {  // Creates a void that the code can run at any time, and creates two doubles: "liftSpeed" and "levels"
			int newLiftTarget;                                      // Creates the integer "newLiftTarget"

			if (opModeIsActive()) {     // Do the following after the start button has been pressed and until the stop button is pressed
				newLiftTarget = (robot.lift.getCurrentPosition() + (int) (Inches * robot.COUNTS_PER_LIFT_INCH));

				robot.lift.setTargetPosition(newLiftTarget);

				robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

				robot.lift.setPower(liftSpeed);

				while (opModeIsActive() && robot.lift.isBusy()) {
					telemetry.addData("lift position", robot.lift.getCurrentPosition());
					telemetry.update();
				}

				robot.lift.setPower(0);
				robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			}
		}

	public void encoderSlide (double slideSpeed, double Inches){
		int newLiftTarget;                                      // Creates the integer "newLiftTarget"

		if (opModeIsActive()) {     // Do the following after the start button has been pressed and until the stop button is pressed
			newLiftTarget = (robot.slide.getCurrentPosition() + (int) (Inches * robot.COUNTS_PER_SLIDE_INCH));

			robot.slide.setTargetPosition(newLiftTarget);

			robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

			robot.slide.setPower(slideSpeed);

			while (opModeIsActive() && robot.slide.isBusy()) {
				telemetry.addData("lift position", robot.slide.getCurrentPosition());
				telemetry.update();
			}

			robot.slide.setPower(0);
			robot.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}
	}





	/** Gyro Turning Voids **/

	public void gyroTurn (  double speed, double angle) {

		// keep looping while we are still active, and not on heading.
		while (opModeIsActive() && !onHeading(speed, angle, robot.P_TURN_COEFF)) {
			// Update telemetry & Allow time for other processes to run.
			telemetry.update();
		}
	}

	/**
	 *  Method to obtain & hold a heading for a finite amount of time
	 *  Move will stop once the requested time has elapsed
	 *
	 * @param speed      Desired speed of turn.
	 * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
	 *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
	 *                   If a relative angle is required, add/subtract from current heading.
	 * @param holdTime   Length of time (in seconds) to hold the specified heading.
	 */
	public void gyroHold( double speed, double angle, double holdTime) {

		ElapsedTime holdTimer = new ElapsedTime();

		// keep looping while we have time remaining.
		holdTimer.reset();
		while (opModeIsActive() && (holdTimer.time() < holdTime)) {
			// Update telemetry & Allow time for other processes to run.
			onHeading(speed, angle, org.firstinspires.ftc.teamcode.Test.Auto_Tests.AutoHardwareGalileo.P_TURN_COEFF);
			telemetry.update();
		}

		// Stop all motion;
		robot.motorFrontLeft.setPower(0);
		robot.motorBackLeft.setPower(0);
		robot.motorFrontRight.setPower(0);
		robot.motorBackRight.setPower(0);
	}

	/**
	 * Perform one cycle of closed loop heading control.
	 *
	 * @param speed     Desired speed of turn.
	 * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
	 *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
	 *                  If a relative angle is required, add/subtract from current heading.
	 * @param PCoeff    Proportional Gain coefficient
	 * @return
	 */
	boolean onHeading(double speed, double angle, double PCoeff) {
		double   error ;
		double   steer ;
		boolean  onTarget = false ;
		double leftSpeed;
		double rightSpeed;

		// determine turn power based on +/- error
		error = getError(angle);

		if (Math.abs(error) <= robot.HEADING_THRESHOLD) {
			steer = 0.0;
			leftSpeed  = 0.0;
			rightSpeed = 0.0;
			onTarget = true;
		}
		else {
			steer = getSteer(error, PCoeff);
			rightSpeed  = speed * steer;
			leftSpeed   = -rightSpeed;
		}

		// Send desired speeds to motors.
		robot.motorFrontLeft.setPower(leftSpeed);
		robot.motorBackLeft.setPower(leftSpeed);
		robot.motorFrontRight.setPower(rightSpeed);
		robot.motorBackRight.setPower(rightSpeed);

		// Display it for the driver.
		telemetry.addData("Target", "%5.2f", angle);
		telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
		telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

		return onTarget;
	}

	/**
	 * getError determines the error between the target angle and the robot's current heading
	 * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
	 * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
	 *          +ve error means the robot should turn LEFT (CCW) to reduce error.
	 */
	public double getError(double targetAngle) {

		double robotError;

		// calculate error in -179 to +180 range  (
		robotError = targetAngle - robot.angles.firstAngle;
		while (robotError > 180)  robotError -= 360;
		while (robotError <= -180) robotError += 360;
		return robotError;
	}

	/**
	 * returns desired steering force.  +/- 1 range.  +ve = steer left
	 * @param error   Error angle in robot relative degrees
	 * @param PCoeff  Proportional Gain Coefficient
	 * @return
	 */
	public double getSteer(double error, double PCoeff) {
		return Range.clip(error * PCoeff, -1, 1);
	}





	/**Gyro Telemetry**/

	void composeTelemetry() {

		// At the beginning of each telemetry update, grab a bunch of data
		// from the IMU that we will then display in separate lines.
		telemetry.addAction(new Runnable() { @Override public void run()
		{
			// Acquiring the angles is relatively expensive; we don't want
			// to do that in each of the three items that need that info, as that's
			// three times the necessary expense.
			robot.angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
			robot.gravity  = robot.imu.getGravity();
		}
		});
		telemetry.addLine()
				.addData("heading", new Func<String>() {
					@Override public String value() {
						return formatAngle(robot.angles.angleUnit, robot.angles.firstAngle);
					}
				});
	}

	String formatAngle(AngleUnit angleUnit, double angle) {
		return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
	}

	String formatDegrees(double degrees){
		return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
	}
}