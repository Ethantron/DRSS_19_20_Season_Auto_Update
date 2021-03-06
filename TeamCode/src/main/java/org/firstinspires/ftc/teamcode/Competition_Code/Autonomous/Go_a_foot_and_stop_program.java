package org.firstinspires.ftc.teamcode.Competition_Code.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;

@Autonomous(name = "Go_a_foot_and_stop_program", group= "Autonomous")
public class Go_a_foot_and_stop_program extends LinearOpMode{

	AutoHardwareGalileo robot = new AutoHardwareGalileo();   //Calls Upon Robot Definitions File

	private ElapsedTime runtime = new ElapsedTime(); //Sets timer for encoders

	double step = 1; //Sets the steps for the autonomous

	@Override
	public void runOpMode(){

		robot.init(hardwareMap); //Calls Upon Robot Initialization File

		composeTelemetry(); //Gyro Telemetry Initialization

		telemetry.addData("Drive Train: ", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Payload: ", "Initialized");          // Adds telemetry to the screen to show that the payload is initialized
		telemetry.addData("Status: ", "Ready");                 // Adds telemetry to the screen to show that the robot is ready
		telemetry.addData("Press Play to Start ", "TeleOp");    // Adds telemetry to the screen to tell the drivers that the code is ready to start
		telemetry.update();                                                   // Tells the telemetry to display on the phone
		waitForStart();

		if (step == 1) {
			//Move the slide forward, and drop lift
			encoderSlide(1, 4); // Move the slide forward 4 inches
			encoderLift(1, -1.375); // Drop the lift downward 1.375 inches

			step++;
		}

		if (step == 2){
			stepTelemetry();
			encoderDrive(.5,12,10);
			telemetry.addData("Done","!");
			telemetry.update();
			step++;
		}
	}

	private void stepTelemetry(){
		telemetry.addData("Current step: ", step);
		telemetry.addData("Skystone Position: ", robot.pos);
		telemetry.update();
	}

	//Repeated Voids
	public void encoderDrive(double speed, double Inches, double timeoutS){

		//Create our target variables
		int newFrontLeftTarget;
		int newFrontRightTarget;
		int newBackLeftTarget;
		int newBackRightTarget;

		// Ensure that the opmode is still active
		if (opModeIsActive()) {

			// Math to calculate each target position for the motors
			newFrontLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int)(Inches * AutoHardwareGalileo.COUNTS_PER_INCH);
			newFrontRightTarget = robot.motorFrontRight.getCurrentPosition() + (int)(Inches * AutoHardwareGalileo.COUNTS_PER_INCH);
			newBackLeftTarget = robot.motorBackLeft.getCurrentPosition() + (int)(Inches * AutoHardwareGalileo.COUNTS_PER_INCH);
			newBackRightTarget = robot.motorBackRight.getCurrentPosition() + (int)(Inches * AutoHardwareGalileo.COUNTS_PER_INCH);

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

			// reset the timeout time and start motion.
			runtime.reset();
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
					(runtime.seconds() < timeoutS) &&
					(robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy() && robot.motorBackLeft.isBusy() && robot.motorBackRight.isBusy())) {

				// Display it for the driver.
				telemetry.addData("Motor Paths",  "Running at %7d : %7d : %7d : %7d", //Tells us where we are
						robot.motorFrontLeft.getCurrentPosition(), //Front Left Position
						robot.motorFrontRight.getCurrentPosition(), //Front Right Position
						robot.motorBackLeft.getCurrentPosition(), //Back Left Position
						robot.motorBackRight.getCurrentPosition()); //Back Right Position
				telemetry.update();
			}

			// Stop all motion;
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

	public void encoderLift(double liftSpeed, double Inches) {  // Creates a void that the code can run at any time, and creates two doubles: "liftSpeed" and "levels"
		int newLiftTarget;                                      // Creates the integer "newLiftTarget"

		if (opModeIsActive()) {     // Do the following after the start button has been pressed and until the stop button is pressed
			newLiftTarget = (robot.lift.getCurrentPosition() + (int) (Inches * AutoHardwareGalileo.COUNTS_PER_LIFT_INCH));

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

	public void encoderTurn(double speed, double angle, double timeoutS) {
		//Create our target variables
		int newFrontLeftTarget;
		int newFrontRightTarget;
		int newBackLeftTarget;
		int newBackRightTarget;

		//Create our new circumference variables
		double c = 60.35; //Circumference of arc created by robot wheels (Radius is 9.605")
		double ANGLE_RATIO = angle / 360; //Ratio of angle relative to entire circle
		double CIRCUMFERENCE_OF_ANGLE = c * ANGLE_RATIO; //Circumference of Angle
		int COUNTS_PER_DISTANCE = (int) ((CIRCUMFERENCE_OF_ANGLE * AutoHardwareGalileo.COUNTS_PER_INCH) * 1.305);

		// Ensure that the opmode is still active
		if (opModeIsActive()) {

			// Math to calculate each target position for the motors
			newFrontLeftTarget = robot.motorFrontLeft.getCurrentPosition() - COUNTS_PER_DISTANCE;
			newFrontRightTarget = robot.motorFrontRight.getCurrentPosition() + COUNTS_PER_DISTANCE;
			newBackLeftTarget = robot.motorBackLeft.getCurrentPosition() - COUNTS_PER_DISTANCE;
			newBackRightTarget = robot.motorBackRight.getCurrentPosition() + COUNTS_PER_DISTANCE;

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

			// reset the timeout time and start motion.
			runtime.reset();
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
					(runtime.seconds() < timeoutS) &&
					(robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy() && robot.motorBackLeft.isBusy() && robot.motorBackRight.isBusy())) {

				// Display it for the driver.
				telemetry.addData("FLM: Path2", "Running at %7d", //Tells us where we are
						robot.motorFrontLeft.getCurrentPosition()); //Front Left Position
				telemetry.addData("FRM: Path2", "Running at %7d", //Tells us where we are
						robot.motorFrontRight.getCurrentPosition()); //Front Right Position
				telemetry.addData("BLM: Path2", "Running at %7d", //Tells us where we are
						robot.motorBackLeft.getCurrentPosition()); //Back Left Position
				telemetry.addData("BRM: Path2", "Running at %7d", //Tells us where we are
						robot.motorBackRight.getCurrentPosition()); //Back Right Position
				telemetry.update();
			}

			// Stop all motion;
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

	public void gyroTurn (  double speed, double angle) {

		// keep looping while we are still active, and not on heading.
		while (opModeIsActive() && !onHeading(speed, angle, AutoHardwareGalileo.P_TURN_COEFF)) {
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
			onHeading(speed, angle, AutoHardwareGalileo.P_TURN_COEFF);
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

		if (Math.abs(error) <= AutoHardwareGalileo.HEADING_THRESHOLD) {
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
