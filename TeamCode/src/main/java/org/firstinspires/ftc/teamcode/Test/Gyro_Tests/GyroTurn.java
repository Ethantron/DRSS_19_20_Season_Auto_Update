package org.firstinspires.ftc.teamcode.Test.Gyro_Tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@Autonomous(name = "GyroTurn", group = "Misc_Test")
public class GyroTurn extends LinearOpMode {

	// Robot definitions
	// Defines the Motors
	public DcMotor motorFrontRight;
	public DcMotor motorFrontLeft;
	public DcMotor motorBackRight;
	public DcMotor motorBackLeft;

	// Encoder definitions
	private ElapsedTime runtime = new ElapsedTime();
	static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
	static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
	static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);


	// Defines the gyro
	BNO055IMU imu;

	// State used for updating telemetry
	Orientation angles;
	Acceleration gravity;

	//Turning Variables
	static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
	static final double     TURN_SPEED              = 0.2;     // Nominal half speed for better accuracy.

	static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
	static final double     P_TURN_COEFF            = 0.15;     // Larger is more responsive, but also less stable
	static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

	@Override
	public void runOpMode() {

		// Drivetrain initialization
		motorFrontLeft = hardwareMap.dcMotor.get("FL");
		motorFrontRight = hardwareMap.dcMotor.get("FR");
		motorBackLeft = hardwareMap.dcMotor.get("BL");
		motorBackRight = hardwareMap.dcMotor.get("BR");

		motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
		motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
		motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
		motorBackRight.setDirection(DcMotor.Direction.REVERSE);

		// Encoder initialization
		telemetry.addData("Status", "Resetting Encoders");
		telemetry.update();

		//Stops and Resets Encoders
		motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		//Tells Robots to Reset Encoders
		motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		//Set up IMU
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
		parameters.loggingEnabled = true;
		parameters.loggingTag = "IMU";
		parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

		// Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
		// on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
		// and named "imu".
		imu = hardwareMap.get(BNO055IMU.class, "imu");
		imu.initialize(parameters);

		// Set up our telemetry dashboard
		composeTelemetry();

		telemetry.addData("Status: ", "Initialized");
		telemetry.addData(">", "Press Play to start op mode");
		telemetry.update();

		waitForStart();

		gyroTurn( TURN_SPEED, -45.0);         // Turn  CCW to -45 Degrees
		gyroHold( TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second

		telemetry.addData("Turning ", "Done :)!");
		telemetry.update();

		encoderDrive(.6, 12, 10);

		sleep(5000);

	}

	public void encoderDrive(double speed, double Inches, double timeoutS) {

		//Create our target variables
		int newFrontLeftTarget;
		int newFrontRightTarget;
		int newBackLeftTarget;
		int newBackRightTarget;

		// Ensure that the opmode is still active
		if (opModeIsActive()) {

			// Math to calculate each target position for the motors
			newFrontLeftTarget = motorFrontLeft.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);
			newFrontRightTarget = motorFrontRight.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);
			newBackLeftTarget = motorBackLeft.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);
			newBackRightTarget = motorBackRight.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);

			//Set Target Positions to respective motors
			motorFrontLeft.setTargetPosition(newFrontLeftTarget);
			motorFrontRight.setTargetPosition(newFrontRightTarget);
			motorBackLeft.setTargetPosition(newBackLeftTarget);
			motorBackRight.setTargetPosition(newBackRightTarget);

			// Turn On RUN_TO_POSITION
			motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

			// reset the timeout time and start motion.
			runtime.reset();
			motorFrontLeft.setPower(speed);
			motorFrontRight.setPower(speed);
			motorBackLeft.setPower(speed);
			motorBackRight.setPower(speed);

			// keep looping while we are still active, and there is time left, and both motors are running.
			// Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
			// its target position, the motion will stop.  This is "safer" in the event that the robot will
			// always end the motion as soon as possible.
			// However, if you require that BOTH motors have finished their moves before the robot continues
			// onto the next step, use (isBusy() || isBusy()) in the loop test.
			while (opModeIsActive() &&
					(runtime.seconds() < timeoutS) &&
					(motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorBackLeft.isBusy() && motorBackRight.isBusy())) {

				// Display it for the driver.
				/*telemetry.addData("Motor Paths",  "Running at %7d : %7d : %7d : %7d", //Tells us where we are
						motorFrontLeft.getCurrentPosition(), //Front Left Position
						motorFrontRight.getCurrentPosition(), //Front Right Position
						motorBackLeft.getCurrentPosition(), //Back Left Position
						motorBackRight.getCurrentPosition()); //Back Right Position
				telemetry.update();*/
			}

			// Stop all motion;
			motorFrontLeft.setPower(0);
			motorFrontRight.setPower(0);
			motorBackLeft.setPower(0);
			motorBackRight.setPower(0);

			// Turn off RUN_TO_POSITION
			motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

			//  sleep(250);   // optional pause after each move
		}
	}

	public void gyroTurn (  double speed, double angle) {

		// keep looping while we are still active, and not on heading.
		while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
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
			onHeading(speed, angle, P_TURN_COEFF);
			telemetry.update();
		}

		// Stop all motion;
		motorFrontLeft.setPower(0);
		motorBackLeft.setPower(0);
		motorFrontRight.setPower(0);
		motorBackRight.setPower(0);
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

		if (Math.abs(error) <= HEADING_THRESHOLD) {
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
		motorFrontLeft.setPower(leftSpeed);
		motorBackLeft.setPower(leftSpeed);
		motorFrontRight.setPower(rightSpeed);
		motorBackRight.setPower(rightSpeed);

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
		robotError = targetAngle - angles.firstAngle;
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
			angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
			gravity  = imu.getGravity();
		}
		});
		telemetry.addLine()
				.addData("heading", new Func<String>() {
					@Override public String value() {
						return formatAngle(angles.angleUnit, angles.firstAngle);
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

