package org.firstinspires.ftc.teamcode.Test.Auto_Tests.Pseudodometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Test.Auto_Tests.AutoHardwareGalileo;

import java.util.Locale;

@TeleOp (name = "EncoderTurnCalculation", group = "Autonomous")
public class EncoderTurnCalculation extends LinearOpMode{

	AutoHardwareGalileo robot = new AutoHardwareGalileo();   //Calls Upon Robot Definitions File

	private ElapsedTime runtime = new ElapsedTime(); //Sets timer for encoders

	double step = 1; //Sets the steps for the autonomous

	//Encoder Reading Variables
	double motorFrontLeftEncValue = 0;
	double motorFrontRightEncValue = 0;
	double motorBackLeftEncValue = 0;
	double motorBackRightEncValue = 0;
	double motorEncValuesAvg = 0;

	double encReading10 = 0;
	double encReading15 = 0;
	double encReading30 = 0;
	double encReading45 = 0;
	double encReading90 = 0;
	double encReading90Avg = 0;
	double calculatedEncReading = 0;

	@Override
	public void runOpMode() throws InterruptedException{
		robot.init(hardwareMap); //Calls Upon Robot Initialization File

		composeTelemetry(); //Gyro Telemetry Initialization

		telemetry.addData("Drive Train: ", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Payload: ", "Initialized");          // Adds telemetry to the screen to show that the payload is initialized
		telemetry.addData("Status: ", "Ready");                 // Adds telemetry to the screen to show that the robot is ready
		telemetry.addData("Press Play to Start ", "TeleOp");    // Adds telemetry to the screen to tell the drivers that the code is ready to start
		telemetry.update();                                                   // Tells the telemetry to display on the phone
		waitForStart();

		while (opModeIsActive()) {
			if (step == 1) { //Stop and Reset Encoders
				stopAndResetEncoder();
				step++;
			}

			if (step == 2) { //Turn 10 Degrees
				gyroTurn(.1,10);
				gyroHold(.1,10,1);

				motorFrontLeftEncValue = robot.motorFrontLeft.getCurrentPosition();
				motorFrontRightEncValue = robot.motorFrontRight.getCurrentPosition();
				motorBackLeftEncValue = robot.motorBackLeft.getCurrentPosition();
				motorBackRightEncValue = robot.motorBackRight.getCurrentPosition();

				motorEncValuesAvg = (motorBackLeftEncValue + motorBackRightEncValue + motorFrontLeftEncValue + motorFrontRightEncValue) / 4;
				encReading10 = motorEncValuesAvg * 9;

				stopAndResetEncoder();

				step++;
			}

			if (step == 3) { //Turn 15 Degrees
				gyroTurn(.1,25);
				gyroHold(.1,25,1);

				motorFrontLeftEncValue = robot.motorFrontLeft.getCurrentPosition();
				motorFrontRightEncValue = robot.motorFrontRight.getCurrentPosition();
				motorBackLeftEncValue = robot.motorBackLeft.getCurrentPosition();
				motorBackRightEncValue = robot.motorBackRight.getCurrentPosition();

				motorEncValuesAvg = (motorBackLeftEncValue + motorBackRightEncValue + motorFrontLeftEncValue + motorFrontRightEncValue) / 4;
				encReading15 = motorEncValuesAvg * 6;

				stopAndResetEncoder();

				step++;
			}

			if (step == 4) { //Turn 30 Degrees
				gyroTurn(.1,55);
				gyroHold(.1,55,1);

				motorFrontLeftEncValue = robot.motorFrontLeft.getCurrentPosition();
				motorFrontRightEncValue = robot.motorFrontRight.getCurrentPosition();
				motorBackLeftEncValue = robot.motorBackLeft.getCurrentPosition();
				motorBackRightEncValue = robot.motorBackRight.getCurrentPosition();

				motorEncValuesAvg = (motorBackLeftEncValue + motorBackRightEncValue + motorFrontLeftEncValue + motorFrontRightEncValue) / 4;
				encReading30 = motorEncValuesAvg * 3;

				stopAndResetEncoder();

				step++;
			}

			if (step == 4) { //Turn 45 Degrees
				gyroTurn(.1,100);
				gyroHold(.1,100,1);

				motorFrontLeftEncValue = robot.motorFrontLeft.getCurrentPosition();
				motorFrontRightEncValue = robot.motorFrontRight.getCurrentPosition();
				motorBackLeftEncValue = robot.motorBackLeft.getCurrentPosition();
				motorBackRightEncValue = robot.motorBackRight.getCurrentPosition();

				motorEncValuesAvg = (motorBackLeftEncValue + motorBackRightEncValue + motorFrontLeftEncValue + motorFrontRightEncValue) / 4;
				encReading45 = motorEncValuesAvg * 2;

				stopAndResetEncoder();

				step++;
			}

			if (step == 6) { //Turn back to 0 Degrees
				gyroTurn(.1,0);
				gyroHold(.1,0,1);

				stopAndResetEncoder();

				step++;
			}

			if (step == 6) { //Turn 90 Degrees
				gyroTurn(.1,90);
				gyroHold(.1,90,1);

				motorFrontLeftEncValue = robot.motorFrontLeft.getCurrentPosition();
				motorFrontRightEncValue = robot.motorFrontRight.getCurrentPosition();
				motorBackLeftEncValue = robot.motorBackLeft.getCurrentPosition();
				motorBackRightEncValue = robot.motorBackRight.getCurrentPosition();

				motorEncValuesAvg = (motorBackLeftEncValue + motorBackRightEncValue + motorFrontLeftEncValue + motorFrontRightEncValue) / 4;
				encReading90 = motorEncValuesAvg;

				stopAndResetEncoder();

				step++;
			}

			if (step == 7) { //Calculate Average Encoder Value
				encReading90Avg = (encReading10 + encReading15 + encReading30 + encReading45 + encReading90) / 5;
				calculatedEncReading = encReading90Avg / 90;

				telemetry.addData("10 Degrees", encReading10);
				telemetry.addData("15 Degrees", encReading15);
				telemetry.addData("30 Degrees", encReading30);
				telemetry.addData("45 Degrees", encReading45);
				telemetry.addData("90 Degrees", encReading90);
				telemetry.addData("90 Degree Avg Enc Value", encReading90Avg);
				telemetry.addData("1 Degree Avg Enc Value", calculatedEncReading);
				telemetry.update();
			}
		}
	}





	/** Gyro Turning **/

	public void gyroTurn (  double speed, double angle) {

		// keep looping while we are still active, and not on heading.
		while (opModeIsActive() && !onHeading(speed, angle, org.firstinspires.ftc.teamcode.Test.Auto_Tests.AutoHardwareGalileo.P_TURN_COEFF)) {
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

		if (Math.abs(error) <= org.firstinspires.ftc.teamcode.Test.Auto_Tests.AutoHardwareGalileo.HEADING_THRESHOLD) {
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

	public void stopAndResetEncoder() {
		//Stop and Reset Encoders
		robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		//Run Using Encoders
		robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
