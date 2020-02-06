package org.firstinspires.ftc.teamcode.Test.Auto_Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

@Autonomous(name = "Auto_Red_Depot", group= "Autonomous")
public class Auto_Red_Depot extends LinearOpMode {

	AutoHardwareGalileo robot = new AutoHardwareGalileo();   //Calls Upon Robot Definitions File

	private ElapsedTime runtime = new ElapsedTime(); //Sets timer for encoders

	double step = 1; //Sets the steps for the autonomous

	@Override
	public void runOpMode() {

		robot.init(hardwareMap); //Calls Upon Robot Initialization File

		composeTelemetry(); //Gyro Telemetry Initialization

		// Skystone detection initialization
		initVuforia(); //Vuforia Initialization

		if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
			initTfod(); //Tensor Flow Object Detection Initialization
		} else {
			telemetry.addData("Sorry!", "This device is not compatible with TFOD");
		}

		telemetry.addData("Drive Train: ", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Payload: ", "Initialized");          // Adds telemetry to the screen to show that the payload is initialized
		telemetry.addData("Status: ", "Ready");                 // Adds telemetry to the screen to show that the robot is ready
		telemetry.addData("Press Play to Start ", "TeleOp");    // Adds telemetry to the screen to tell the drivers that the code is ready to start
		telemetry.update();                                                   // Tells the telemetry to display on the phone
		waitForStart();

		while (opModeIsActive()) {

			if (step == 1) { //Move forward and scan the first block
				stepTelemetry(); //Display telemetry

				//Move the slide forward, and drop lift
				encoderSlide(1, 4);  // Move the slide forward 4 inches
				encoderLift(1, -1.4); // Drop the lift downward 1.4 inches

				//Open the grabber
				robot.grabStone.setPosition(.6); //Set the grabber to open position

				//Move Forward
				encoderDrive(0.2, 12, 10);  // Forward 12 Inches with 10 Sec timeout

				if (robot.tfod != null) {
					robot.tfod.activate();
				}

				sleep(1000);

				//Start Scanning
				robot.pos++; //Tells code that it is checking position 1
				scan(); //Scan for skystone

				step++; //Next Step
			}

			if (step == 2 && !robot.Skystone) { //If the first block wasn't the skystone, move to the second block and scan it
				stepTelemetry(); //Display Telemetry

				//Setting skystone position for later
				robot.pos++; //If we didn't see the skystone in position 1, move to next position

				//Strafe Left to next block
				robot.motorFrontLeft.setPower(-.6); //Set the motors to strafe left
				robot.motorFrontRight.setPower(.6); //Set the motors to strafe left
				robot.motorBackLeft.setPower(.6); //Set the motors to strafe left
				robot.motorBackRight.setPower(-.6); //Set the motors to strafe left
				sleep(500); //Wait 500 milliseconds
				robot.motorFrontLeft.setPower(0); //Stop all power to the motors
				robot.motorFrontRight.setPower(0); //Stop all power to the motors
				robot.motorBackLeft.setPower(0); //Stop all power to the motors
				robot.motorBackRight.setPower(0); //Stop all power to the motors

				//Scan for second skystone
				sleep(robot.scanTime); //Wait 2 seconds to give vuforia time to identify the skystone
				scan(); //Scan for Skystone

				step++; //Next Step
			}

			if (step == 3 && !robot.Skystone) { //If the first two blocks weren't the skystone, it must be the third. Move and grab it
				stepTelemetry(); //Display Telemetry

				//Setting skystone position for later
				robot.pos++; //If we didn't see the skystone, move to next position

				//Strafe Left to next block
				robot.motorFrontLeft.setPower(-.6); //Set the motors to strafe left
				robot.motorFrontRight.setPower(.6); //Set the motors to strafe left
				robot.motorBackLeft.setPower(.6); //Set the motors to strafe left
				robot.motorBackRight.setPower(-.6); //Set the motors to strafe left
				sleep(500); //Wait for 500 milliseconds
				robot.motorFrontLeft.setPower(0); //Stop all power to the motors
				robot.motorFrontRight.setPower(0); //Stop all power to the motors
				robot.motorBackLeft.setPower(0); //Stop all power to the motors
				robot.motorBackRight.setPower(0); //Stop all power to the motors

				//Set skystone as true, which also moves us on to the next step
				robot.Skystone = true; //If position 1 and 2 are not skystone, then it must be position 3

			}

			if (step > 1 && step < 4 && robot.Skystone) { //If skystone is true
				step = 4; //If the skystone is found, move on to grabbing
			}

			if (step == 4) { //Grabbing the first skystone
				stepTelemetry(); //Display Telemetry

				//Move forward to grab skystone
				encoderDrive(.2, 18, 10); //Moves forward 18 inches to the block

				//Grab skystone
				robot.grabStone.setPosition(0.0); //Grab the Skystone
				sleep(300); //Wait 300 milliseconds

				//Move the lift up
				encoderLift(1, 1.25); //Lift up the lift 1.25"
				sleep(300); //Wait 300 milliseconds

				step++; //Move to the next step
			}

			if (step == 5) { //Move backwards with the skystone
				stepTelemetry(); //Display Telemetry

				//Move backwards
				encoderDrive(.6, -7, 10); //Move backwards 7 inches

				step++; //Move to the next step
			}

			if (step == 6) { //Turn 90 degrees clockwise
				stepTelemetry(); //Display Telemetry

				//Turn Clockwise
				encoderTurn(.25, -100, 10); //Turn CW 100 Degrees

				step++; //Move to the next step
			}

			if (step == 7) { //Run across the line
				stepTelemetry(); //Display Telemetry

				//Set distances needed to be moved by each position
				if (robot.pos == 1) { //If the skystone was in position 1
					encoderDrive(1, 40, 10); //Run forward 40 inches at speed of 1
					step++; //Move to the next step
				}

				if (robot.pos == 2) { //If the skystone was in position 2
					encoderDrive(1, 44, 10); //Run forward 44 inches at speed of 1
					step++; //Move to the next step
				}

				if (robot.pos == 3) { //If the skystone was in position 3
					encoderDrive(1, 56, 10); //Run forward 56 inches at speed of 1
					step++; //Move to the next step
				}
			}

			if (step == 8) { //Drop off the first skystone
				stepTelemetry(); //Display telemetry

				//Place the skystone
				robot.grabStone.setPosition(0.6); //Release the skystone

				step++; //Move to next step
			}

			if (step == 9) { //Run back to the second skystone
				stepTelemetry(); //Display telemetry
				if (robot.pos == 1) { //If the skystone was in position 1
					encoderDrive(.6,-67,10); //Move backwards 67 inches to second skystone
					step++;
				}
				if (robot.pos == 2) { //If the skystone was in position 2
					encoderDrive(.6,-69,10); //Move backwards 69 inches to second skystone
					step++;
				}
				if (robot.pos == 3) { //If the skystone was in position 3
					//encoderDrive(.6,-75,10); //Use in case we want second skystone on pos 3
					step++; //Move to next step
				}
			}

			if (step == 10 && (robot.pos == 1 || robot.pos == 2)) { //Turn toward the second skystone
				stepTelemetry(); //Display telemetry

				//Drive the lift up
				encoderLift(1, -1.15); //Drop the lift 1.15"

				//Turn 90 degrees counterclockwise
				encoderTurn(.25, 100, 10); //Turn CCW 100 Degrees
				// gyroTurn(0.1, 0); //Use gyro to make sure we are at the right angle
				// gyroHold(0.1, 0, 0.5); //Hold the angle for .5 seconds

				step++; //Move to next step
			}

			if (step == 10 && robot.pos == 3) { //Move backwards to park
				stepTelemetry(); //Display telemetry

				//Move backwards
				encoderDrive(1, -16, 10); //Move Backwards 16 inches

				//Strafe Left to get out of the way
				robot.motorFrontLeft.setPower(-.4); //Set power to strafe left
				robot.motorFrontRight.setPower(.4); //Set power to strafe left
				robot.motorBackLeft.setPower(.4); //Set power to strafe left
				robot.motorBackRight.setPower(-.4); //Set power to strafe left
				sleep(500); //Wait 500 milliseconds

				step++; //Move to next step
			}

			if (step == 11 && robot.pos==3) { //Stop strafing and end position 3
				//Stop Strafing
				robot.motorFrontLeft.setPower(0); //Stop all power to the motors
				robot.motorFrontRight.setPower(0); //Stop all power to the motors
				robot.motorBackLeft.setPower(0); //Stop all power to the motors
				robot.motorBackRight.setPower(0); //Stop all power to the motors
				//End of position 3
			}

			if (step == 11 && (robot.pos == 1 || robot.pos == 2)) { //Grab the second skystone
				stepTelemetry(); //Display telemetry

				//Drive forward
				encoderDrive(.2, 16, 10); //Moves forward 16 inches to the block

				//Grab the skystone
				robot.grabStone.setPosition(0.0); //Grab the Skystone
				sleep(300); //Wait 300 milliseconds

				//Drive the lift up
				encoderLift(1, 1.25); //Lift up the lift 1.25"
				sleep(300); //Wait 300 milliseconds

				step++; //Move to next step
			}

			if (step == 12) { //Move backwards with skystone
				stepTelemetry(); //Display telemetry

				//Move backward
				encoderDrive(.6, -22, 10); //Move backwards 22 inches

				step++; //Move to next step
			}

			if (step == 13) { //Turn 90 degrees
				stepTelemetry(); //Display telemetry
				encoderTurn(.25, -100, 10); //Turn CW 100 Degrees
				step++; //Move to next step
			}

			if (step == 14) { //Start moving back across the line
				if (robot.pos == 1) { //If the skystone was in position 1
					encoderDrive(1, 67, 10); //Move forward across the line
					step++; // Move to next step
				}

				else if (robot.pos == 2) { //If the skystone was in position 2
					encoderDrive(1, 70, 10); //Move forward across the line
					step++; //Move to next step
				}
			}

			if (step == 15) { //Release the skystone
				stepTelemetry(); //Display Telemetry

				//Release the skystone
				robot.grabStone.setPosition(0.6); //release the grabber

				step++; //Move to next step
			}

			if (step == 16) { //Move backward to the line
				stepTelemetry(); //Display Telemetry

				//Move backward
				encoderDrive(1,-16,10); //Move backward 16 inches

				step++; //Move to next step
			}

			if (step == 17) { //Strafe left

				//Strafe Left to get out of the way
				robot.motorFrontLeft.setPower(-.4); //Set power to strafe left
				robot.motorFrontRight.setPower(.4); //Set power to strafe left
				robot.motorBackLeft.setPower(.4); //Set power to strafe left
				robot.motorBackRight.setPower(-.4); //Set power to strafe left
				sleep(750); //Wait 750 milliseconds

				step++; //move to next step
			}

			if (step == 18) { //Stop motors and end of position 1 and 2
				//Stop all motors
				robot.motorFrontLeft.setPower(0); //Stop all power to the motors
				robot.motorFrontRight.setPower(0); //Stop all power to the motors
				robot.motorBackLeft.setPower(0); //Stop all power to the motors
				robot.motorBackRight.setPower(0); //Stop all power to the motors
				//End of position 1 & 2
			}
		}
	}

	private void stepTelemetry(){
		telemetry.addData("Current step: ", step);
		telemetry.addData("Skystone Position: ", robot.pos);
		telemetry.update();
	}


	//Skystone Position Voids

	public void scan() {
		if (robot.tfod != null) {
			// getUpdatedRecognitions() will return null if no new information is available since
			// the last time that call was made.
			List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
			if (updatedRecognitions != null) {
				telemetry.addData("# Object Detected", updatedRecognitions.size());

				// step through the list of recognitions and display boundary info.
				int i = 0;
				for (Recognition recognition : updatedRecognitions) {
					telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
					telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
							recognition.getLeft(), recognition.getTop());
					telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
							recognition.getRight(), recognition.getBottom());
					if (recognition.getLabel().equals(AutoHardwareGalileo.LABEL_SECOND_ELEMENT)) {
						robot.Skystone = true;
					}
				}
				telemetry.update();
			}
		}
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

	//Initialization Voids

	// Skystone detection configuration
	private void initVuforia () {
		/*
		 * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
		 */
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

		parameters.vuforiaLicenseKey = AutoHardwareGalileo.VUFORIA_KEY;
		parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

		//  Instantiate the Vuforia engine
		robot.vuforia = ClassFactory.getInstance().createVuforia(parameters);

		// Loading trackables is not necessary for the TensorFlow Object Detection engine.
	}

	// Initialize the TensorFlow Object Detection engine
	private void initTfod () {
		int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
				"tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
		tfodParameters.minimumConfidence = 0.8;
		robot.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, robot.vuforia);
		robot.tfod.loadModelFromAsset(AutoHardwareGalileo.TFOD_MODEL_ASSET, AutoHardwareGalileo.LABEL_FIRST_ELEMENT, AutoHardwareGalileo.LABEL_SECOND_ELEMENT);
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