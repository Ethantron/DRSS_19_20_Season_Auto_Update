package org.firstinspires.ftc.teamcode.Competition_Code.Autonomous;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

@Disabled
@Autonomous(name = "Auto_Scanning", group= "Autonomous")
public class Auto_Scanning extends LinearOpMode {

	// Robot definitions
	public DcMotor motorFrontRight;
	public DcMotor motorFrontLeft;
	public DcMotor motorBackRight;
	public DcMotor motorBackLeft;
	double step = 1;
	double pos = 0;
	int scanTime = 2000;

	public DcMotor lift;
	public DcMotor slide;

	public Servo grabStone;
	public Servo wrist;

	// Skystone detection definitions
	boolean Skystone = false;
	private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
	private static final String LABEL_FIRST_ELEMENT = "Stone";
	private static final String LABEL_SECOND_ELEMENT = "Skystone";
	private static final String VUFORIA_KEY = "ASHBoLr/////AAABmbIUXlLiiEgrjVbqu8Iavlg6iPFigYso/+BCZ9uMzyAZFoo9CIzpV818SAqrjzuygz3hCeLW/ImK3xMH7DalGMwavqetwXS9Jw4I+rff2naxgV7n+EtYFvdCkUJDHfHVq1A4mhxDHgrjWZEqnLmZk25ppnIizQ0Ozcq4h6UmrWndEVEz8eKcCgn+IuglCEoEswvNBRAaKm/TAlpxLRNC6jQkZdJUh/TGYT05g9YCZo4+1ugmx01jrPCyHQVPVoeXm6VebLIuP7sNPw7njYzmVi2ffV5bYc4vf5kc5l5JwhBdPqnxuMfDLnHWaCkAO1UlVWqy2eY7/4b6iUYI2yN16ZKswSzLMmMNtPBu7e9HhKxA";

	private VuforiaLocalizer vuforia;
	private TFObjectDetector tfod;

	// Encoder definitions
	private ElapsedTime runtime = new ElapsedTime();
	static final double     COUNTS_PER_MOTOR_REV    = 560 ;    // eg: REV 20:1 Motor Encoder
	static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
	static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

	// Defines the gyro
	BNO055IMU imu;

	// State used for updating telemetry
	Orientation angles;
	Acceleration gravity;

	//Turning Variables
	static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
	static final double     P_TURN_COEFF            = 0.15;     // Larger is more responsive, but also less stable

	//Lift positioning definitions
	static final double COUNTS_PER_LIFT_INCH = 55;  // Sets the double "COUNTS_PER_LEVEL" to 300    | Defines how long the lift needs to run to go up one level | About 55  counts per inch

	// Color sensor definitions
	ColorSensor color_sensor;
	ColorSensor color2;

	@Override
	public void runOpMode() {

		// Drivetrain initialization
		motorFrontLeft = hardwareMap.dcMotor.get("FL");
		motorFrontRight = hardwareMap.dcMotor.get("FR");
		motorBackLeft = hardwareMap.dcMotor.get("BL");
		motorBackRight = hardwareMap.dcMotor.get("BR");

		motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
		motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
		motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
		motorBackRight.setDirection(DcMotor.Direction.FORWARD);

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

		//Hand Initiailization
		grabStone = hardwareMap.servo.get("GS");

		wrist = hardwareMap.servo.get("W");
		wrist.setPosition(.4); // Center the wrist

		//Lift Initialization
		lift = hardwareMap.dcMotor.get("LT");
		lift.setDirection(DcMotor.Direction.FORWARD);

		//Slide Initialization
		slide = hardwareMap.dcMotor.get("SL");
		slide.setDirection(DcMotor.Direction.FORWARD);


		// Gyroscope initialization
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
		parameters.loggingEnabled = true;
		parameters.loggingTag = "IMU";
		parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
		imu = hardwareMap.get(BNO055IMU.class, "imu");
		imu.initialize(parameters);
		composeTelemetry();

		// Color sensor initialization
		color_sensor = hardwareMap.get(ColorSensor.class, "color_sensor");
		color2 = hardwareMap.get(ColorSensor.class, "color2");
		//  sensorDistance = hardwareMap.get(DistanceSensor.class, "color_sensor");
		float hsvValues[] = {0F, 0F, 0F};
		final float values[] = hsvValues;
		final double SCALE_FACTOR = 255;

		// Skystone detection initialization
		initVuforia();

		if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
			initTfod();
		} else {
			telemetry.addData("Sorry!", "This device is not compatible with TFOD");
		}

		if (tfod != null) {
			tfod.activate();
		}

		telemetry.addData("Drive Train: ", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Payload: ", "Initialized");          // Adds telemetry to the screen to show that the payload is initialized
		telemetry.addData("Status: ", "Ready");                 // Adds telemetry to the screen to show that the robot is ready
		telemetry.addData("Press Play to Start ", "TeleOp");    // Adds telemetry to the screen to tell the drivers that the code is ready to start
		telemetry.update();                                           // Tells the telemetry to display on the phone
		waitForStart();

		while (opModeIsActive()) {

			if (step == 1) { //Move forward and scan the first block
				stepTelemetry(); //Display telemetry

				//Open the grabber
				grabStone.setPosition(.6); //Set the grabber to open position

				//Move Forward
				encoderDrive(0.2, 12, 10);  // Forward 12 Inches with 10 Sec timeout

				//Move Slide Forward
				slide.setPower(1); //Move Slide Forward
				sleep(200); //Wait 200 Milliseconds
				slide.setPower(0); //Stop Moving Slide Forward

				//Start Scanning
				pos++; //Tells code that it is checking position 1
				scan(); //Scan for skystone

				step++; //Next Step
			}

			if (step == 2 && !Skystone) { //If the first block wasn't the skystone, move to the second block and scan it
				stepTelemetry(); //Display Telemetry

				//Setting skystone position for later
				pos++; //If we didn't see the skystone in position 1, move to next position

				//Strafe Left to next block
				motorFrontLeft.setPower(-.6); //Set the motors to strafe left
				motorFrontRight.setPower(.6); //Set the motors to strafe left
				motorBackLeft.setPower(.6); //Set the motors to strafe left
				motorBackRight.setPower(-.6); //Set the motors to strafe left
				sleep(500); //Wait 500 milliseconds
				motorFrontLeft.setPower(0); //Stop all power to the motors
				motorFrontRight.setPower(0); //Stop all power to the motors
				motorBackLeft.setPower(0); //Stop all power to the motors
				motorBackRight.setPower(0); //Stop all power to the motors

				//Scan for second skystone
				sleep(scanTime); //Wait 2 seconds to give vuforia time to identify the skystone
				scan(); //Scan for Skystone

				step++; //Next Step
			}

			if (step == 3 && !Skystone) { //If the first two blocks weren't the skystone, it must be the third. Move and grab it
				stepTelemetry(); //Display Telemetry

				//Setting skystone position for later
				pos++; //If we didn't see the skystone, move to next position

				//Strafe Left to next block
				motorFrontLeft.setPower(-.6); //Set the motors to strafe left
				motorFrontRight.setPower(.6); //Set the motors to strafe left
				motorBackLeft.setPower(.6); //Set the motors to strafe left
				motorBackRight.setPower(-.6); //Set the motors to strafe left
				sleep(500); //Wait for 500 milliseconds
				motorFrontLeft.setPower(0); //Stop all power to the motors
				motorFrontRight.setPower(0); //Stop all power to the motors
				motorBackLeft.setPower(0); //Stop all power to the motors
				motorBackRight.setPower(0); //Stop all power to the motors

				//Set skystone as true, which also moves us on to the next step
				Skystone = true; //If position 1 and 2 are not skystone, then it must be position 3

			}

			if (step > 1 && step < 4 && Skystone) { //If skystone is true
				step = 4; //If the skystone is found, move on to grabbing
			}

			if (step == 4) { //Grabbing the first skystone
				stepTelemetry(); //Display Telemetry

				//Move forward to grab skystone
				encoderDrive(.2, 18, 10); //Moves forward 18 inches to the block

				//Grab skystone
				grabStone.setPosition(0.0); //Grab the Skystone
				sleep(300); //Wait 300 milliseconds

				//Move the lift up
				encoderLift(1, 1.25); //Lift up the lift 1.25"
				sleep(300); //Wait 300 milliseconds

				step++; //Move to the next step
			}

			if (step == 5) { //Move backwards with the skystone
				stepTelemetry(); //Display Telemetry

				//Move backwards
				encoderDrive(.6, -18, 10); //Move backwards 18 inches

				step++; //Move to the next step
			}

			if (step == 6) { //Turn 90 degrees clockwise
				stepTelemetry(); //Display Telemetry

				//Turn Clockwise
				encoderTurn(.25, -90, 10); //Turn CW 90 Degrees

				step++; //Move to the next step
			}

			if (step == 7) { //Run across the line =
				stepTelemetry(); //Display Telemetry

				//Set distances needed to be moved by each position
				if (pos == 1) { //If the skystone was in position 1
					encoderDrive(1, 35, 10); //Run forward 35 inches at speed of 1
					step++; //Move to the next step
				}

				if (pos == 2) { //If the skystone was in position 2
					encoderDrive(1, 39, 10); //Run forward 39 inches at speed of 1
					step++; //Move to the next step
				}

				if (pos == 3) { //If the skystone was in position 3
					encoderDrive(1, 51, 10); //Run forward 51 inches at speed of 1
					step++; //Move to the next step
				}
			}

/*
			//Parking color sensor
			if (step == 7){ //Start moving
				stepTelemetry();
				motorFrontRight.setPower(.6);
				motorFrontLeft.setPower(.6);
				motorBackLeft.setPower(.6);
				motorBackRight.setPower(.6);
			}

			// Does it see the line?
			while (step == 7 && opModeIsActive()){
				Color.RGBToHSV((int) (color_sensor.red() * SCALE_FACTOR),
						(int) (color_sensor.green() * SCALE_FACTOR),
						(int) (color_sensor.blue() * SCALE_FACTOR),
						hsvValues);
				Color.RGBToHSV((int) (color2.red() * SCALE_FACTOR),
						(int) (color2.green() * SCALE_FACTOR),
						(int) (color2.blue() * SCALE_FACTOR),
						hsvValues);

				// Send the info back to driver station using telemetry function.
				telemetry.addData("Step: ", step);
				telemetry.addData("Hue", hsvValues[0]);
				telemetry.update();

				if (hsvValues[0] > 150 ){ // Checks if it is brighter than the mats
					step++;
				}
			}*/

			if (step == 8) { //Drop off the first skystone
				stepTelemetry(); //Display telemetry

				//Place the skystone
				grabStone.setPosition(0.6); //Release the skystone

				step++; //Move to next step
			}

			if (step == 9) { //Run back to the second skystone
				stepTelemetry(); //Display telemetry
				if (pos == 1) { //If the skystone was in position 1
					encoderDrive(.6,-59,10); //Move backwards 49 inches to second skystone
				}
				if (pos == 2) { //If the skystone was in position 2
					encoderDrive(.6,-63,10); //Move backwards 58 inches to second skystone
				}
				if (pos == 3) { //If the skystone was in position 3
					//encoderDrive(.6,-75,10); //Use in case we want second skystone on pos 3
					step++; //Move to next step
				}
			}

			if (step == 10 && (pos == 1 || pos == 2)) { //Turn toward the second skystone
				stepTelemetry(); //Display telemetry

				//Drive the lift up
				encoderLift(1, -1.25); //Drop the lift 1.25"

				//Turn 90 degrees counterclockwise
				encoderTurn(.25, 90, 10); //Turn CCW 90 Degrees
				gyroTurn(0.1, 0); //Use gyro to make sure we are at the right angle
				gyroHold(0.1, 0, 0.5); //Hold the angle for .5 seconds

				step++; //Move to next step
			}

			if (step == 10 && pos == 3) { //Move backwards to park
				stepTelemetry(); //Display telemetry

				//Move backwards
				encoderDrive(1, -6, 10); //Move Backwards 6 inches

				//Strafe Left to get out of the way
				motorFrontLeft.setPower(-.4); //Set power to strafe left
				motorFrontRight.setPower(.4); //Set power to strafe left
				motorBackLeft.setPower(.4); //Set power to strafe left
				motorBackRight.setPower(-.4); //Set power to strafe left
				sleep(300); //Wait 300 milliseconds

				step++; //Move to next step
			}

			if (step == 11 && pos==3) { //Stop strafing and end position 3
				//Stop Strafing
				motorFrontLeft.setPower(0); //Stop all power to the motors
				motorFrontRight.setPower(0); //Stop all power to the motors
				motorBackLeft.setPower(0); //Stop all power to the motors
				motorBackRight.setPower(0); //Stop all power to the motors
				//End of position 3
			}

			if (step == 11 && (pos == 1 || pos == 2)) { //Grab the second skystone
				stepTelemetry(); //Display telemetry

				//Drive forward
				encoderDrive(.2, 18, 10); //Moves forward 18 inches to the block

				//Grab the skystone
				grabStone.setPosition(0.0); //Grab the Skystone
				sleep(300); //Wait 300 milliseconds

				//Drive the lift up
				encoderLift(1, 1.25); //Lift up the lift 1.25"
				sleep(300); //Wait 300 milliseconds

				step++; //Move to next step
			}

			if (step == 12) { //Move backwards with skystone
				stepTelemetry(); //Display telemetry

				//Move backward
				encoderDrive(.6, -18, 10); //Move backwards 18 inches

				step++; //Move to next step
			}

			if (step == 13) { //Turn 90 degrees
				stepTelemetry(); //Display telemetry
				encoderTurn(.25, -90, 10); //Turn CW 90 Degrees
				step++; //Move to next step
			}

			if (step == 14) { //Start moving back across the line
				if (pos == 1) { //If the skystone was in position 1
					encoderDrive(1, 59, 10); //Move forward across the line
					step++; // Move to next step
				}

				else if (pos == 2) { //If the skystone was in position 2
					encoderDrive(1, 63, 10); //Move forward across the line
					step++; //Move to next step
				}
			}

			/*// Does it see the line?
			while (step == 15 && opModeIsActive()){
				Color.RGBToHSV((int) (color_sensor.red() * SCALE_FACTOR),
						(int) (color_sensor.green() * SCALE_FACTOR),
						(int) (color_sensor.blue() * SCALE_FACTOR),
						hsvValues);
				Color.RGBToHSV((int) (color2.red() * SCALE_FACTOR),
						(int) (color2.green() * SCALE_FACTOR),
						(int) (color2.blue() * SCALE_FACTOR),
						hsvValues);

				// Send the info back to driver station using telemetry function.
				telemetry.addData("Step: ", step);
				telemetry.addData("Hue", hsvValues[0]);
				telemetry.update();

				if (hsvValues[0] > 150 ){ // Checks if it is red or blue
					step++;
				}
			}*/

			if (step == 15) { //Release the skystone
				stepTelemetry(); //Display Telemetry

				//Release the skystone
				grabStone.setPosition(0.6); //release the grabber

				step++; //Move to next step
			}

			if (step == 16) { //Move backward to the line
				stepTelemetry(); //Display Telemetry

				//Move backward
				encoderDrive(1,-6,10); //Move backward 6 inches

				step++; //Move to next step
			}

			if (step == 17) { //Strafe left

				//Strafe Left to get out of the way
				motorFrontLeft.setPower(-.4); //Set power to strafe left
				motorFrontRight.setPower(.4); //Set power to strafe left
				motorBackLeft.setPower(.4); //Set power to strafe left
				motorBackRight.setPower(-.4); //Set power to strafe left
				sleep(300); //Wait 300 milliseconds

				step++; //move to next step
			}

			if (step == 18) { //Stop motors and end of position 1 and 2
				//Stop all motors
				motorFrontLeft.setPower(0); //Stop all power to the motors
				motorFrontRight.setPower(0); //Stop all power to the motors
				motorBackLeft.setPower(0); //Stop all power to the motors
				motorBackRight.setPower(0); //Stop all power to the motors
				//End of position 1 & 2
			}
		}
	}

	private void stepTelemetry(){
		telemetry.addData("Current step: ", step);
		telemetry.addData("Skystone Position: ", pos);
		telemetry.update();
	}


	//Skystone Position Voids

	@SuppressLint("DefaultLocale")
	public void scan() {
		if (tfod != null) {
			// getUpdatedRecognitions() will return null if no new information is available since
			// the last time that call was made.
			List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
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
					if (recognition.getLabel() == LABEL_SECOND_ELEMENT) {
						Skystone = true;
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
				telemetry.addData("Motor Paths",  "Running at %7d : %7d : %7d : %7d", //Tells us where we are
						motorFrontLeft.getCurrentPosition(), //Front Left Position
						motorFrontRight.getCurrentPosition(), //Front Right Position
						motorBackLeft.getCurrentPosition(), //Back Left Position
						motorBackRight.getCurrentPosition()); //Back Right Position
				telemetry.update();
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

	public void encoderLift(double liftSpeed, double Inches) {  // Creates a void that the code can run at any time, and creates two doubles: "liftSpeed" and "levels"
		int newLiftTarget;                                      // Creates the integer "newLiftTarget"

		if (opModeIsActive()) {     // Do the following after the start button has been pressed and until the stop button is pressed
			newLiftTarget = (lift.getCurrentPosition() + (int) (Inches * COUNTS_PER_LIFT_INCH));

			lift.setTargetPosition(newLiftTarget);

			lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

			lift.setPower(liftSpeed);

			while (opModeIsActive() && lift.isBusy()) {
				telemetry.addData("lift position", lift.getCurrentPosition());
				telemetry.update();
			}

			lift.setPower(0);
			lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}
	}

	public void encoderTurn(double speed, double angle, double timeoutS) {
		//Create our target variables
		int newFrontLeftTarget;
		int newFrontRightTarget;
		int newBackLeftTarget;
		int newBackRightTarget;

		//Create our new circumference variables
		double c = 60.35; //Circumference of arc created by robot wheels (Radius is 9.605)
		double ANGLE_RATIO = angle / 360; //Ratio of angle relative to entire circle
		double CIRCUMFERENCE_OF_ANGLE = c * ANGLE_RATIO; //Circumference of Angle
		int COUNTS_PER_DISTANCE = (int) ((CIRCUMFERENCE_OF_ANGLE * COUNTS_PER_INCH) * 1.305);

		// Ensure that the opmode is still active
		if (opModeIsActive()) {

			// Math to calculate each target position for the motors
			newFrontLeftTarget = motorFrontLeft.getCurrentPosition() - COUNTS_PER_DISTANCE;
			newFrontRightTarget = motorFrontRight.getCurrentPosition() + COUNTS_PER_DISTANCE;
			newBackLeftTarget = motorBackLeft.getCurrentPosition() - COUNTS_PER_DISTANCE;
			newBackRightTarget = (motorBackRight.getCurrentPosition() + COUNTS_PER_DISTANCE);

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
				telemetry.addData("FLM: Path2", "Running at %7d", //Tells us where we are
						motorFrontLeft.getCurrentPosition()); //Front Left Position
				telemetry.addData("FRM: Path2", "Running at %7d", //Tells us where we are
						motorFrontRight.getCurrentPosition()); //Front Right Position
				telemetry.addData("BLM: Path2", "Running at %7d", //Tells us where we are
						motorBackLeft.getCurrentPosition()); //Back Left Position
				telemetry.addData("BRM: Path2", "Running at %7d", //Tells us where we are
						motorBackRight.getCurrentPosition()); //Back Right Position
				telemetry.update();
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

	//Initialization Voids

	// Skystone detection configuration
	private void initVuforia () {
		/*
		 * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
		 */
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

		parameters.vuforiaLicenseKey = VUFORIA_KEY;
		parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

		//  Instantiate the Vuforia engine
		vuforia = ClassFactory.getInstance().createVuforia(parameters);

		// Loading trackables is not necessary for the TensorFlow Object Detection engine.
	}

	// Initialize the TensorFlow Object Detection engine
	private void initTfod () {
		int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
				"tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
		tfodParameters.minimumConfidence = 0.8;
		tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
		tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
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