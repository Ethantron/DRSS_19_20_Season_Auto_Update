package org.firstinspires.ftc.teamcode.Test.Auto_Tests.Pseudodometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Test.Auto_Tests.AutoHardwareGalileo;

@Autonomous (name = "coordinateSystemTest", group = "Autonomous")
public class coordinateSystemTest extends LinearOpMode{

	AutoHardwareGalileo robot = new AutoHardwareGalileo();   //Calls Upon Robot Definitions File

	double step = 1; //Sets the steps for the autonomous

	//Odometry Variables
	double currentAngle = 0;
	double currentPositionX = 0;
	double currentPositionY = 0;

	@Override
	public void runOpMode() throws InterruptedException{
		robot.init(hardwareMap); //Calls Upon Robot Initialization File

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
				step++;
			}

			if (step == 3) {
				//Move Forward
				encoderDrive(.3,8);

				//Set position
				currentPositionY += 8;

				//Grab the Skystone
				robot.grabStone.setPosition(0);
				sleep(500);

				//Move lift up
				encoderLift(1.25, 1);

				//Move back with the skystone
				encoderDrive(1,-10);
				currentPositionY -= 10;

				step++;
			}

			if (step == 4) {
				telemetry.addData("current Position X: ", currentPositionX);
				telemetry.addData("current Position Y: ", currentPositionY);
				telemetry.addData("Current Angle: ", currentAngle);
				telemetry.update();

				if (gamepad1.a) {
					step++;
				}
			}

			if (step == 5) {
				runToCoordinate(68,26,1,0,1);

				step++;
			}

			/*if (step == 4) {
				//Move Back with skystone
				encoderDrive(1, -4);

				//set position
				currentPositionY -= 4;

				step++;
			}

			if (step == 5) {
				//turn -90 degrees
				encoderTurn(1,-90);
				currentAngle = -90;

				//Move forward to the foundation
				encoderDrive(1,70);
				currentPositionX += 70;

				//Lift up the skystone
				encoderLift(1,1);

				//turn to face the foundation
				encoderTurn(1,90);
				currentAngle = 0;

				step++;

			}

			if (step == 6) {
				//Move forward toward the foundation
				encoderDrive(1,10);
				currentPositionY += 10;

				//Drop down foundation movers
				robot.foundationMoverR.setPosition(1);
				robot.foundationMoverL.setPosition(1);
				sleep(250);

				//Backup with the foundation
				encoderDrive(1,-10);
				currentPositionY -= 10;

				step++;
			}

			if (step == 7) {
				//Turn the foundation
				encoderTurn(1,-90);
				currentAngle = -90;

				//move the foundation into the buildzone
				encoderDrive(1,5);
				currentPositionX += 5;

				//release the foundation
				robot.foundationMoverR.setPosition(0);
				robot.foundationMoverL.setPosition(0);

				//release the skystone
				robot.grabStone.setPosition(1);
				sleep(250);
			}*/
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
}