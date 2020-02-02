// Defines where the code is located
package org.firstinspires.ftc.teamcode.Test.TelyOp_Tests;

// Imports codes that the robot uses
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;        // Imports Linear Operation mode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;              // Imports Driver Controlled mode
import com.qualcomm.robotcore.hardware.DcMotor;                     // Imports motor definitions
import com.qualcomm.robotcore.hardware.Servo;                       // Imports servo definitions
import com.qualcomm.robotcore.util.ElapsedTime;                     // Imports timer definitions
import com.qualcomm.robotcore.util.Range;                           // Imports motor ranges definitions

// Defines robot display name
@Disabled
@TeleOp (name = "19-20 TeleOp", group= "TeleOp")                    // Sets codes mode to TelyOp and sets the display name for the code
public class TeleOp_19_20_Linear extends LinearOpMode {             // Sets the codes name and sets it to Linear OpMode

	/** Robot definitions **/

	// Timer definitions
	ElapsedTime ResetTime = new ElapsedTime();

	// Drive motor definitions
	public DcMotor motorFrontRight;                                 // Defines the front right motor
	public DcMotor motorFrontLeft;                                  // Defines the front left motor
	public DcMotor motorBackRight;                                  // Defines the back right motor
	public DcMotor motorBackLeft;                                   // Defines the back left motor

	// Mechanum Definitions
	double Frontright;                                              // Sets the double "Frontright"             | Helps with motor calculations
	double Frontleft;                                               // Sets the double "Frontleft"              | Helps with motor calculations
	double Backright;                                               // Sets the double "Backright"              | Helps with motor calculations
	double Backleft;                                                // Sets the double "Backleft                | Helps with motor calculations
	double Speed = 1;                                               // Sets the double "Speed" to one           | Controls overall speed of the drive motors
	double SpeedSetting = 1;                                        // Sets the double "SpeedSetting" to one    | Allows us to remember what the previous speed was

	// Payload motor and servo definitions
	public DcMotor lift;                                            // Defines the lift motor
	public DcMotor slide;                                           // Defines the slide motor
	public Servo grabStone;                                         // Defines the stone grabber servo
	public Servo wrist;                                             // Defines the wrist servo
	public Servo FoundationMoverL;                                  // Defines the left foundation servo
	public Servo FoundationMoverR;                                  // Defines the right foundation servo

	//Lift positioning definitions
	double LiftPower = 1;                                           // Sets the double "LiftPower" to one       | Defines how fast the lift moves
	double upstep = 0;                                              // Sets the double "upstep" to zero         |
	double upcount = 0;                                             // Sets the double "upcount" to zero        |
	double claw_status = 1;                                         // Sets the double "claw_status" to one     |
	double height = 0;                                              // Tells what level the lift will go to     |
	double CurrentHeight = 0;
	Boolean NeedFoundation = false;
	//Lift Encoder Definitions
	static final double     COUNTS_PER_LEVEL    = 300 ;    // About 55  counts per inch

	// End of Definitions

	@Override
	public void runOpMode() {

		/** Drive Train initialization **/

		//Motor Initialization
		motorFrontRight = hardwareMap.dcMotor.get("FR");
		motorFrontLeft = hardwareMap.dcMotor.get("FL");
		motorBackLeft = hardwareMap.dcMotor.get("BL");
		motorBackRight = hardwareMap.dcMotor.get("BR");

		//Motor Direction Initialization
		motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
		motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
		motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
		motorBackRight.setDirection(DcMotor.Direction.REVERSE);

		/** End of Drive Train Initialization **/

		/** Payload Initialization **/

		//Lift Initialization
		lift = hardwareMap.dcMotor.get("LT");
		lift.setDirection(DcMotor.Direction.FORWARD);

		//Slide Initialization
		slide = hardwareMap.dcMotor.get("SL");
		slide.setDirection(DcMotor.Direction.FORWARD);

		//Hand Initialization
		grabStone = hardwareMap.servo.get("GS");
		wrist = hardwareMap.servo.get("W");
		//wrist.setPosition(0.5); // Center the wrist

		//Foundation Mover Initialization
		FoundationMoverL = hardwareMap.servo.get("GL");
		FoundationMoverR = hardwareMap.servo.get("GR");

		// Encoder initialization
		telemetry.addData("Status", "Resetting Encoders");
		telemetry.update();

		//Stops and Resets Encoders
		lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		//Tells Robots to Reset Encoders
		lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		//Initialized Telemetry
		telemetry.addData("Drive Train: ", "Initialized");
		telemetry.addData("Payload: ", "Initialized");
		telemetry.addData("Status: ", "Ready");
		telemetry.addData("> Press Play to Start ", "TeleOp");
		telemetry.update();
		/** End of Payload Initialization **/

		waitForStart();

		while (opModeIsActive()) {

			/** Gamepad 1 Controls (Drive Train) ==> **/

			/**Mechanum Drive Controls**/
			// left stick controls direction
			// right stick X controls rotation
			float gamepad1LeftY = gamepad1.left_stick_y;
			float gamepad1LeftX = -gamepad1.left_stick_x;
			float gamepad1RightX = -gamepad1.right_stick_x;

			// Mechanum formulas
			double FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
			double FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
			double BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
			double BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

			// clip the right/left values so that the values never exceed +/- 1

			// write the values to the motors
			motorFrontRight.setPower(Frontright);
			motorFrontLeft.setPower(Frontleft);
			motorBackLeft.setPower(Backleft);
			motorBackRight.setPower(Backright);

			// sets speed
			Frontright = Range.clip(Math.pow(FrontRight, 3), -Speed, Speed);
			Frontleft = Range.clip(Math.pow(FrontLeft, 3), -Speed, Speed);
			Backright = Range.clip(Math.pow(BackRight, 3), -Speed, Speed);
			Backleft = Range.clip(Math.pow(BackLeft, 3), -Speed, Speed);

			//Speed Controls
			if (gamepad1.left_trigger > .3) { //While the left trigger is being held down
				Speed = .25; //Sets the speed to quarter speed
			} else {

				if (gamepad1.a || SpeedSetting == 1) {
					SpeedSetting = 1; // Tells the code that we are on full speed
					Speed = 1; // Full Speed
				}

				if (gamepad1.b || SpeedSetting == .75) {
					SpeedSetting = .75; // Tells the code that we are on three quarter speed
					Speed = .75; // Three Quarter Speed
				}

				if (gamepad1.x || SpeedSetting == .5) {
					SpeedSetting = .5; // Tells the code that we are on half speed
					Speed = .50; // Half Speed
				}

				if (gamepad1.y || SpeedSetting == .25) {
					SpeedSetting = .25; // Tells the code that we are on quarter speed
					Speed = .25; // Quarter Speed
				}
			}
			//End of Speed Controls
			/** End of Mechanum Drive Controls **/

			/** Foundation Mover Controls **/
			// Lowering Foundation Movers
			if (gamepad1.dpad_down) {
				FoundationMoverL.setPosition(1);
				FoundationMoverR.setPosition(1);
			}
			// End of Lowering Foundation Movers

			// Raising Foundation Movers
			if (gamepad1.dpad_up) {
				FoundationMoverL.setPosition(0);
				FoundationMoverR.setPosition(0);
			}
			// End of Raising Foundation Movers
			/** End of Foundation Mover Controls **/


			/** Gamepad 2 Controls (Payload) ==> **/

			/** Lift System Controls **/
			// Moving The Lift Upward
			if (gamepad2.left_bumper && (CurrentHeight > 0)) {
				height = -CurrentHeight;
				sleep(150);
				CurrentHeight = 0;
			}
			// End of Moving the Lift Upward

			// Zeroing the Lift
			if (!gamepad2.left_bumper && !gamepad2.right_bumper) {
				lift.setPower(.001); //Holds lift in place
			}
			// End of Zeroing the lift

			// Moving the Lift Downward
			if (gamepad2.right_bumper && (CurrentHeight < 7)) {
				if (CurrentHeight == 0) { //If we need to lift past the foundation
					NeedFoundation = true;
				}
				height++;
				CurrentHeight++;
				sleep(200);
			}

			//Start lift
			if (gamepad2.right_trigger > .3) {
				encoderLift(1,height);
			}

			//Foundation Override
			if (gamepad2.x) {
				NeedFoundation = false;
			}

			/** End of Lift System Controls **/

			/** Lift Speed Brake Controls **/
			if (gamepad2.left_trigger > .3 && gamepad2.left_stick_y >.3) { //While the left trigger is being held down
				lift.setPower(1);
			}

			if (gamepad2.left_trigger > .3 && (gamepad2.left_stick_y < .3 && gamepad2.left_stick_y > -.3)) { //While the left trigger is being held down
				lift.setPower(0.001);
			}

			if (gamepad2.left_trigger > .3 && gamepad2.left_stick_y < -.3) { //While the left trigger is being held down
				lift.setPower(-1);
			}
			/** End of Lift Speed Brake Controls **/

			/** Slide System Controls **/
			// Moving The Slide Outward
			if (gamepad2.left_stick_y > .25 && gamepad2.left_trigger < .3) {
				slide.setPower(-1); //Set power to the slide
			}
			// End of Moving the Slide Outward

			// Zeroing the Slide
			if ((gamepad2.left_stick_y < .25 && gamepad2.left_stick_y > -.25) && gamepad2.left_trigger <= .3) {
				slide.setPower(0); //Stops power to the slid
			}
			// End of Zeroing the Slide

			// Moving the Slide Inward
			if (gamepad2.left_stick_y < -.25 && gamepad2.left_trigger <= .3) {
				slide.setPower(1); //Set power to the slide
			}
			// End of Moving the Slide Inward
			/** End of Slide System Controls **/

			/** Hand System Controls **/
			// Wrist Controls
			if (gamepad2.right_stick_x > 0.1) {
				wrist.setPosition(wrist.getPosition() + .0025); //Move the wrist right
			}

			if (gamepad2.right_stick_x < -0.1) {
				wrist.setPosition(wrist.getPosition() - .0025); //Move the wrist Left
			}

			if (gamepad2.right_stick_x < -0.1 && gamepad2.right_stick_x > 0.1) {
				wrist.setPosition(wrist.getPosition()); //Set the wrist to the current position
			}

			if (gamepad2.dpad_up) {
				wrist.setPosition(0.4);
			}
			// End of Wrist controls

			// Grabber Controls
			if (gamepad2.a) {
				encoderPlace(1, 50);
			}

			if (gamepad2.y) {
				grabStone.setPosition(0); //Clamp down with the grabber
			}

			if (gamepad2.b) {
				grabStone.setPosition(.3); //Release the grabber
			}
			// End of Grabber Controls
			/** End of Hand System Control **/

			/** Beginning of Telemetry **/
			// Lift Telemetry
			telemetry.addData("Height Change:", height);
			telemetry.addData("Current Height: ",CurrentHeight);

			//Wrist Telemetry
			telemetry.addData("Speed", Speed);
			telemetry.addData("Wrist", wrist.getPosition());
			telemetry.update();

			/** End of Telemetry **/
		}
	}

	public void encoderLift(double LiftSpeed, double levels) {
		int newLiftTarget;

		if (opModeIsActive()) {

			newLiftTarget = (lift.getCurrentPosition() + (int) (levels * COUNTS_PER_LEVEL)) + 50;

			if (NeedFoundation) {
				newLiftTarget = (lift.getCurrentPosition() + (int) (levels * COUNTS_PER_LEVEL)) + 150;
				NeedFoundation = false;
			}

			lift.setTargetPosition(newLiftTarget);

			lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

			lift.setPower(LiftSpeed);

			while (opModeIsActive() && lift.isBusy()) {

				telemetry.addData("lift position", lift.getCurrentPosition());
				telemetry.update();

			}

			lift.setPower(0);

			lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

			height = 0;

		}
	}

	public void encoderPlace(double DropSpeed, double distance) {
		int newDropTarget;

		if (opModeIsActive()) {

			newDropTarget = lift.getCurrentPosition() - (int) (distance);

			lift.setTargetPosition(newDropTarget);

			lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

			lift.setPower(DropSpeed);

			while (opModeIsActive() && lift.isBusy()) {

				telemetry.addData("lift position", lift.getCurrentPosition());
				telemetry.update();

			}

			lift.setPower(0);

			grabStone.setPosition(.3);
			sleep(100);

			//Return the lift back up
			newDropTarget = lift.getCurrentPosition() + (int) (distance);

			lift.setTargetPosition(newDropTarget);

			lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

			lift.setPower(DropSpeed);

			while (opModeIsActive() && lift.isBusy()) {

				telemetry.addData("lift position", lift.getCurrentPosition());
				telemetry.update();

			}

			lift.setPower(0);

			lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		}
	}
}
