package org.firstinspires.ftc.teamcode.Test.TelyOp_Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Button Test", group= "TeleOp Test")
public class Button_Test extends OpMode {

	// Motor Definitions
	public DcMotor motorFrontRight;
	public DcMotor motorFrontLeft;
	public DcMotor motorBackRight;
	public DcMotor motorBackLeft;
	public Servo grabStone;


	// Mechanum Definitions
	double Frontleft;
	double Frontright;
	double Backleft;
	double Backright;
	double Speed = 1;

	DigitalChannel LeftButton;
	DigitalChannel RightButton;
	DigitalChannel Button;

	@Override
	public void init() {

		/** Drive Train initialization **/

		//Motor Initialization
		motorFrontRight = hardwareMap.dcMotor.get("FR");
		motorFrontLeft = hardwareMap.dcMotor.get("FL");
		motorBackLeft = hardwareMap.dcMotor.get("BL");
		motorBackRight = hardwareMap.dcMotor.get("BR");
		grabStone = hardwareMap.servo.get("GS");


		//Motor Direction Initialization
		motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
		motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
		motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
		motorBackRight.setDirection(DcMotor.Direction.REVERSE);


		Button = hardwareMap.get(DigitalChannel.class, "B");
		Button.setMode(DigitalChannel.Mode.INPUT);


		LeftButton = hardwareMap.get(DigitalChannel.class, "LB");
		LeftButton.setMode(DigitalChannel.Mode.INPUT);

		RightButton = hardwareMap.get(DigitalChannel.class, "RB");
		RightButton.setMode(DigitalChannel.Mode.INPUT);

		//Initialized Telemetry
		telemetry.addData("Drive Train: ", "Initialized");
		telemetry.addData("Status: ", "Initialized");
		telemetry.addData("> Press Play to Start ", "TeleOp");
		telemetry.update();
	}

	@Override
	public void loop() {

		//Speed Controls
		if (gamepad1.a) {
			Speed = 1; // Full Speed
		}

		if (gamepad1.b) {
			Speed = .75; // Three Quarter Speed
		}

		if (gamepad1.x) {
			Speed = .50; // Half Speed
		}

		if (gamepad1.y) {
			Speed = .25; // Quarter Speed
		}

		if (!Button.getState()){
			grabStone.setPosition(0.3);
			telemetry.addData("Button", "Pressed ;)");
			telemetry.update();
		} else if (Button.getState()) {
			grabStone.setPosition(0.0);
			telemetry.addData("Button", "Not Pressed!?!?!?");
			telemetry.update();
		}

		if (!LeftButton.getState()){
			motorFrontLeft.setPower(0.0);
			motorBackLeft.setPower(0.0);

			float gamepad1LeftY = gamepad1.left_stick_y;
			float gamepad1LeftX = -gamepad1.left_stick_x;
			float gamepad1RightX = -gamepad1.right_stick_x;

			// Mechanum formulas

			double FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
			double BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;


			// clip the right/left values so that the values never exceed +/- 1

			// write the values to the motors
			motorFrontRight.setPower(Frontright);


			motorBackRight.setPower(Backright);

			// sets speed
			Frontright = Range.clip(Math.pow(FrontRight, 3), -Speed, Speed);

			Backright = Range.clip(Math.pow(BackRight, 3), -Speed, Speed);

		} else if(!RightButton.getState()) {

			motorFrontRight.setPower(0.0);
			motorBackRight.setPower(0.0);

			float gamepad1LeftY = gamepad1.left_stick_y;
			float gamepad1LeftX = -gamepad1.left_stick_x;
			float gamepad1RightX = -gamepad1.right_stick_x;

			// Mechanum formulas
			double FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
			double BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

			// clip the right/left values so that the values never exceed +/- 1

			// write the values to the motors

			motorFrontLeft.setPower(Frontleft);
			motorBackLeft.setPower(Backleft);


			// sets speed

			Frontleft = Range.clip(Math.pow(FrontLeft, 3), -Speed, Speed);

			Backleft = Range.clip(Math.pow(BackLeft, 3), -Speed, Speed);

		} else if(!LeftButton.getState() && !RightButton.getState()) {
			motorFrontLeft.setPower(0.0);
			motorBackLeft.setPower(0.0);
			motorFrontRight.setPower(0.0);
			motorBackRight.setPower(0.0);
		} else{
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


		}

		telemetry.addData("LB: ", LeftButton.getState());
		telemetry.addData("RB: ", RightButton.getState());
		telemetry.update();
	}
}
