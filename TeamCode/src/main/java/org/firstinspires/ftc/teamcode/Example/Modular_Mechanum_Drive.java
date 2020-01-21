package org.firstinspires.ftc.teamcode.Example;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Modular_Mechanum_Drive", group= "Modular_Drivetrains")
public class Modular_Mechanum_Drive extends OpMode{

	public DcMotor motorFrontRight;
	public DcMotor motorFrontLeft;
	public DcMotor motorBackRight;
	public DcMotor motorBackLeft;
	double Speed = 1;
	double Frontleft;
	double Frontright;
	double Backleft;
	double Backright;

	public void init(){

		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

		motorFrontRight = hardwareMap.dcMotor.get("FR");
		motorFrontLeft = hardwareMap.dcMotor.get("FL");
		motorBackLeft = hardwareMap.dcMotor.get("BL");
		motorBackRight = hardwareMap.dcMotor.get("BR");
		//These work without reversing (Tetrix motors).
		//AndyMark motors may be opposite, in which case uncomment these lines:

		motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
		motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
		motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
		motorBackRight.setDirection(DcMotor.Direction.REVERSE);

	}

	@Override
	public void loop(){

		// left stick controls direction
		// right stick X controls rotation

		float gamepad1LeftY = gamepad1.left_stick_y;
		float gamepad1LeftX = -gamepad1.left_stick_x;
		float gamepad1RightX = -gamepad1.right_stick_x;

		// Mechanum formulas

		double FrontLeft = -gamepad1LeftY - gamepad1LeftX + gamepad1RightX;
		double FrontRight = gamepad1LeftY - gamepad1LeftX + gamepad1RightX;
		double BackRight = gamepad1LeftY + gamepad1LeftX + gamepad1RightX;
		double BackLeft = -gamepad1LeftY + gamepad1LeftX + gamepad1RightX;

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

		if (gamepad1.a){
			Speed = 1;
		}

		if (gamepad1.b){
			Speed = .75;
		}

		if (gamepad1.x){
			Speed = .50;
		}

		if (gamepad1.y){
			Speed = .25;
		}

		telemetry.addData("speed", Speed);
		telemetry.update();
	}

	/*
	 * This method scales the joystick input so for low joystick values, the
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	public double scaleInput(double dVal){
		double[] scaleArray ={ 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);

		// index should be positive.
		if (index < 0){
			index = -index;
		}

		// index cannot exceed size of array minus 1.
		if (index > 16){
			index = 16;
		}

		// get value from the array.
		double dScale;
		if (dVal < 0){
			dScale = -scaleArray[index];
		} else{
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
	}

}