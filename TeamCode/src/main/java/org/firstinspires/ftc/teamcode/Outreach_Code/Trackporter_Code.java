package org.firstinspires.ftc.teamcode.Outreach_Code;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import static java.lang.Math.abs;

@TeleOp(name = "Trackporter_Code", group = "Outreach_Code")
public class Trackporter_Code extends OpMode {

	Servo leftMotor;
	Servo rightMotor;

	Servo pan;
	Servo tilt;

	DcMotor light;

	boolean smooth = true;
	boolean fast = false;

	double left;
	double right;

	double leftUp;
	double leftDown;
	double rightUp;
	double rightDown;

	float stickY = gamepad1.left_stick_y;
	float stickX = -gamepad1.left_stick_x;


	public void init(){
		leftMotor = hardwareMap.servo.get("Servoleft");
		leftMotor.setDirection(Servo.Direction.FORWARD);

		rightMotor = hardwareMap.servo.get("Servoright");
		rightMotor.setDirection(Servo.Direction.FORWARD);

		pan = hardwareMap.servo.get("Pan");

		tilt = hardwareMap.servo.get("Tilt");

		light = hardwareMap.dcMotor.get("Light");
		light.setDirection(DcMotor.Direction.FORWARD);

		leftMotor.setPosition(0.5);
		rightMotor.setPosition(0.5);

		pan.setPosition(0.5);

		telemetry.addData("Left Motor: ", "Initialized");
		telemetry.addData("Right Motor: ", "Initialized");
		telemetry.addData("Tilt Servo: ", "Initialized");
		telemetry.addData("Pan Servo: ", "Initialized");
		telemetry.addData("Light: ", "Initialized");
		telemetry.addData("Ready to ", "Start");
		telemetry.update();
	}

	@Override
	public void loop(){

		left = -stickY - stickX;
		right = stickY - stickX;

		if (!fast) {
			left = Math.pow(left, 3) + 0.5;
			right = Math.pow(right, 3) + 0.5;
		} else if (fast){
			left = Math.pow(left,3) / 2 + 0.5;
			right = Math.pow(right, 3) / 2 + 0.5;
		}

		leftMotor.setPosition(left);
		rightMotor.setPosition(right);

		if (stickY > 0 || stickY < 0 || stickX > 0 || stickX < 0){
			light.setPower(-1.0);
		} else {
			light.setPower(0.0);
		}

		if (gamepad1.x && !fast){
			fast = true;
		} else if (gamepad1.x && fast){
			fast = false;
		}

		if (gamepad1.back && smooth){
			smooth = false;
		} else if(gamepad1.back && !smooth){
			smooth = true;
		}

		if (gamepad1.right_trigger > 0.3){
			leftMotor.setPosition(0.5);
			rightMotor.setPosition(0.5);
		} else if (smooth) {
			leftUp = abs(leftMotor.getPosition() + 0.00075);
			leftDown = abs(leftMotor.getPosition() - 0.00075);

			rightUp = abs(rightMotor.getPosition() + 0.00075);
			rightDown = abs(rightMotor.getPosition() - 0.00075);

			if (leftMotor.getPosition() < left){
				leftMotor.setPosition(leftUp);
			} else if (leftMotor.getPosition() > left){
				leftMotor.setPosition(leftDown);
			} else{
				leftMotor.setPosition(left);
			}

			if (rightMotor.getPosition() < right){
				rightMotor.setPosition(rightUp);
			} else if (rightMotor.getPosition() > right){
				rightMotor.setPosition(rightDown);
			} else{
				rightMotor.setPosition(right);
			}
		} else if (!smooth) {
			leftMotor.setPosition(left);
			rightMotor.setPosition(right);
		}

		if (gamepad1.right_stick_x < -0.1 && pan.getPosition() > 0.15){
			pan.setPosition(pan.getPosition() - 0.005);
		} else if (gamepad1.right_stick_x > 0.1 && pan.getPosition() < 0.67){
			pan.setPosition(pan.getPosition() + 0.005);
		}

		if (gamepad1.right_stick_y < -0.1){
			tilt.setPosition(tilt.getPosition() - 0.005);
		} else if (gamepad1.right_stick_y > 0.1){
			tilt.setPosition(tilt.getPosition() + 0.005);
		}

		if (gamepad1.a){
			tilt.setPosition(0.5);
			pan.setPosition(0.5);
		}

		if (gamepad1.b){
			tilt.setPosition(0.5);
			pan.setPosition(0.33);
		}

		telemetry.addData("Controls: ", "Left Stick Controls Robot, Right Stick Controls Camera");
		telemetry.addData("Controls: ", "Back Button Switches Smooth Controls, Right Trigger is Emergency Stop");
		telemetry.addData("Controls: ", "'X' Switches Speed, 'A' Centers Camera Forward, 'B' Centers Camera Backwards");

		if (fast){
			telemetry.addData("Speed: ", "Fast");
		} else if (!fast){
			telemetry.addData("Speed: ", "Slow");
		}

		if (smooth){
			telemetry.addData("Smooth Controls: ", "On");
		} else if (!smooth){
			telemetry.addData("Smooth Controls: ", "Off");
		}

		telemetry.addData("Pan Position: ", pan.getPosition());
		telemetry.addData("Tilt Position: ", tilt.getPosition());

		telemetry.update();
	}
}
