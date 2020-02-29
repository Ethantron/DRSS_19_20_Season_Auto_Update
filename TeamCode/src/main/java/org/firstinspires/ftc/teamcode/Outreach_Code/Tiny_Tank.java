/*Controls Used:
Left Stick, Right Stick, Dpad up, Dpad Down, Dpad Left, Dpad Right, A Button, Right Trigger, Left Bumper
Controls Left:
X Button, B Button, Y Button, Back Button, Start Button, Left Trigger, Right Bumper
*/
//The code that is required for the robot to be identified and function.

package org.firstinspires.ftc.teamcode.Outreach_Code;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.abs;


@SuppressWarnings("MagicConstant")
@TeleOp(name = "Tiny_Tank", group= "team")
public class Tiny_Tank extends OpMode {

	//Introduces the programmable parts of the robot and sets different variables.

	Servo Servoleft;
	//Port 0
	Servo Servoright;
	//Port 1
	DcMotor Light;

	Servo Pan;
	//Port 2
	Servo Tilt;
	//Port 3
	// DcMotor Relay;
	double slowtggl = 0;
	double speed = 1;
	double Valadjust = 0;
	double ValAdjust = 0;



	//Initializes the different movable parts of the robot and sets the direction.

	public void init() {
		Servoleft = hardwareMap.servo.get("Servoleft");
		Servoleft.setDirection(Servo.Direction.FORWARD);

		Servoright = hardwareMap.servo.get("Servoright");
		Servoright.setDirection(Servo.Direction.FORWARD);

		Light = hardwareMap.dcMotor.get("RedLight");
		Light.setDirection(DcMotorSimple.Direction.FORWARD);

		//  Relay = hardwareMap.dcMotor.get("Relay");
		//  Relay.setDirection(DcMotorSimple.Direction.FORWARD);

		Pan = hardwareMap.servo.get("Pan"); //Continuous Rotation Servo
		Tilt = hardwareMap.servo.get("Tilt"); //180 Degree Servo

		//Tilt.setPosition(0.5); //Sets servo into "Zero" position
		Pan.setPosition(0.5);
		Servoright.setPosition(0.5);
		Servoleft.setPosition(0.5);
		//    TiltActive = 1;


	}

	//Start of the actual code.

	public void loop() {

		//Sets the relay's power to speed.

		//  Relay.setPower(relayset);

		//Allows you to press one button to turn on or off the relay.

        /*if (currentstep == 0){
            if (gamepad1.a){
                Relay.setPower(relayset = 1);
                currentstep ++;
            }
        }
        if (currentstep == 1){
            if(gamepad1.a){
                Relay.setPower(relayset = 0);
                currentstep --;
            }
        }*/

		//The code that allows the wheels to be controlled by the joysticks on the controllers.

		double joy = gamepad1.left_stick_y;

		//double Valadjust = (joy * joy * joy) / 2 + 0.5;

		double Joy = gamepad1.right_stick_y;

		//double ValAdjust = (Joy * Joy * Joy) / 2 + 0.5;

		if (gamepad1.right_bumper && speed == 0){
			speed = 1;
		}

		else if (gamepad1.right_bumper && speed == 1){
			speed = 0;
		}

		if (speed == 0){
			Valadjust = (joy * joy * joy) + 0.5;
			ValAdjust = (Joy * Joy * Joy) + 0.5;
		}

		if (speed == 1){
			Valadjust = (joy * joy * joy) / 2 + 0.5;
			ValAdjust = (Joy * Joy * Joy) / 2 + 0.5;
		}

		if (gamepad1.left_bumper && slowtggl == 0){
			slowtggl = 1;
		}

		else if (gamepad1.left_bumper && slowtggl == 1){
			slowtggl = 0;
		}

		if (slowtggl == 0) {
			//Right Wheel
			double Rightup = abs(Servoright.getPosition() + 0.0075); //0.0075 is the rate that the speed is changed at per frame
			double Rightdown = abs(Servoright.getPosition() - 0.0075);
			if (Servoright.getPosition() < Valadjust) {
				Servoright.setPosition(Rightup);
			}

			if (Servoright.getPosition() > Valadjust) {
				Servoright.setPosition(Rightdown);
			} else if (!(Servoright.getPosition() < Valadjust) && !(Servoright.getPosition() > Valadjust) && gamepad1.right_trigger <= 0.3) {
				Servoright.setPosition(Valadjust);
			}

			//Left Wheel
			double Leftup = abs(Servoleft.getPosition() + 0.0075);
			double Leftdown = abs(Servoleft.getPosition() - 0.0075);
			if (Servoleft.getPosition() < ValAdjust) {
				Servoleft.setPosition(Leftup);
			}

			if (Servoleft.getPosition() > ValAdjust) {
				Servoleft.setPosition(Leftdown);
			}

			if (gamepad1.right_trigger > .3) {
				Servoleft.setPosition(0.5);
				Servoright.setPosition(0.5);
			} else if (!(Servoleft.getPosition() < ValAdjust) && !(Servoleft.getPosition() > ValAdjust) && gamepad1.right_trigger <= 0.3) {
				Servoleft.setPosition(ValAdjust);
			}
		}

		else if (slowtggl == 1){
			Servoleft.setPosition(ValAdjust);
			Servoright.setPosition(Valadjust);
		}

		if (joy < 0 || joy > 0 || Joy < 0 || Joy > 0) {
			Light.setPower(-1);
		}

		else {
			Light.setPower(0);
		}

		if (gamepad1.dpad_down){
			Tilt.setPosition(.75);
		}

		if (gamepad1.dpad_up){
			Tilt.setPosition(.25);
		}

		if (gamepad1.a){
			Tilt.setPosition(.50);
			Pan.setPosition(.50);
		}

		if (gamepad1.b){
			Pan.setPosition(.33);
		}

		if (gamepad1.dpad_left){
			if (Pan.getPosition() > 0.15) {
				Pan.setPosition(Pan.getPosition() - 0.003);
			}
		}

		if (gamepad1.dpad_right){
			if (Pan.getPosition() <0.67) {
				Pan.setPosition(Pan.getPosition() + 0.003);
			}
		}

		if (slowtggl == 0) {
			telemetry.addData("Slow mode:", "on");
		}

		if (slowtggl == 1) {
			telemetry.addData("Slow mode:", "off");
		}

		if (speed == 0) {
			telemetry.addData("Slow", "");
		}

		if (speed == 1) {
			telemetry.addData("Fast", "");
		}
		telemetry.addData("pan", Pan.getPosition());

	}
}