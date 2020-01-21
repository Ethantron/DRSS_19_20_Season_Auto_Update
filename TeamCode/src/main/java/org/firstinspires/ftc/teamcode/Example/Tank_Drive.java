package org.firstinspires.ftc.teamcode.Example;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SampleRevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Tank_Drive", group= "Modular_Drivetrains")
public class Tank_Drive extends OpMode{

	public DcMotor motorFrontRight;
	public DcMotor motorFrontLeft;
	public DcMotor motorBackRight;
	public DcMotor motorBackLeft;
	double Speed = 1;
	double Frontleft;
	double Frontright;
	double Backleft;
	double Backright;

	private final static int LED_PERIOD = 10;
	RevBlinkinLedDriver blinkinLedDriver;
	RevBlinkinLedDriver.BlinkinPattern pattern;

	Telemetry.Item patternName;
	Telemetry.Item display;
	Deadline ledCycleDeadline;
	Deadline gamepadRateLimit;
	SampleRevBlinkinLedDriver.DisplayKind displayKind;

	protected enum DisplayKind {
		MANUAL,
		AUTO
	}

	public void init(){

		displayKind = SampleRevBlinkinLedDriver.DisplayKind.MANUAL;

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


		blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
		pattern = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE;
		blinkinLedDriver.setPattern(pattern);

		display = telemetry.addData("Display Kind: ", displayKind.toString());
		patternName = telemetry.addData("Pattern: ", pattern.toString());

		ledCycleDeadline = new Deadline(LED_PERIOD, TimeUnit.SECONDS);
	}

	@Override
	public void loop(){

		// sets speed
		Frontright = Range.clip(Math.pow(gamepad1.right_stick_y, 3), -Speed, Speed);
		Frontleft = Range.clip(Math.pow(-gamepad1.left_stick_y, 3), -Speed, Speed);
		Backright = Range.clip(Math.pow(gamepad1.right_stick_y, 3), -Speed, Speed);
		Backleft = Range.clip(Math.pow(-gamepad1.left_stick_y, 3), -Speed, Speed);

		// write the values to the motors
		motorFrontRight.setPower(Frontright);
		motorFrontLeft.setPower(Frontleft);
		motorBackLeft.setPower(Backleft);
		motorBackRight.setPower(Backright);

		if (gamepad1.a){
			Speed = 1;
			pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE;
			displayPattern();
		}

		if (gamepad1.b){
			Speed = .75;
			pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
			displayPattern();
		}

		if (gamepad1.x){
			Speed = .50;
			pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
			displayPattern();
		}

		if (gamepad1.y){
			Speed = .25;
			pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
			displayPattern();
		}

		telemetry.addData("speed", Speed);
		telemetry.update();

		if (gamepad1.left_stick_y>0.1 && gamepad1.right_stick_y>0.1){
			pattern = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;
			displayPattern();
		}
		if (gamepad1.left_stick_y<-0.1 && gamepad1.right_stick_y<-0.1){
			pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE;
			displayPattern();
		}

		if(gamepad1.left_stick_y>.1 && gamepad1.right_stick_y<-.1){
			pattern = RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE;
			displayPattern();
		}

		if(gamepad1.left_stick_y<-.1 && gamepad1.right_stick_y>.1){
			pattern = RevBlinkinLedDriver.BlinkinPattern.SINELON_FOREST_PALETTE;
			displayPattern();
		}
	}

	protected void setDisplayKind(SampleRevBlinkinLedDriver.DisplayKind displayKind)
	{
		this.displayKind = displayKind;
		display.setValue(displayKind.toString());
	}

	protected void doAutoDisplay()
	{
		if (ledCycleDeadline.hasExpired()) {
			pattern = pattern.next();
			displayPattern();
			ledCycleDeadline.reset();
		}
	}

	protected void displayPattern()
	{
		blinkinLedDriver.setPattern(pattern);
		patternName.setValue(pattern.toString());
	}
}
