package org.firstinspires.ftc.teamcode.Example;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
	public DigitalChannel Button; // Defines the Stone Button on the grabber
	public DigitalChannel zero;
	public DigitalChannel one;
	public DigitalChannel two;
	public DigitalChannel three;
	public DigitalChannel four;
	public DigitalChannel five;


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

		Button = hardwareMap.get(DigitalChannel.class, "button"); // Initializes the stone button name for configuration
		Button.setMode(DigitalChannel.Mode.INPUT);                                 // Initializes the mode of the button

		zero = hardwareMap.get(DigitalChannel.class, "zero"); // Initializes the stone button name for configuration
		zero.setMode(DigitalChannel.Mode.INPUT);                                 // Initializes the mode of the button

		one = hardwareMap.get(DigitalChannel.class, "one"); // Initializes the stone button name for configuration
		one.setMode(DigitalChannel.Mode.INPUT);                                 // Initializes the mode of the button

		two = hardwareMap.get(DigitalChannel.class, "two"); // Initializes the stone button name for configuration
		two.setMode(DigitalChannel.Mode.INPUT);                                 // Initializes the mode of the button

		three = hardwareMap.get(DigitalChannel.class, "three"); // Initializes the stone button name for configuration
		three.setMode(DigitalChannel.Mode.INPUT);                                 // Initializes the mode of the button

		four = hardwareMap.get(DigitalChannel.class, "four"); // Initializes the stone button name for configuration
		four.setMode(DigitalChannel.Mode.INPUT);                                 // Initializes the mode of the button

		five = hardwareMap.get(DigitalChannel.class, "five"); // Initializes the stone button name for configuration
		five.setMode(DigitalChannel.Mode.INPUT);                                 // Initializes the mode of the button
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

		if (!Button.getState()){
			pattern = RevBlinkinLedDriver.BlinkinPattern.CONFETTI;
			displayPattern();
		}

		if (!zero.getState()){
			pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE;
			displayPattern();
		}

		if (!one.getState()){
			pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
			displayPattern();
		}

		if (!two.getState()){
			pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER;
			displayPattern();
		}

		if (!three.getState()){
			pattern = RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE;
			displayPattern();
		}

		if (!four.getState()){
			pattern = RevBlinkinLedDriver.BlinkinPattern.LIME;
			displayPattern();
		}

		if (!five.getState()){
			pattern = RevBlinkinLedDriver.BlinkinPattern.TWINKLES_RAINBOW_PALETTE;
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
