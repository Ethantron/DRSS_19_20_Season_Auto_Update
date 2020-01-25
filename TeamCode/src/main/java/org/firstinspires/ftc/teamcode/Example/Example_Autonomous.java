package org.firstinspires.ftc.teamcode.Example;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Example_Autonomous", group= "Modular_Drivetrains")
public class Example_Autonomous extends LinearOpMode {

	public DcMotor motorFrontRight;
	public DcMotor motorFrontLeft;
	public DcMotor motorBackRight;
	public DcMotor motorBackLeft;

	public long time;

	public DigitalChannel zero;
	public DigitalChannel one;
	public DigitalChannel two;
	public DigitalChannel three;
	public DigitalChannel four;
	public DigitalChannel five;


	private ElapsedTime runtime = new ElapsedTime(); //Sets timer for encoders

	double step = 0; //Sets the steps for the autonomous

	@Override
	public void runOpMode(){
		motorFrontLeft = hardwareMap.dcMotor.get("FL");
		motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

		motorFrontRight = hardwareMap.dcMotor.get("FR");
		motorFrontRight.setDirection(DcMotor.Direction.FORWARD);

		motorBackLeft = hardwareMap.dcMotor.get("BL");
		motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

		motorBackRight = hardwareMap.dcMotor.get("BR");
		motorBackRight.setDirection(DcMotor.Direction.FORWARD);


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

		if (!zero.getState()){
			time = 0;
		} else if (!one.getState()){
			time = 1;
		} else if (!two.getState()){
			time = 2;
		} else if (!three.getState()){
			time = 3;
		} else if (!four.getState()){
			time = 4;
		} else if (!five.getState()){
			time = 5;
		}

		telemetry.addData("Drive Train: ", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Delay: ",  time + " seconds");          // Adds telemetry to the screen to show that the payload is initialized
		telemetry.addData("Status: ", "Ready");                 // Adds telemetry to the screen to show that the robot is ready
		telemetry.addData("Press Play to Start ", "Autonomous");    // Adds telemetry to the screen to tell the drivers that the code is ready to start
		telemetry.update();                                                   // Tells the telemetry to display on the phone
		waitForStart();

		if (step == 0){
			sleep(time*1000);
			step++;
		}

		if (step == 1){
			motorFrontLeft.setPower(1);
			motorFrontRight.setPower(1);
			motorBackLeft.setPower(1);
			motorBackRight.setPower(1);
			sleep(1000);
			motorFrontLeft.setPower(0);
			motorFrontRight.setPower(0);
			motorBackLeft.setPower(0);
			motorBackRight.setPower(0);
			telemetry.addData("Program", "Finished");
		}
	}
}
