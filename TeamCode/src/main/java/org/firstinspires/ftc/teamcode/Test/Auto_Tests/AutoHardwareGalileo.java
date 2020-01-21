package org.firstinspires.ftc.teamcode.Test.Auto_Tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


public class AutoHardwareGalileo {
	//Drive train definitions
	public DcMotor motorFrontRight;
	public DcMotor motorFrontLeft;
	public DcMotor motorBackRight;
	public DcMotor motorBackLeft;

	//Lift Definitions
	public DcMotor lift;
	public DcMotor slide;

	public Servo grabStone;
	public Servo wrist;

	//Color Sensor Definitions
	ColorSensor color_sensor;
	ColorSensor color2;

	//Foundation Mover Definitions
	public Servo foundationMoverL;      // Defines the left foundation servo
	public Servo foundationMoverR;      // Defines the right foundation servo

	//Stone Button Definitions
	public DigitalChannel stoneButton; // Defines the Stone Button on the grabber

	//Foundation Bumper Definitions
	public TouchSensor foundationBumperLeft;  //Defines the left foundation bumper button
	public TouchSensor foundationBumperRight; //Defines the right foundation bumper button

	//Gyro Definitions
	// Defines the gyro
	BNO055IMU imu;

	// State used for updating telemetry
	Orientation angles;
	Acceleration gravity;

	//Generic Auto Definitions
	//Drive Train Encoder Definitions
	public static final double     COUNTS_PER_MOTOR_REV    = 560 ;    // eg: REV 20:1 Motor Encoder
	public static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
	public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

	//Lift Encoder Definitions
	static final double COUNTS_PER_LIFT_INCH = 55;  // Sets the double "COUNTS_PER_LEVEL" to 300    | Defines how long the lift needs to run to go up one level | About 55  counts per inch

	//Gyro Turning Definitions
	static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
	static final double     P_TURN_COEFF            = 0.15;     // Larger is more responsive, but also less stable

	//Vuforia Definitions
	//Scanning Timing and Position Definitions
	public double pos = 0;
	public int scanTime = 2000;

	// Skystone detection definitions
	public boolean Skystone = false;
	public static final String TFOD_MODEL_ASSET = "Skystone.tflite";
	public static final String LABEL_FIRST_ELEMENT = "Stone";
	public static final String LABEL_SECOND_ELEMENT = "Skystone";
	public static final String VUFORIA_KEY = "ASHBoLr/////AAABmbIUXlLiiEgrjVbqu8Iavlg6iPFigYso/+BCZ9uMzyAZFoo9CIzpV818SAqrjzuygz3hCeLW/ImK3xMH7DalGMwavqetwXS9Jw4I+rff2naxgV7n+EtYFvdCkUJDHfHVq1A4mhxDHgrjWZEqnLmZk25ppnIizQ0Ozcq4h6UmrWndEVEz8eKcCgn+IuglCEoEswvNBRAaKm/TAlpxLRNC6jQkZdJUh/TGYT05g9YCZo4+1ugmx01jrPCyHQVPVoeXm6VebLIuP7sNPw7njYzmVi2ffV5bYc4vf5kc5l5JwhBdPqnxuMfDLnHWaCkAO1UlVWqy2eY7/4b6iUYI2yN16ZKswSzLMmMNtPBu7e9HhKxA";

	public VuforiaLocalizer vuforia;
	public TFObjectDetector tfod;

	//Local OpMode Members
	HardwareMap hwMap =  null;

	//Constructor
	public AutoHardwareGalileo () {
		//Purposefully Left Empty
	}

	//Initialization Void
	public void init(HardwareMap ahwMap) {
		//Save Reference to Hardware Map
		hwMap = ahwMap;

		//Drive Train Initialization
		motorFrontLeft = hwMap.dcMotor.get("FL");
		motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

		motorFrontRight = hwMap.dcMotor.get("FR");
		motorFrontRight.setDirection(DcMotor.Direction.FORWARD);

		motorBackLeft = hwMap.dcMotor.get("BL");
		motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

		motorBackRight = hwMap.dcMotor.get("BR");
		motorBackRight.setDirection(DcMotor.Direction.FORWARD);

		//Encoder Initialization
		//Stop and Reset Encoders
		motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		//Run Using Encoders
		motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		//Lift Initialization
		lift = hwMap.dcMotor.get("LT");
		lift.setDirection(DcMotor.Direction.FORWARD);

		//Hand Initialization
		grabStone = hwMap.servo.get("GS");

		wrist = hwMap.servo.get("W");
		wrist.setPosition(.4); // Center the wrist

		//Slide Initialization
		slide = hwMap.dcMotor.get("SL");
		slide.setDirection(DcMotor.Direction.FORWARD);

		//Foundation Mover Initialization
		foundationMoverR = hwMap.servo.get("GR");     // Initializes the right foundation movers name for configuration
		foundationMoverL = hwMap.servo.get("GL");     // Initializes the left foundation movers name for configuration
		foundationMoverR.setPosition(0);                    // Sets the right foundation mover to point up
		foundationMoverL.setPosition(0);                    // Sets the left foundation mover to point up

		//Stone Button Sensor Initialization
		stoneButton = hwMap.get(DigitalChannel.class, "stone_button"); // Initializes the stone button name for configuration
		stoneButton.setMode(DigitalChannel.Mode.INPUT);                           // Initializes the mode of the button

		//Foundation Bumpers Initialization
		foundationBumperLeft = hwMap.touchSensor.get("bumper_left");   // Initializes the stone button name for configuration
		foundationBumperRight = hwMap.touchSensor.get("bumper_right"); // Initializes the stone button name for configuration

		//Gyro Initialization
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
		parameters.loggingEnabled = true;
		parameters.loggingTag = "IMU";
		parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
		imu = hwMap.get(BNO055IMU.class, "imu");
		imu.initialize(parameters);

		//Color Sensor Initialization
		color_sensor = hwMap.get(ColorSensor.class, "color_sensor");
		color2 = hwMap.get(ColorSensor.class, "color2");
		//sensorDistance = hardwareMap.get(DistanceSensor.class, "color_sensor");
		float hsvValues[] = {0F, 0F, 0F};
		final float values[] = hsvValues;
		final double SCALE_FACTOR = 255;
	}
}
