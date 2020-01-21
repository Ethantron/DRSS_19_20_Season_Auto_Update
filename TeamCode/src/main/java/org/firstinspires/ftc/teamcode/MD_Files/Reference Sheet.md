TelyOp reference sheet.

	Important:
	Put this at the end of every Init.

		telemetry.addData("Status: ", "Initialized");
		telemetry.addData(">", "Press Play to start op mode");
		telemetry.update();

	TelyOp:
	
		@TeleOp(name = "", group= "")

	Galileo Definitions:
		AutoHardwareGalileo robot = new AutoHardwareGalileo();   //Calls Upon Robot Definitions File

	Galileo Initialization:
		robot.init(hardwareMap); //Calls Upon Robot Initialization File

		composeTelemetry(); //Gyro Telemetry Initialization

		// Skystone detection initialization
			initVuforia(); //Vuforia Initialization

			if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
				initTfod(); //Tensor Flow Object Detection Initialization
			} else {
				telemetry.addData("Sorry!", "This device is not compatible with TFOD");
			}

			if (robot.tfod != null) {
				robot.tfod.activate();
			}

			telemetry.addData("Drive Train: ", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
			telemetry.addData("Payload: ", "Initialized");          // Adds telemetry to the screen to show that the payload is initialized
			telemetry.addData("Status: ", "Ready");                 // Adds telemetry to the screen to show that the robot is ready
			telemetry.addData("Press Play to Start ", "TeleOp");    // Adds telemetry to the screen to tell the drivers that the code is ready to start
			telemetry.update();                                     // Tells the telemetry to display on the phone
			waitForStart();

	Mechanum definitions:

		public DcMotor motorFrontRight;
		public DcMotor motorFrontLeft;
		public DcMotor motorBackRight;
		public DcMotor motorBackLeft;
		double Speed = 1;
		double Frontleft;
		double Frontright;
		double Backleft;
		double Backright;

	Mecanum Init:
	
		motorFrontRight = hardwareMap.dcMotor.get("FR");
		motorFrontLeft = hardwareMap.dcMotor.get("FL");
		motorBackLeft = hardwareMap.dcMotor.get("BL");
		motorBackRight = hardwareMap.dcMotor.get("BR");

		motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
		motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
		motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
		motorBackRight.setDirection(DcMotor.Direction.REVERSE);

Autonomous directions reference sheet.

	Stop:
	
		robot.motorFrontRight.setPower(0);
		robot.motorFrontLeft.setPower(0);
		robot.motorBackLeft.setPower(0);
		robot.motorBackRight.setPower(0);

	Forward:
	
		robot.motorFrontRight.setPower(.6);
		robot.motorFrontLeft.setPower(.6);
		robot.motorBackLeft.setPower(.6);
		robot.motorBackRight.setPower(.6);

	Reverse:
	
		robot.motorFrontRight.setPower(-.6);
		robot.motorFrontLeft.setPower(-.6);
		robot.motorBackLeft.setPower(-.6);
		robot.motorBackRight.setPower(-.6);

	Turn Left:
	
		robot.motorFrontRight.setPower(.6);
		robot.motorFrontLeft.setPower(-.6);
		robot.motorBackLeft.setPower(-.6);
		robot.motorBackRight.setPower(.6);

	Turn Right:
	
		robot.motorFrontRight.setPower(-.6);
		robot.motorFrontLeft.setPower(.6);
		robot.motorBackLeft.setPower(.6);
		robot.motorBackRight.setPower(-.6);

	Strafe Left:
	
		robot.motorFrontRight.setPower(.7);
		robot.motorFrontLeft.setPower(-.4);
		robot.motorBackLeft.setPower(.7);
		robot.motorBackRight.setPower(-.4);

	Strafe Right:
	
		robot.motorFrontRight.setPower(-.7);
		robot.motorFrontLeft.setPower(.4);
		robot.motorBackLeft.setPower(-.7);
		robot.motorBackRight.setPower(.4);

	Timing Reference:
	
		ResetTime.reset();
		while (opModeIsActive() && (ResetTime.seconds() < 1)){
		telemetry.addData("Where", "Step : 1 %2.5f S Elapsed", ResetTime.seconds());
		telemetry.update();
		}

	Reset Time:

		ElapsedTime ResetTime = new ElapsedTime();

	Andymark 20:1 Encoder Definitions:

		static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: Andymark 20:1 Motor Encoder
		static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
		static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

	Andymark 40:1 Encoder Definitions:

		static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: Andymark 40:1 Motor Encoder
		static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
		static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

	Andymark 60:1 Encoder Definitions:

		static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: Andymark 60:1 Motor Encoder
		static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
		static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

	REV 20:1 Planetary Motor Encoder Definitions:

		static final double     COUNTS_PER_MOTOR_REV    = 560 ;    // eg: REV 20:1 Motor Encoder
		static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
		static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

	REV 40:1 Spur Motor Encoder Definitions:

		static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: REV 40:1 Motor Encoder
		static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
		static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

	REV Core Hex Motor Encoder Definitions:

		static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // eg: REV Core Hex Motor Encoder
		static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
		static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);
