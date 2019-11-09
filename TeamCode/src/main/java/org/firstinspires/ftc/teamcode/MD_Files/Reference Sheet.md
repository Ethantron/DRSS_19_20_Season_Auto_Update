TelyOp reference sheet.

    Important:
    Put this at the end of every Init.

        telemetry.addData("Status: ", "Initialized");
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

    TelyOp:
    
        @TeleOp(name = "", group= "")

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
    
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    Forward:
    
        motorFrontRight.setPower(.6);
        motorFrontLeft.setPower(.6);
        motorBackLeft.setPower(.6);
        motorBackRight.setPower(.6);

    Reverse:
    
        motorFrontRight.setPower(-.6);
        motorFrontLeft.setPower(-.6);
        motorBackLeft.setPower(-.6);
        motorBackRight.setPower(-.6);

    Turn Left:
    
        motorFrontRight.setPower(.6);
        motorFrontLeft.setPower(-.6);
        motorBackLeft.setPower(-.6);
        motorBackRight.setPower(.6);

    Turn Right:
    
        motorFrontRight.setPower(-.6);
        motorFrontLeft.setPower(.6);
        motorBackLeft.setPower(.6);
        motorBackRight.setPower(-.6);

    Strafe Left:
    
        motorFrontRight.setPower(.7);
        motorFrontLeft.setPower(-.4);
        motorBackLeft.setPower(.7);
        motorBackRight.setPower(-.4);

    Strafe Right:
    
        motorFrontRight.setPower(-.7);
        motorFrontLeft.setPower(.4);
        motorBackLeft.setPower(-.7);
        motorBackRight.setPower(.4);

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
