package org.firstinspires.ftc.teamcode.Test.Auto_Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "EncoderTurnTest", group = "Auto_Test")
public class EncoderTurnTest extends LinearOpMode {

    // Robot definitions
    //Defines the Motors
    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;

    //Defines the Variables
    double step = 0;

    //Timer
    private ElapsedTime runtime = new ElapsedTime();

    //Encoder
    static final double COUNTS_PER_MOTOR_REV = 560;    // eg: REV 20:1 Motor Encoder
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {

        // Drive train initialization
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);

        //Encoder Initialization
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        //Stops and Resets Encoders
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Tells Robots to Reset Encoders
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.clear();

        telemetry.addData("Status: ", "Initialized");
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart(); //Waits for the Driver to Press Start

        encoderTurn(.25, 90, 10); //Turn CCW 90 Degrees
        sleep(1000);
        encoderTurn(.25, 90, 10); //Turn CCW 180 Degrees
        sleep(1000);
        encoderTurn(.25, 90, 10); //Turn CCW 270 Degrees
        sleep(1000);
        encoderTurn(.25, 90, 10); //Turn CCW 360 Degrees
        sleep(1000);
        encoderTurn(.25, -180, 10); //Turn CW 180 Degrees
        sleep(1000);
        encoderTurn(.25, -360, 10); //Turn CW 360 Degrees
        sleep(1000);
        encoderTurn(.25, 80, 10); //Turn CCW 80 Degrees
        sleep(1000);
        telemetry.addData("Turning ", "Complete! :)");
        telemetry.update();
        sleep(1000);
    }

    public void encoderTurn(double speed, double angle, double timeoutS) {
        //Create our target variables
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        //Create our new circumference variables
        double r = 9.605; //Radius of arc created by robot wheels
        double c = 60.35; //Circumference of arc created by robot wheels
        double ANGLE_RATIO = angle / 360; //Ratio of angle relative to entire circle
        double CIRCUMFERENCE_OF_ANGLE = c * ANGLE_RATIO; //Circumference of Angle
        int COUNTS_PER_DISTANCE = (int) ((CIRCUMFERENCE_OF_ANGLE * COUNTS_PER_INCH) * 1.35);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Math to calculate each target position for the motors
            newFrontLeftTarget = motorFrontLeft.getCurrentPosition() - COUNTS_PER_DISTANCE;
            newFrontRightTarget = motorFrontRight.getCurrentPosition() + COUNTS_PER_DISTANCE;
            newBackLeftTarget = motorBackLeft.getCurrentPosition() - COUNTS_PER_DISTANCE;
            newBackRightTarget = (motorBackRight.getCurrentPosition() + COUNTS_PER_DISTANCE);

            //Set Target Positions to respective motors
            motorFrontLeft.setTargetPosition(newFrontLeftTarget);
            motorFrontRight.setTargetPosition(newFrontRightTarget);
            motorBackLeft.setTargetPosition(newBackLeftTarget);
            motorBackRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorFrontLeft.setPower(speed);
            motorFrontRight.setPower(speed);
            motorBackLeft.setPower(speed);
            motorBackRight.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorBackLeft.isBusy() && motorBackRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("FLM: Path2", "Running at %7d", //Tells us where we are
                        motorFrontLeft.getCurrentPosition()); //Front Left Position
                telemetry.addData("FRM: Path2", "Running at %7d", //Tells us where we are
                        motorFrontRight.getCurrentPosition()); //Front Right Position
                telemetry.addData("BLM: Path2", "Running at %7d", //Tells us where we are
                        motorBackLeft.getCurrentPosition()); //Back Left Position
                telemetry.addData("BRM: Path2", "Running at %7d", //Tells us where we are
                        motorBackRight.getCurrentPosition()); //Back Right Position
                telemetry.update();
            }

            // Stop all motion;
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
