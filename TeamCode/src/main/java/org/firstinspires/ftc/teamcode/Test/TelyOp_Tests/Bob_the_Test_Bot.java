package org.firstinspires.ftc.teamcode.Test.TelyOp_Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Bob_the_Test_Bot", group= "TelyOp_Tests")
public class Bob_the_Test_Bot extends LinearOpMode{

    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;
    public DcMotor lift;
    public DcMotor slide;
    public Servo grabL;
    public Servo grabR;
    public Servo grabStone;
    double Frontleft;
    double Frontright;
    double Backleft;
    double Backright;
    double Speed = 1;
    double upstep = 0;
    double upcount = 0;

    public void runOpMode(){

        // Drive Train initilization
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        // Payload initialization
        lift = hardwareMap.dcMotor.get("LT");
        slide = hardwareMap.dcMotor.get("SL");

        lift.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);

        grabL = hardwareMap.servo.get("GL");
        grabR = hardwareMap.servo.get("GR");
        grabStone = hardwareMap.servo.get("GS");

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();


        // Drivetrain Controls

            // left stick controls direction
            // right stick X controls rotation
            float gamepad1LeftY = gamepad1.left_stick_y;
            float gamepad1LeftX = -gamepad1.left_stick_x;
            float gamepad1RightX = gamepad1.right_stick_x;

            // Mechanum formulas
            double FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            double FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            double BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            double BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

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

            if (gamepad1.dpad_left){
                grabL.setPosition(90);
                grabR.setPosition(0);
            }

            if (gamepad1.dpad_right){
                grabR.setPosition(90);
                grabL.setPosition(0);
            }

            if (gamepad1.dpad_down){
                grabL.setPosition(0);
                grabR.setPosition(0);
            }

            if (gamepad1.dpad_up){
                grabL.setPosition(90);
                grabR.setPosition(90);
            }

        // Payload Controls

            if (gamepad2.right_bumper && upcount <= 6){
                lift.setPower(0.5);
                sleep(500);
                lift.setPower(0.1);
                upcount++;
            }
            else{
                telemetry.addData("Maximum","Hight Reached");
            }

            if (gamepad2.left_bumper && upcount >= 0){
                lift.setPower(-0.3);
                sleep(500);
                lift.setPower(0.1);
                upcount--;
            }

            if (gamepad2.left_trigger > 0.1){
                lift.setPower(0.5);
            }

            if (gamepad2.right_trigger > 0.1){
                lift.setPower(-0.3);
            }

            if (!(gamepad2.left_trigger > 0.1 && gamepad2.right_trigger > 0.1)){
                lift.setPower(0.1);
            }

            if (gamepad2.left_stick_x > 0.1){
                slide.setPower(0.3);
            }

            if (gamepad2.left_stick_x < 0.1){
                slide.setPower(-0.3);
            }

            if (!(gamepad2.left_stick_x > 0.1) && !(gamepad2.left_stick_x < 0.1)){
                slide.setPower(0);
            }

            if (gamepad2.a){
                grabStone.setPosition(90);
            }

            if (gamepad2.b){
                grabStone.setPosition(0);
            }

        //Telemetry

        telemetry.addData("Current Speed: ", Speed);

        if (grabL.getPosition() == 90){
            telemetry.addData("Left Grabber: ", "Open");
        }
        else if (grabL.getPosition() == 0){
            telemetry.addData("Left Grabber: ", "Closed");
        }

        if (grabR.getPosition() == 90){
            telemetry.addData("Right Grabber: ", "Open");
        }
        else if (grabR.getPosition() == 0){
            telemetry.addData("Right Grabber: ", "Closed");
        }

        if (grabStone.getPosition() == 90){
            telemetry.addData("Stone Grabber: ", "Open");
        }
        else if (grabStone.getPosition() == 0){
            telemetry.addData("Stone Grabber: ", "Closed");
        }
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
