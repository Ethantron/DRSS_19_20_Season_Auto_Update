package org.firstinspires.ftc.teamcode.Test.TelyOp_Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "19-20 TeleOp Linear", group= "TeleOp")
public class TeleOp_19_20_Linear extends LinearOpMode {

    // Robot definitions

    ElapsedTime ResetTime = new ElapsedTime();

    // Motor Definitions
    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;

    // Mechanum Definitions
    double Frontleft;
    double Frontright;
    double Backleft;
    double Backright;
    double Speed = 1;
    double SpeedSetting = 1;

    // Payload Definitions
    public DcMotor lift;
    public DcMotor slide;
    public Servo grabStone;
    public Servo wrist;
    public Servo FoundationMoverL;
    public Servo FoundationMoverR;

    //Lift Positioning Definitions
    double LiftPower = 1;
    double upstep = 0;
    double upcount = 0;
    double claw_status = 1;
    double height = 0; //Tells what level the lift will go to
    double CurrentHeight = 0;
    Boolean NeedFoundation = false;
        //Lift Encoder Definitions
        static final double     COUNTS_PER_LEVEL    = 250 ;    // eg: REV Core Hex Motor Encoder

    // End of Definitions

    @Override
    public void runOpMode() {

        /** Drive Train initialization **/

        //Motor Initialization
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");

        //Motor Direction Initialization
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        //Initialized Telemetry
        telemetry.addData("Drive Train: ", "Initialized");
        telemetry.update();

        /** End of Drive Train Initialization **/

        /** Payload Initialization **/

        //Lift Initialization
        lift = hardwareMap.dcMotor.get("LT");
        lift.setDirection(DcMotor.Direction.FORWARD);

        //Slide Initialization
        slide = hardwareMap.dcMotor.get("SL");
        slide.setDirection(DcMotor.Direction.FORWARD);

        //Hand Initialization
        grabStone = hardwareMap.servo.get("GS");
        wrist = hardwareMap.servo.get("W");
        //wrist.setPosition(0.5); // Center the wrist

        //Foundation Mover Initialization
        FoundationMoverL = hardwareMap.servo.get("GL");
        FoundationMoverR = hardwareMap.servo.get("GR");

        // Encoder initialization
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        //Stops and Resets Encoders
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Tells Robots to Reset Encoders
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Initialized Telemetry
        telemetry.addData("Payload: ", "Initialized");
        telemetry.update();

        /** End of Payload Initialization **/

        telemetry.addData("Status: ", "Initialized");
        telemetry.addData("> Press Play to Start ", "TeleOp");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            /** Gamepad 1 Controls (Drive Train) ==> **/

            /**Mechanum Drive Controls**/
            // left stick controls direction
            // right stick X controls rotation
            float gamepad1LeftY = gamepad1.left_stick_y;
            float gamepad1LeftX = -gamepad1.left_stick_x;
            float gamepad1RightX = -gamepad1.right_stick_x;

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

            //Speed Controls
            if (gamepad1.a) {
                SpeedSetting = 1; // Tells the code that we are on full speed
                Speed = 1; // Full Speed
            }

            if (gamepad1.b) {
                SpeedSetting = .75; // Tells the code that we are on three quarter speed
                Speed = .75; // Three Quarter Speed
            }

            if (gamepad1.x) {
                SpeedSetting = .5; // Tells the code that we are on half speed
                Speed = .50; // Half Speed
            }

            if (gamepad1.y) {
                SpeedSetting = .25; // Tells the code that we are on quarter speed
                Speed = .25; // Quarter Speed
            }
            //End of Speed Controls
            /** End of Mechanum Drive Controls **/

            /** Foundation Mover Controls **/
            // Lowering Foundation Movers
            if (gamepad1.dpad_down) {
                FoundationMoverL.setPosition(1);
                FoundationMoverR.setPosition(1);
            }
            // End of Lowering Foundation Movers

            // Raising Foundation Movers
            if (gamepad1.dpad_up) {
                FoundationMoverL.setPosition(0);
                FoundationMoverR.setPosition(0);
            }
            // End of Raising Foundation Movers
            /** End of Foundation Mover Controls **/

            /** Speed Brake Controls **/
            if (gamepad1.left_trigger > .3) { //While the left trigger is being held down
                Speed = .25; //Sets the speed to quarter speed
            }
            if (gamepad1.left_trigger <= .3) { //While the left trigger is not being held down
                if (SpeedSetting == .25) { //If the speed we were previously on was .25
                    Speed = .25; //Set the speed to .25
                }

                if (SpeedSetting == .5) { //If the speed we were previously on was .5
                    Speed = .5; //Set the speed to .5
                }

                if (SpeedSetting == .75) { //If the speed we were previously on was .75
                    Speed = .75; //Set the speed to .75
                }

                if (SpeedSetting == 1) { //If the speed we were previously on was 1
                    Speed = 1; //Set the speed to 1
                }
            }
            /** End of Speed Brake Controls **/

            /** Gamepad 2 Controls (Payload) ==> **/

            /** Lift System Controls **/
            // Moving The Lift Upward
            if (gamepad2.left_bumper && (CurrentHeight > 0)) {
                height = -CurrentHeight;
                sleep(150);
                CurrentHeight = 0;
            }
            // End of Moving the Lift Upward

            // Zeroing the Lift
            if (!gamepad2.left_bumper && !gamepad2.right_bumper) {
                lift.setPower(.001); //Holds lift in place
            }
            // End of Zeroing the lift

            // Moving the Lift Downward
            if (gamepad2.right_bumper && (CurrentHeight < 7)) {
                if (CurrentHeight == 0) { //If we need to lift past the foundation
                    NeedFoundation = true;
                }
                height++;
                CurrentHeight++;
                sleep(200);
            }

            //Start lift
            if (gamepad2.right_trigger > .3) {
                encoderLift(1,height);
            }

            //Foundation Override
            if (gamepad2.x) {
                NeedFoundation = false;
            }

            /** End of Lift System Controls **/

            /** Lift Speed Brake Controls **/
            if (gamepad2.left_trigger > .3) { //While the left trigger is being held down
                LiftPower = .25; //Sets the speed to quarter speed
            }
            if (gamepad2.left_trigger <= .3) { //While the left trigger is not being held down
                LiftPower = 1; //Sets the speed to full
            }
            /** End of Lift Speed Brake Controls **/

            /** Slide System Controls **/
            // Moving The Slide Outward
            if (gamepad2.left_stick_y > .25) {
                slide.setPower(-1); //Set power to the slide
            }
            // End of Moving the Slide Outward

            // Zeroing the Slide
            if (gamepad2.left_stick_y < .25 && gamepad2.left_stick_y > -.25) {
                slide.setPower(0); //Stops power to the slid
            }
            // End of Zeroing the Slide

            // Moving the Slide Inward
            if (gamepad2.left_stick_y < -.25) {
                slide.setPower(1); //Set power to the slide
            }
            // End of Moving the Slide Inward
            /** End of Slide System Controls **/

            /** Hand System Controls **/
            // Wrist Controls
            if (gamepad2.right_stick_x > 0.1) {
                wrist.setPosition(wrist.getPosition() + .0025); //Move the wrist right
            }

            if (gamepad2.right_stick_x < -0.1) {
                wrist.setPosition(wrist.getPosition() - .0025); //Move the wrist Left
            }

            if (gamepad2.right_stick_x < -0.1 && gamepad2.right_stick_x > 0.1) {
                wrist.setPosition(wrist.getPosition()); //Set the wrist to the current position
            }

            if (gamepad2.dpad_up) {
                wrist.setPosition(0.4);
            }
            // End of Wrist controls

            // Grabber Controls
            if (gamepad2.a) {
                grabStone.setPosition(0); //Clamp down with the grabber
            }

            if (gamepad2.b) {
                grabStone.setPosition(.3); //Release the grabber
            }
            // End of Grabber Controls
            /** End of Hand System Control **/

            /** Beginning of Telemetry **/
            // Lift Telemetry
            telemetry.addData("Height Change:", height);
            telemetry.addData("Current Height: ",CurrentHeight);

            //Wrist Telemetry
            telemetry.addData("Speed", Speed);
            telemetry.addData("Wrist", wrist.getPosition());
            telemetry.update();

            /** End of Telemetry **/
        }
    }

    public void encoderLift(double LiftSpeed, double levels) {
        int newLiftTarget;

        if (opModeIsActive()) {

            newLiftTarget = lift.getCurrentPosition() + (int) (levels * COUNTS_PER_LEVEL);

            if (NeedFoundation) {
                newLiftTarget = (lift.getCurrentPosition() + (int) (levels * COUNTS_PER_LEVEL) + 100);
                NeedFoundation = false;
            }

            lift.setTargetPosition(newLiftTarget);

            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            lift.setPower(LiftSpeed);

            while (opModeIsActive() && lift.isBusy()) {

                telemetry.addData("lift position", lift.getCurrentPosition());
                telemetry.update();

            }

            lift.setPower(0);

            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            height = 0;

        }
    }
}
