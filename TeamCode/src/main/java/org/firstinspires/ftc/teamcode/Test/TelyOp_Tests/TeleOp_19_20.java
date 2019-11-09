package org.firstinspires.ftc.teamcode.Test.TelyOp_Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "19-20 TeleOp", group= "TeleOp")
public class TeleOp_19_20 extends OpMode {

    // Robot definitions

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

    // Payload Definitions
    public DcMotor lift;
    public DcMotor slide;
    public Servo grabStone;
    public Servo wrist;
    public Servo FoundationMoverL;
    public Servo FoundationMoverR;

    //Lift Positioning Definitions
    double upstep = 0;
    double upcount = 0;

    //Lift Encoder Definitions
    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // eg: REV Core Hex Motor Encoder
    double step = 0;

    double claw_status = 1;

    // End of Definitions

    @Override
    public void init() {

        /** Drive Train initialization **/

        //Motor Initialization
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");

        //Motor Drection Initialization
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
            //Lift Encoder Initialization
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Stop and reset the lift encoders
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Tell motor to run using encoders

        //Slide Initiailization
        slide = hardwareMap.dcMotor.get("SL");
        slide.setDirection(DcMotor.Direction.FORWARD);

        //Hand Initiailization
        grabStone = hardwareMap.servo.get("GS");
        wrist = hardwareMap.servo.get("W");
        //wrist.setPosition(0.5); // Center the wrist

        //Foundation Mover Initialization
        FoundationMoverL = hardwareMap.servo.get("GL");
        FoundationMoverR = hardwareMap.servo.get("GR");

        //Initialized Telemetry
        telemetry.addData("Payload: ", "Initialized");
        telemetry.update();

        /** End of Payload Initiailization **/

        telemetry.addData("Status: ", "Initialized");
        telemetry.addData("> Press Play to Start ", "TeleOp");
        telemetry.update();
    }

    @Override
    public void loop() {

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
        if (gamepad1.a){
            Speed = 1; // Full Speed
        }

        if (gamepad1.b){
            Speed = .75; // Three Quarter Speed
        }

        if (gamepad1.x){
            Speed = .50; // Half Speed
        }

        if (gamepad1.y){
            Speed = .25; // Quarter Speed
        }
        //End of Speed Controls
        /** End of Mechanum Drive Controls **/

        /** Foundation Mover Controls **/
        // Lowering Foundation Movers
        if (gamepad1.dpad_down){
            FoundationMoverL.setPosition(1);
            FoundationMoverR.setPosition(1);
        }
        // End of Lowering Foundation Movers

        // Raising Foundation Movers
        if (gamepad1.dpad_up){
            FoundationMoverL.setPosition(0);
            FoundationMoverR.setPosition(0);
        }

        telemetry.addData("Speed", Speed);
        telemetry.addData("Wrist", wrist.getPosition());
        telemetry.update();
        // End of Raising Foundation Movers
        /** End of Foundation Mover Controls **/

        /** Gamepad 2 Controls (Payload) ==> **/

        /** Lift System Controls **/
        // Moving The Lift Upward
        if (gamepad2.left_trigger > .25) {
            lift.setPower(-1); //Set power to the slide
        }
        // End of Moving the Lift Upward

        // Zeroing the Lift
        if (gamepad2.left_trigger < .25 && gamepad2.right_trigger < .25) {
            lift.setPower(.0002); //Holds lift in place
        }
        // End of Zeroing the lift

        // Moving the Lift Downward
        if (gamepad2.right_trigger > .25) {
            lift.setPower(1); //Set power to the slide
        }
        //End of Moving the Lift Downward

        //Encoder Lift Testing
        if (gamepad2.right_bumper) {
            encoderLift(.5, 1); //Move up one revolution at .5 speed
        }
        //End of Encoder Lift Testing
        /** End of Lift System Controls **/

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

        if (gamepad2.dpad_up){
            wrist.setPosition(0.4);
        }
        // End of Wrist controls

        // Grabber Controls
        if (gamepad2.a){
            grabStone.setPosition(0); //Clamp down with the grabber
        }

        if (gamepad2.b) {
            grabStone.setPosition(.75); //Release the grabber
        }
        // End of Grabber Controls
        /** End of Hand System Control **/
    }

    public void encoderLift(double speed, double Revolutions) {

            int newLiftTarget;
            int moveCounts;

            moveCounts = (int) (Revolutions * COUNTS_PER_MOTOR_REV);

        if (step == 0) { //Define lift target positon
            newLiftTarget = lift.getCurrentPosition() + moveCounts;
            lift.setTargetPosition(newLiftTarget);
            step++;
        }

        if (step == 1) { //Set lift to run to position
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            step++;
        }

        if (step == 2) { //Set power to lift
            lift.setPower(speed);
            step++;
        }

        if (step == 3) { //While the lift is running to position
            if (lift.isBusy()) {
                telemetry.addData("Current position", lift.getCurrentPosition());
                telemetry.update();
            }
            else { //Once it reaches position
                step++;
            }
        }

        if (step == 4) { //Stop Lift motors
            lift.setPower(0);
            step++;
        }

        if (step == 5) { //Set motors to run with encoders
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            step++;
        }

        if (step == 6) {
            loop();
        }
    }
}
