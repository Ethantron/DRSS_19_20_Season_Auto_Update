/** Pre-initialization code **/

// Defines where the code is located
package org.firstinspires.ftc.teamcode.Competition_Code.TelyOp;

// Imports codes that the robot uses
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;        // Imports Linear Operation mode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;              // Imports Driver Controled mode
import com.qualcomm.robotcore.hardware.DcMotor;                     // Imports motor definitions
import com.qualcomm.robotcore.hardware.Servo;                       // Imports servo definitions
import com.qualcomm.robotcore.util.ElapsedTime;                     // Imports timer definitions
import com.qualcomm.robotcore.util.Range;                           // Imports motor ranes definitions

// Defines robot display name
@TeleOp (name = "Galileo", group= "TeleOp")                         // Sets codes mode to TelyOp and sets the display name for the code
public class Galileo extends LinearOpMode {                         // Sets the codes name and sets it to Linear OpMode

    /**Robot definitions **/

    // Drivetrain definitions
        // Drive motor definitions
        public DcMotor motorFrontRight;                                 // Defines the front right motor
        public DcMotor motorFrontLeft;                                  // Defines the front left motor
        public DcMotor motorBackRight;                                  // Defines the back right motor
        public DcMotor motorBackLeft;                                   // Defines the back left motor

        // Mechanum Definitions
        double frontRight;                                              // Sets the double "frontRight"               | Helps with motor calculations
        double frontLeft;                                               // Sets the double "fronLeft"                 | Helps with motor calculations
        double backRight;                                               // Sets the double "backRight"                | Helps with motor calculations
        double backLeft;                                               // Sets the double "backLeft                  | Helps with motor calculations
        double speed = 1;                                               // Sets the double "speed" to one             | Controls overall speed of the drive motors
        double speedSetting = 1;                                        // Sets the double "speedSetting" to one      | Allows us to remember what the previous speed was

        public Servo foundationMoverL;                                  // Defines the left foundation servo
        public Servo foundationMoverR;                                  // Defines the right foundation servo
    //End drivetrain definitions

    //Payload definitions
        // Payload motor and servo definitions
        public DcMotor lift;                                            // Defines the lift motor
        public DcMotor slide;                                           // Defines the slide motor
        public Servo grabStone;                                         // Defines the stone grabber servo
        public Servo wrist;                                             // Defines the wrist servo


        //Lift positioning definitions
        double liftPower = 1;                                           // Sets the double "liftPower" to one         | Defines how fast the lift moves
        double height = 0;                                              // Sets the double "height" to zero           | Defines the level the lift should move to
        double currentHeight = 0;                                       // Sets the double "currentHeight" to zero    | Counts what level the lift is on
        boolean needFoundation = false;                                 // Sets the boolean "needFoundation" to false | Defines wheter the lift needs to account for the foundations
        static final double COUNTS_PER_LEVEL = 300;                     // Sets the double "COuNTS_PER_LEVEL" to 300 | Defines how long the lift needs to run to go up one level | About 55  counts per inch
    // End payload definitions

    // Misc definitions
        ElapsedTime ResetTime = new ElapsedTime();                      // Enables a built in timer 
    //End misc definitions

    /** End robot definitions **/
    
/** End pre-initialization code **/

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

        //Foundation Mover Initialization
        foundationMoverL = hardwareMap.servo.get("GL");
        foundationMoverR = hardwareMap.servo.get("GR");

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

        /** End of payload initialization **/

        /** Encoder initalization **/
        // Encoder initialization
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        //Stops and Resets Encoders
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Tells Robots to Reset Encoders
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /** End encoder initalization **/

        //Initializeation Telemetry
        telemetry.addData("Drive Train: ", "Initialized");
        telemetry.addData("Payload: ", "Initialized");
        telemetry.addData("Status: ", "Ready");
        telemetry.addData("Press Play to Start ", "TeleOp");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            /** Gamepad 1 controls (drive train) ==> **/

            /**Mechanum drive controls**/
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
            motorFrontRight.setPower(frontRight);
            motorFrontLeft.setPower(frontLeft);
            motorBackLeft.setPower(backLeft);
            motorBackRight.setPower(backRight);

            // sets speed
            frontRight = Range.clip(Math.pow(FrontRight, 3), -speed, speed);
            frontLeft = Range.clip(Math.pow(FrontLeft, 3), -speed, speed);
            backRight = Range.clip(Math.pow(BackRight, 3), -speed, speed);
            backLeft = Range.clip(Math.pow(BackLeft, 3), -speed, speed);

            //speed Controls
            if (gamepad1.left_trigger > .3) { //While the left trigger is being held down
                speed = .25; //Sets the speed to quarter speed
            } else {

                if (gamepad1.a || speedSetting == 1) {
                    speedSetting = 1; // Tells the code that we are on full speed
                    speed = 1; // Full speed
                }

                if (gamepad1.b || speedSetting == .75) {
                    speedSetting = .75; // Tells the code that we are on three quarter speed
                    speed = .75; // Three Quarter speed
                }

                if (gamepad1.x || speedSetting == .5) {
                    speedSetting = .5; // Tells the code that we are on half speed
                    speed = .50; // Half speed
                }

                if (gamepad1.y || speedSetting == .25) {
                    speedSetting = .25; // Tells the code that we are on quarter speed
                    speed = .25; // Quarter speed
                }
            }
            //End of speed Controls
            /** End of mechanum drive controls **/

            /** Foundation mover controls **/
            // Lowering Foundation Movers
            if (gamepad1.dpad_down) {
                foundationMoverL.setPosition(1);
                foundationMoverR.setPosition(1);
            }
            // End of Lowering Foundation Movers

            // Raising Foundation Movers
            if (gamepad1.dpad_up) {
                foundationMoverL.setPosition(0);
                foundationMoverR.setPosition(0);
            }
            // End of Raising Foundation Movers
            /** End of foundation mover controls **/

            /** End of gamepad 1 controls (drive train) **/


            /** Gamepad 2 controls (payload) ==> **/

            /** Lift system controls **/
            // Moving The Lift Upward
            if (gamepad2.left_bumper && (currentHeight > 0)) {
                height = -currentHeight;
                sleep(150);
                currentHeight = 0;
            }
            // End of Moving the Lift Upward

            // Zeroing the Lift
            if (!gamepad2.left_bumper && !gamepad2.right_bumper) {
                lift.setPower(.001); //Holds lift in place
            }
            // End of Zeroing the lift

            // Moving the Lift Downward
            if (gamepad2.right_bumper && (currentHeight < 7)) {
                if (currentHeight == 0) { //If we need to lift past the foundation
                    needFoundation = true;
                }
                height++;
                currentHeight++;
                sleep(200);
            }

            //Start lift
            if (gamepad2.right_trigger > .3) {
                encoderLift(1, height);
            }

            //Foundation Override
            if (gamepad2.x) {
                needFoundation = false;
            }

            /** End of lift system controls **/

            /** Lift speed brake controls **/
            if (gamepad2.left_trigger > .3 && gamepad2.left_stick_y > .3) { //While the left trigger is being held down
                lift.setPower(1);
            }

            if (gamepad2.left_trigger > .3 && (gamepad2.left_stick_y < .3 && gamepad2.left_stick_y > -.3)) { //While the left trigger is being held down
                lift.setPower(0.001);
            }

            if (gamepad2.left_trigger > .3 && gamepad2.left_stick_y < -.3) { //While the left trigger is being held down
                lift.setPower(-1);
            }
            /** End of lift speed brake controls **/

            /** Slide system controls **/
            // Moving The Slide Outward
            if (gamepad2.left_stick_y > .25 && gamepad2.left_trigger < .3) {
                slide.setPower(-1); //Set power to the slide
            }
            // End of Moving the Slide Outward

            // Zeroing the Slide
            if ((gamepad2.left_stick_y < .25 && gamepad2.left_stick_y > -.25) && gamepad2.left_trigger <= .3) {
                slide.setPower(0); //Stops power to the slid
            }
            // End of Zeroing the Slide

            // Moving the Slide Inward
            if (gamepad2.left_stick_y < -.25 && gamepad2.left_trigger <= .3) {
                slide.setPower(1); //Set power to the slide
            }
            // End of Moving the Slide Inward
            /** End of slide system controls **/

            /** Hand system controls **/
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
                encoderPlace(1, 50);
            }

            if (gamepad2.y) {
                grabStone.setPosition(0); //Clamp down with the grabber
            }

            if (gamepad2.b) {
                grabStone.setPosition(.3); //Release the grabber
            }
            // End of Grabber Controls
            /** End of hand system control **/

            /** Beginning of telemetry **/
            // Lift Telemetry
            telemetry.addData("Height Change:", height);
            telemetry.addData("Current Height: ", currentHeight);

            //Wrist Telemetry
            telemetry.addData("Speed", speed);
            telemetry.addData("Wrist", wrist.getPosition());
            telemetry.update();

            /** End of telemetry **/
        }
    }

    public void encoderLift(double LiftSpeed, double levels) {
        int newLiftTarget;

        if (opModeIsActive()) {
            newLiftTarget = (lift.getCurrentPosition() + (int) (levels * COUNTS_PER_LEVEL)) + 50;

            if (needFoundation) {
                newLiftTarget = (lift.getCurrentPosition() + (int) (levels * COUNTS_PER_LEVEL)) + 150;
                needFoundation = false;
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

    public void encoderPlace(double DropSpeed, double distance) {
        int newDropTarget;

        if (opModeIsActive()) {
            newDropTarget = lift.getCurrentPosition() - (int) (distance);
            lift.setTargetPosition(newDropTarget);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(DropSpeed);

            while (opModeIsActive() && lift.isBusy()) {
                telemetry.addData("lift position", lift.getCurrentPosition());
                telemetry.update();
            }

            lift.setPower(0);
            grabStone.setPosition(.3);
            sleep(100);

            //Return the lift back up
            newDropTarget = lift.getCurrentPosition() + (int) (distance);
            lift.setTargetPosition(newDropTarget);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(DropSpeed);

            while (opModeIsActive() && lift.isBusy()) {
                telemetry.addData("lift position", lift.getCurrentPosition());
                telemetry.update();
            }

            lift.setPower(0);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}