/** Pre-initialization code **/

// Defines where the code is located
package org.firstinspires.ftc.teamcode.Competition_Code.TelyOp;

// Imports codes that the robot uses
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;    // Imports Linear Operation mode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;          // Imports Driver Controled mode
import com.qualcomm.robotcore.hardware.DcMotor;                 // Imports motor definitions
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;                   // Imports servo definitions
import com.qualcomm.robotcore.util.ElapsedTime;                 // Imports timer definitions
import com.qualcomm.robotcore.util.Range;                       // Imports motor ranes definitions

// Defines robot display name
@TeleOp (name = "Galileo", group= "TeleOp")     // Sets codes mode to TelyOp and sets the display name for the code
public class Galileo extends LinearOpMode {     // Sets the codes name and sets it to Linear OpMode

    /**Robot definitions **/

    // Drivetrain definitions
        // Drive motor definitions
        public DcMotor motorFrontRight;     // Defines the front right motor
        public DcMotor motorFrontLeft;      // Defines the front left motor
        public DcMotor motorBackRight;      // Defines the back right motor
        public DcMotor motorBackLeft;       // Defines the back left motor

        // Mechanum Definitions
        double frontRight;          // Sets the double "frontRight"             | Helps with motor calculations
        double frontLeft;           // Sets the double "fronLeft"               | Helps with motor calculations
        double backRight;           // Sets the double "backRight"              | Helps with motor calculations
        double backLeft;            // Sets the double "backLeft                | Helps with motor calculations
        double speed = 1;           // Sets the double "speed" to one           | Controls overall speed of the drive motors
        double speedSetting = 1;    // Sets the double "speedSetting" to one    | Allows us to remember what the previous speed was
        boolean QSB = false;        // Sets the boolean "QSB" to false          | Allows us to know if he quarter speed brake is on
        boolean HSB = false;        // Sets the boolean "HSB" to false          | Allows us to know if the half speed brake is on

        public Servo foundationMoverL;      // Defines the left foundation servo
        public Servo foundationMoverR;      // Defines the right foundation servo
        boolean foundationMoverPos = true;         // Defines the foundation mover positon
    //End drivetrain definitions

    //Payload definitions
        // Payload motor and servo definitions
        public DcMotor lift;        // Defines the lift motor
        public DcMotor slide;       // Defines the slide motor
        public Servo grabStone;     // Defines the stone grabber servo
        public Servo wrist;         // Defines the wrist servo
        public DigitalChannel stoneButton; // Defines the Stone Button on the grabber


        //Lift positioning definitions
        double liftPower = 1;                           // Sets the double "liftPower" to one           | Defines how fast the lift moves
        double height = 0;                              // Sets the double "height" to zero             | Defines the level the lift should move to
        double currentHeight = 0;                       // Sets the double "currentHeight" to zero      | Counts what level the lift is on
        boolean needFoundation = false;                 // Sets the boolean "needFoundation" to false   | Defines wheter the lift needs to account for the foundations
        static final double COUNTS_PER_LEVEL = 300;     // Sets the double "COuNTS_PER_LEVEL" to 300    | Defines how long the lift needs to run to go up one level | About 55  counts per inch
    // End payload definitions

    // Misc definitions
        ElapsedTime ResetTime = new ElapsedTime();      // Enables a built in timer
    //End misc definitions

    /** End robot definitions **/
    
/** End pre-initialization code **/

    @Override
    public void runOpMode() {       // Begins running the initialization code when the "int" button is pressed

        /** Drive Train initialization **/

        //Motor Initialization
        motorFrontRight = hardwareMap.dcMotor.get("FR");    // Initializes the front right motors name for configuration
        motorFrontLeft = hardwareMap.dcMotor.get("FL");     // Initializes the front left motors name for configuration
        motorBackRight = hardwareMap.dcMotor.get("BR");     // Initializes the back right motors name for configuration
        motorBackLeft = hardwareMap.dcMotor.get("BL");      // Initializes the back left motors name for configuration

        //Motor Direction Initialization
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);    // Sets the front right motors direction to reverse
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);     // Sets the front left motors direction to reverse
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);     // Sets the back right motors direction to reverse
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);      // Sets the back left motors direction to reverse

        //Foundation Mover Initialization
        foundationMoverR = hardwareMap.servo.get("GR");     // Initializes the right foundation movers name for configuration
        foundationMoverL = hardwareMap.servo.get("GL");     // Initializes the left foundation movers name for configuration
        foundationMoverR.setPosition(1);                    // Sets the right foundation mover to point down
        foundationMoverL.setPosition(1);                    // Sets the left foundation mover to point down
        /** End of Drive Train Initialization **/

        /** Payload Initialization **/

        //Lift Initialization
        lift = hardwareMap.dcMotor.get("LT");           // Initializes the lift motors name for configuration
        lift.setDirection(DcMotor.Direction.FORWARD);   // Sets the lift motors direction to forward

        //Slide Initialization
        slide = hardwareMap.dcMotor.get("SL");          // Initializes the slide motors name for configuration
        slide.setDirection(DcMotor.Direction.FORWARD);  // Sets the slide motors direction to forward

        //Hand Initialization
        grabStone = hardwareMap.servo.get("GS");    // Initializes the grabber servos name for configuration
        wrist = hardwareMap.servo.get("W");         // Initializes the wrist servos name for configuration

        //Stone Button Sensor Initialization
        stoneButton = hardwareMap.get(DigitalChannel.class, "stone_button"); // Initializes the stone button name for configuration
        stoneButton.setMode(DigitalChannel.Mode.INPUT);                                 // Initializes the mode of the button
        /** End of payload initialization **/

        /** Encoder initialization **/
        // Encoder initialization
        telemetry.addData("Status", "Resetting Encoders");  // Adds telemetry to the screen to show that the robot is resetting the encoders
        telemetry.update();                                       // Tells the telemetry to display on the phone

        //Stops and Resets Encoders
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // Tells the lift motor to reset its encoder

        //Tells Robots to Reset Encoders
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);    // Tells the lift motor to ru using its encoder

        /** End encoder initalization **/

        //Initializeation Telemetry
        telemetry.addData("Drive Train: ", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
        telemetry.addData("Payload: ", "Initialized");          // Adds telemetry to the screen to show that the payload is initialized
        telemetry.addData("Status: ", "Ready");                 // Adds telemetry to the screen to show that the robot is ready
        telemetry.addData("Press Play to Start ", "TeleOp");    // Adds telemetry to the screen to tell the drivers that the code is ready to start
        telemetry.update();                                           // Tells the telemetry to display on the phone

        waitForStart();     // Tells the code to wait here until the drivers have pressed the start button

        while (opModeIsActive()) {      // Do the following after the start button has been pressed and until the stop button is pressed

            /** Gamepad 1 controls (drive train) ==> **/

            /**Mechanum drive controls**/
            // left stick controls direction
            // right stick X controls rotation
            float gamepad1LeftY = gamepad1.left_stick_y;        // Sets the gamepads left sticks y position to a float so that we can easily track the stick
            float gamepad1LeftX = -gamepad1.left_stick_x;       // Sets the gamepads left sticks x position to a float so that we can easily track the stick
            float gamepad1RightX = -gamepad1.right_stick_x;     // Sets the gamepads right sticks x position to a float so that we can easily track the stick

            // Mechanum formulas
            double FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;     // Combines the imputs of the sticks to clip their output to a value between 1 and -1
            double FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;     // Combines the imputs of the sticks to clip their output to a value between 1 and -1
            double BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;      // Combines the imputs of the sticks to clip their output to a value between 1 and -1
            double BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;      // Combines the imputs of the sticks to clip their output to a value between 1 and -1

            // sets speed
            frontRight = Range.clip(Math.pow(FrontRight, 3), -speed, speed);    // Slows down the motor and sets its max/min speed to the double "speed"
            frontLeft = Range.clip(Math.pow(FrontLeft, 3), -speed, speed);      // Slows down the motor and sets its max/min speed to the double "speed"
            backRight = Range.clip(Math.pow(BackRight, 3), -speed, speed);      // Slows down the motor and sets its max/min speed to the double "speed"
            backLeft = Range.clip(Math.pow(BackLeft, 3), -speed, speed);        // Slows down the motor and sets its max/min speed to the double "speed"

            //speed Controls
            if (gamepad1.left_trigger > .3) {   // Do the following while the left trigger is being held down
                speed = .25;                    // Sets the speed to quarter speed
                QSB = true;
                HSB = false;
            }
            else if (gamepad1.right_trigger > .3 && gamepad1.left_trigger < .3){   // Do the following  while the right trigger is held down and the left trigger is not
                speed = .5;                     // Sets the speed to half speed
                QSB = false;
                HSB =true;
            }
            else if (gamepad1.right_trigger < .3 && gamepad1.left_trigger < .3){    // Do the following while the left trigger is not being held down

                QSB =false;
                HSB =false;

                if (gamepad1.a || speedSetting == 1) {      // Do the following if the "a" button has been pressed or the double "speedSetting" is equal to 1
                    speedSetting = 1;                       // Tells the code that we are on full speed
                    speed = 1;                              // Sets the speed to 1
                }

                if (gamepad1.b || speedSetting == .75) {    // Do the following if the "b" button has been pressed or the double "speedSetting" is equal to .75
                    speedSetting = .75;                     // Tells the code that we are on three quarter speed
                    speed = .75;                            // Sets the speed to .75
                }

                if (gamepad1.x || speedSetting == .5) {     // Do the following if the "x" button has been pressed or the double "speedSetting" is equal to .5
                    speedSetting = .5;                      // Tells the code that we are on half speed
                    speed = .50;                            // Sets the speed to .5
                }

                if (gamepad1.y || speedSetting == .25) {    // Do the following if the "y" button has been pressed or the double "speedSetting" is equal to .25
                    speedSetting = .25;                     // Tells the code that we are on quarter speed
                    speed = .25;                            // Sets the speed to .25
                }
            }

            motorFrontRight.setPower(frontRight);   // Sets the front right motors speed to the previous double
            motorFrontLeft.setPower(frontLeft);     // Sets the front left motors speed to the previous double
            motorBackRight.setPower(backRight);     // Sets the back right motors speed to the previous double
            motorBackLeft.setPower(backLeft);       // Sets the back left motors speed to the previous double
                //End of speed Controls
            /** End of mechanum drive controls **/

            /** Foundation mover controls **/
            // Lowering Foundation Movers
            if (gamepad1.dpad_down) {               // Do the following if the "down" button has been pressed
                foundationMoverR.setPosition(1);    // Sets the right foundation mover to point down
                foundationMoverL.setPosition(1);    // Sets the left foundation mover to point down
                foundationMoverPos = true;          // Tells the telemetry that the foundation movers are down

            }
            // End of Lowering Foundation Movers

            // Raising Foundation Movers
            if (gamepad1.dpad_up) {                 // Do the following if the "up" button has been pressed
                foundationMoverR.setPosition(0);    // Sets the right foundation mover to point up
                foundationMoverL.setPosition(0);    // Sets the left foundation mover to point up
                foundationMoverPos = false;         // Tells the telemetry that the foundation movers are up
            }
            // End of Raising Foundation Movers
            /** End of foundation mover controls **/

            /** End of gamepad 1 controls (drive train) **/


            /** Gamepad 2 controls (payload) ==> **/

            /** Automatic lift controls **/
            // Moving The Lift Upward
            if (gamepad2.left_bumper && (currentHeight > 0)) { // Do the following if the left bumper is pressed and the current height os greater than 0
                height--;                                      // Sets "height" to -currentHeight
                sleep(150);                        // Tells the code to wait 150 milliseconds
                currentHeight--;                               // sets "currentHeight" to height
            }
            // End of Moving the Lift Upward

            // Zeroing the Lift
            if (!gamepad2.left_bumper && !gamepad2.right_bumper) {  // Do the following if neither bumper is pressed
                lift.setPower(.001);                                // Tells the lift to hold in place by setting the motor power to .001
            }
            // End of Zeroing the lift

            // Moving the Lift Downward
            if (gamepad2.right_bumper && (currentHeight < 7)) {     // Do the following if the right bumper has been pressed and the current height is greater than 7
                if (currentHeight == 0) {                           // Do the following if the current height is 0
                    needFoundation = true;                          // Sets "needFoundation" to true
                }
                height++;                                           // Adds 1 to "height"
                currentHeight++;                                    // Adds 1 to "currentHeight"
                sleep(200);                             // Tells the code to wait 200 milliseconds
            }

            //Start lift
            if (gamepad2.x) {                     // Do the following if the "x" button is pressed
                encoderLift(1, height);  // Tells the lift to move up to set height
            }

            //Foundation Override
            if (gamepad2.dpad_down) {           // Do the following if the "down" button is pressed
                needFoundation = false;         // Overrides need foundation to false
            }

            /** End of automatic lift controls **/

            /** Manual lift controls **/
            if (gamepad2.right_trigger > 0.3) {         // Tells the code ro do the following if the right trigger is pressed
                lift.setPower(gamepad2.right_trigger);  // Sets the lift motor speed to 1
            } 
            
            else if (gamepad2.left_trigger > 0.3) {     // Do the following if the left trigger is held down
                lift.setPower(-gamepad2.left_trigger);  // Sets the lift motor speed to -1
            } 
            
            else if (gamepad2.right_trigger < 0.3 && gamepad2.left_trigger < 0.3){ // Do the following if neither trigger is held down
                lift.setPower(.001);                                               // Tells the lift to hold in place by setting the motor power to .001
            }
            /** End of manual lift controls **/

            /** Slide system controls **/
            if (gamepad2.left_stick_y > .25) {  // Do the following if the left stick is up
                slide.setPower(-1);             //Sets the slide motors speed to -1
            }
            
            if (gamepad2.left_stick_y < .25 && gamepad2.left_stick_y > -.25) {  // Do the following if the left stick is centered
                slide.setPower(0);                                              // Sets the slide motors speed to 0
            }

            if (gamepad2.left_stick_y < -.25 && gamepad2.left_trigger <= .3) {  // Do the followin if the left stick is down
                slide.setPower(1);                                              // Sets the slide motors speed to 1
            }
            /** End of slide system controls **/

            /** Hand system controls **/
            // Wrist Controls
            if (gamepad2.right_stick_x > 0.1) {                 // Do the following if the right stick is right
                wrist.setPosition(wrist.getPosition() + .0025); // Turns the wrist right
            }

            if (gamepad2.right_stick_x < -0.1) {                // Do the following if the right stick is left
                wrist.setPosition(wrist.getPosition() - .0025); // Turns the wrist left
            }

            if (gamepad2.right_stick_x < -0.1 && gamepad2.right_stick_x > 0.1) {    // Do the following if the right stick is centered
                wrist.setPosition(wrist.getPosition());                             // Sets the wrist to its current position
            }

            if (gamepad2.dpad_up) {     // Do the following if the "up" button on the dpad is pressed
                wrist.setPosition(0.4); // Centers the wrist
            }
            // End of Wrist controls

            // Grabber Controls
            if (gamepad2.a) {                              // Do the following if the "a" button is pressed
                encoderPlace(1, 50);    // Moves the lift slightly down and opens the grabber
            }

            if (gamepad2.y) {               // Do the following if the "y" button is pressed
                grabStone.setPosition(0);   // Closes the grabber
            }

            if (gamepad2.b) {               // Do the following if the "b" button is pressed
                grabStone.setPosition(.6);  // Opens the grabber
            }

            if (stoneButton.getState() == true) { // Do the following if the stone button is pressed
                grabStone.setPosition(0);        // Closes the grabber
            } else {                              // Do the following if the stone button is not pressed
                grabStone.setPosition(.6);         // Opens the grabber
            }
            // End of Grabber Controls
            /** End of hand system control **/

            /** End of gamepad 2 controls (payload) **/

            /** Beginning of telemetry **/
            telemetry.addData("Payload ", "Telemetry");          // Adds telemetry to the screen to show that the following telemetry is for the drivetrain

            // Lift telemetry
            telemetry.addData("Desired Height:", height);           // Adds telemetry to the screen to show the desired height of the lift
            telemetry.addData("Current Height: ", currentHeight);   // Adds telemetry to the screen to show the current height of the lift

            //Wrist telemetry
            telemetry.addData("Wrist", wrist.getPosition());        // Adds telemetry to the screen to show the current position of the wrist

            telemetry.addData("", "");                          // Adds a space in the telemetry

            telemetry.addData("Drivetrain ", "Telemetry");      // Adds telemetry to the screen to show that the following telemetry is for the drivetrain

            // Foundation grabber telemetry

            if (!foundationMoverPos) {                                // If the foundation movers are up
                telemetry.addData("Foundation Movers ", "Up"); // Display that they are up
            }

            if (foundationMoverPos) {                                  // If the foundation movers are down
                telemetry.addData("Foundation Movers ", "Down"); // Display that they are down
            }

            // Speed telemetry
            telemetry.addData("Speed", speed);                      // Adds telemetry to the screen to show the current speed of the robot

            telemetry.addData("Quarter speed brake: ", QSB);        // Adds telemetry to the screen to show if the quarter speed brake is on
            telemetry.addData("Half speed brake: ", HSB);           // Adds telemetry to the screen to show if the half speed brake is on

            telemetry.update();                                        // Tells the telemetry to display on the phone

            /** End of telemetry **/
        }
    }

    public void encoderLift(double liftSpeed, double levels) {  // Creates a void that the code can run at any time, and creates two doubles: "liftSpeed" and "levels"
            int newLiftTarget;                                      // Creates the integer "newLiftTarget"

            if (opModeIsActive()) {     // Do the following after the start button has been pressed and until the stop button is pressed
                newLiftTarget = (lift.getCurrentPosition() + (int) (levels * COUNTS_PER_LEVEL)) + 50;

                if (needFoundation) {
                    newLiftTarget = (lift.getCurrentPosition() + (int) (levels * COUNTS_PER_LEVEL)) + 150;
                    needFoundation = false;
                }

                lift.setTargetPosition(newLiftTarget);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(liftSpeed);

                while (opModeIsActive() && lift.isBusy()) {
                    telemetry.addData("lift position", lift.getCurrentPosition());
                    telemetry.update();

                    if (gamepad2.back) { //To jump out of void in case it gets stuck at the bottom
                        height = 0; //Sets height to level 0 so that the lift can continue normal operation after jumping out of loop
                        return; //Jumps out of Private Void
                    }
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