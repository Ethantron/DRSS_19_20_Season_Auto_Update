package org.firstinspires.ftc.teamcode.Competition_Code.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@Autonomous(name = "Galileo_Master", group = "Autonomous")
public class Galileo_Master extends LinearOpMode {

    Galileo_Hardware robot = new Galileo_Hardware();   //Calls Upon Robot Definitions File

    private ElapsedTime runtime = new ElapsedTime(); //Sets timer for encoders

    double step = 1; //Sets the steps for the autonomous


    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "ASHBoLr/////AAABmbIUXlLiiEgrjVbqu8Iavlg6iPFigYso/+BCZ9uMzyAZFoo9CIzpV818SAqrjzuygz3hCeLW/ImK3xMH7DalGMwavqetwXS9Jw4I+rff2naxgV7n+EtYFvdCkUJDHfHVq1A4mhxDHgrjWZEqnLmZk25ppnIizQ0Ozcq4h6UmrWndEVEz8eKcCgn+IuglCEoEswvNBRAaKm/TAlpxLRNC6jQkZdJUh/TGYT05g9YCZo4+1ugmx01jrPCyHQVPVoeXm6VebLIuP7sNPw7njYzmVi2ffV5bYc4vf5kc5l5JwhBdPqnxuMfDLnHWaCkAO1UlVWqy2eY7/4b6iUYI2yN16ZKswSzLMmMNtPBu7e9HhKxA";

    // Since ImageTarget trackables use mm to specify their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    @Override public void runOpMode() {
        robot.init(hardwareMap); //Calls Upon Robot Initialization File
        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.
        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        composeTelemetry(); //Gyro Telemetry Initialization

        telemetry.addData("Drive Train: ", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
        telemetry.addData("Payload: ", "Initialized");          // Adds telemetry to the screen to show that the payload is initialized
        telemetry.addData("Status: ", "Ready");                 // Adds telemetry to the screen to show that the robot is ready
        telemetry.addData("Press Play to Start ", "Autonomous");    // Adds telemetry to the screen to tell the drivers that the code is ready to start
        telemetry.update();                                                   // Tells the telemetry to display on the phone
        waitForStart();

            targetsSkyStone.activate();
            while (!isStopRequested()) {

                // check all the trackable targets to see which one (if any) is visible.
                targetVisible = false;
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        targetVisible = true;
                            if (trackable.getName() == "Red Perimeter 1"){
                                encoderTurn(1, 360, 10);
                            }
                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }
                }

                // Provide feedback as to where the robot is located (if we know).
                if (targetVisible) {
                    // express position (translation) of robot in inches.
                    VectorF translation = lastLocation.getTranslation();
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                    // express the rotation of the robot in degrees.
                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                }
                else {
                    telemetry.addData("Visible Target", "none");
                }
                telemetry.update();
            }

            // Disable Tracking when we are done;
            targetsSkyStone.deactivate();
    }


    public void RedBuild(){

        if (step == 1) {                                    //Move forward
            stepTelemetry();                                //Display Telemetry

            encoderDrive(.4, 46, 10); //Move forward 46 inches just before the foundation

            step++;
        }

        if (step == 2) {                                     //Turn clockwise 90 degrees
            stepTelemetry();                                 //Display Telemetry

            encoderTurn(.35, -90, 10); //Turn CW 90 Degrees
            gyroTurn(.1, -90);                   //Make Sure We are perfectly 90 degrees
            gyroHold(.1,-90,.25);       //Hold Angle for .25 seconds

            step++;
        }

        if (step == 3) {                                    //Lift to clear foundation
            stepTelemetry();                                //Display Telemetry

            encoderLift(1, .75);           //Move the lift up 2.5 inches to clear the foundation

            step++;
        }

        if (step == 4) {                                    //Start moving against the foundation
            stepTelemetry();                                //Display Telemetry

            encoderDrive(.3,12, 10);  //Move forward 12 inches
            sleep(1000);                         //Wait 1 second

            step++;
        }

        if (step == 5) {                                    //Clamp down on the foundation
            stepTelemetry();                                //Display Telemetry

            robot.foundationMoverL.setPosition(1);          //Set Foundation movers to clamp down on the foundation
            robot.foundationMoverR.setPosition(1);          //Set Foundation movers to clamp down on the foundation

            sleep(1000);                        //Wait 1 second

            step++;
        }

        if (step == 6) {                                      //Turn Clockwise 60 degrees
            stepTelemetry();                                  //Display Telemetry

            encoderTurn(.35, -75, 10); //Turn CW 60 degrees

            step++;
        }

        if (step == 7) {                                    //Move the foundation forward 25 inches
            stepTelemetry();                                //Display Telemetry

            encoderDrive(.5, 14, 10); //Move forward 14 inches to place foundation into zone

            step++;
        }

        if (step == 8) {                                    //Release the foundation
            stepTelemetry();                                //Display Telemetry

            robot.foundationMoverL.setPosition(0);          //Set Foundation movers to release the foundation
            robot.foundationMoverR.setPosition(0);          //Set Foundation movers to release the foundation

            step++;
        }

        if (step == 9) {                                   //Move backwards
            stepTelemetry();                                //Display Telemetry

            encoderDrive(.5,-22,10);  //Move backwards 18 inches

            step++;
        }

        if (step == 10) {                                   //Turn to 90 degrees from starting position
            stepTelemetry();                                //Display Telemetry

            encoderTurn(.35, 75, 10);

            step++;
        }

        if (step == 11) { //Strafe right
            stepTelemetry();

            robot.motorFrontRight.setPower(-.7);
            robot.motorFrontLeft.setPower(.7);
            robot.motorBackLeft.setPower(-.7);
            robot.motorBackRight.setPower(.7);
            sleep(1500);
            robot.motorFrontRight.setPower(0);
            robot.motorFrontLeft.setPower(0);
            robot.motorBackLeft.setPower(0);
            robot.motorBackRight.setPower(0);

            step++;
        }

        if (step == 12) {                                   //Park on line
            stepTelemetry();                                //Display Telemetry

            encoderDrive(.6, -20, 10); //Move backwards 20 inches

            step++;
        }

        if (step == 13) {                                   //Make sure motors are stopped, and end autonomous
            stepTelemetry();                                //Display Telemetry

            robot.motorFrontRight.setPower(0);              //Set motor power to stop
            robot.motorFrontLeft.setPower(0);               //Set motor power to stop
            robot.motorBackLeft.setPower(0);                //Set motor power to stop
            robot.motorBackRight.setPower(0);               //Set motor power to stop
            /** End of autonomous **/
        }
    }

    public void RedLoad(){
        while (opModeIsActive()) {

            if (step == 1) { //Move forward and scan the first block
                stepTelemetry(); //Display telemetry

                //Open the grabber
                robot.grabStone.setPosition(.6); //Set the grabber to open position

                //Move Forward
                encoderDrive(0.2, 12, 10);  // Forward 12 Inches with 10 Sec timeout

                //Move Slide Forward
                robot.slide.setPower(1); //Move Slide Forward
                sleep(200); //Wait 200 Milliseconds
                robot.slide.setPower(0); //Stop Moving Slide Forward

                if (robot.tfod != null) {
                    robot.tfod.activate();
                }

                sleep(1000);

                //Start Scanning
                robot.pos++; //Tells code that it is checking position 1
                scan(); //Scan for skystone

                step++; //Next Step
            }

            if (step == 2 && !robot.Skystone) { //If the first block wasn't the skystone, move to the second block and scan it
                stepTelemetry(); //Display Telemetry

                //Setting skystone position for later
                robot.pos++; //If we didn't see the skystone in position 1, move to next position

                //Strafe Left to next block
                robot.motorFrontLeft.setPower(-.6); //Set the motors to strafe left
                robot.motorFrontRight.setPower(.6); //Set the motors to strafe left
                robot.motorBackLeft.setPower(.6); //Set the motors to strafe left
                robot.motorBackRight.setPower(-.6); //Set the motors to strafe left
                sleep(500); //Wait 500 milliseconds
                robot.motorFrontLeft.setPower(0); //Stop all power to the motors
                robot.motorFrontRight.setPower(0); //Stop all power to the motors
                robot.motorBackLeft.setPower(0); //Stop all power to the motors
                robot.motorBackRight.setPower(0); //Stop all power to the motors

                //Scan for second skystone
                sleep(robot.scanTime); //Wait 2 seconds to give vuforia time to identify the skystone
                scan(); //Scan for Skystone

                step++; //Next Step
            }

            if (step == 3 && !robot.Skystone) { //If the first two blocks weren't the skystone, it must be the third. Move and grab it
                stepTelemetry(); //Display Telemetry

                //Setting skystone position for later
                robot.pos++; //If we didn't see the skystone, move to next position

                //Strafe Left to next block
                robot.motorFrontLeft.setPower(-.6); //Set the motors to strafe left
                robot.motorFrontRight.setPower(.6); //Set the motors to strafe left
                robot.motorBackLeft.setPower(.6); //Set the motors to strafe left
                robot.motorBackRight.setPower(-.6); //Set the motors to strafe left
                sleep(600); //Wait for 600 milliseconds
                robot.motorFrontLeft.setPower(0); //Stop all power to the motors
                robot.motorFrontRight.setPower(0); //Stop all power to the motors
                robot.motorBackLeft.setPower(0); //Stop all power to the motors
                robot.motorBackRight.setPower(0); //Stop all power to the motors

                //Set skystone as true, which also moves us on to the next step
                robot.Skystone = true; //If position 1 and 2 are not skystone, then it must be position 3

            }

            if (step > 1 && step < 4 && robot.Skystone) { //If skystone is true
                step = 4; //If the skystone is found, move on to grabbing
            }

            if (step == 4) { //Grabbing the first skystone
                stepTelemetry(); //Display Telemetry

                //Move forward to grab skystone
                encoderDrive(.2, 18, 10); //Moves forward 18 inches to the block

                //Grab skystone
                robot.grabStone.setPosition(0.0); //Grab the Skystone
                sleep(300); //Wait 300 milliseconds

                //Move the lift up
                encoderLift(1, 1.25); //Lift up the lift 1.25"
                sleep(300); //Wait 300 milliseconds

                step++; //Move to the next step
            }

            if (step == 5) { //Move backwards with the skystone
                stepTelemetry(); //Display Telemetry

                //Move backwards
                encoderDrive(.6, -18, 10); //Move backwards 18 inches

                step++; //Move to the next step
            }

            if (step == 6) { //Turn 90 degrees clockwise
                stepTelemetry(); //Display Telemetry

                //Turn Clockwise
                encoderTurn(.25, -90, 10); //Turn CW 90 Degrees

                step++; //Move to the next step
            }

            if (step == 7) { //Run across the line
                stepTelemetry(); //Display Telemetry

                //Set distances needed to be moved by each position
                if (robot.pos == 1) { //If the skystone was in position 1
                    encoderDrive(1, 35, 10); //Run forward 35 inches at speed of 1
                    step++; //Move to the next step
                }

                if (robot.pos == 2) { //If the skystone was in position 2
                    encoderDrive(1, 39, 10); //Run forward 39 inches at speed of 1
                    step++; //Move to the next step
                }

                if (robot.pos == 3) { //If the skystone was in position 3
                    encoderDrive(1, 51, 10); //Run forward 51 inches at speed of 1
                    step++; //Move to the next step
                }
            }

            if (step == 8) { //Drop off the first skystone
                stepTelemetry(); //Display telemetry

                //Place the skystone
                robot.grabStone.setPosition(0.6); //Release the skystone

                step++; //Move to next step
            }

            if (step == 9) { //Run back to the second skystone
                stepTelemetry(); //Display telemetry
                if (robot.pos == 1) { //If the skystone was in position 1
                    encoderDrive(.6,-62,10); //Move backwards 62 inches to second skystone
                    step++;
                }
                if (robot.pos == 2) { //If the skystone was in position 2
                    encoderDrive(.6,-64,10); //Move backwards 58 inches to second skystone
                    step++;
                }
                if (robot.pos == 3) { //If the skystone was in position 3
                    //encoderDrive(.6,-75,10); //Use in case we want second skystone on pos 3
                    step++; //Move to next step
                }
            }

            if (step == 10 && (robot.pos == 1 || robot.pos == 2)) { //Turn toward the second skystone
                stepTelemetry(); //Display telemetry

                //Drive the lift up
                encoderLift(1, -1); //Drop the lift 1"

                //Turn 90 degrees counterclockwise
                encoderTurn(.25, 90, 10); //Turn CCW 90 Degrees
                gyroTurn(0.1, 0); //Use gyro to make sure we are at the right angle
                gyroHold(0.1, 0, 0.5); //Hold the angle for .5 seconds

                step++; //Move to next step
            }

            if (step == 10 && robot.pos == 3) { //Move backwards to park
                stepTelemetry(); //Display telemetry

                //Move backwards
                encoderDrive(1, -16, 10); //Move Backwards 16 inches

                //Strafe Left to get out of the way
                robot.motorFrontLeft.setPower(-.4); //Set power to strafe left
                robot.motorFrontRight.setPower(.4); //Set power to strafe left
                robot.motorBackLeft.setPower(.4); //Set power to strafe left
                robot.motorBackRight.setPower(-.4); //Set power to strafe left
                sleep(500); //Wait 500 milliseconds

                step++; //Move to next step
            }

            if (step == 11 && robot.pos==3) { //Stop strafing and end position 3
                //Stop Strafing
                robot.motorFrontLeft.setPower(0); //Stop all power to the motors
                robot.motorFrontRight.setPower(0); //Stop all power to the motors
                robot.motorBackLeft.setPower(0); //Stop all power to the motors
                robot.motorBackRight.setPower(0); //Stop all power to the motors
                //End of position 3
            }

            if (step == 11 && (robot.pos == 1 || robot.pos == 2)) { //Grab the second skystone
                stepTelemetry(); //Display telemetry

                //Drive forward
                encoderDrive(.2, 22, 10); //Moves forward 22 inches to the block

                //Grab the skystone
                robot.grabStone.setPosition(0.0); //Grab the Skystone
                sleep(300); //Wait 300 milliseconds

                //Drive the lift up
                encoderLift(1, 1.25); //Lift up the lift 1.25"
                sleep(300); //Wait 300 milliseconds

                step++; //Move to next step
            }

            if (step == 12) { //Move backwards with skystone
                stepTelemetry(); //Display telemetry

                //Move backward
                encoderDrive(.6, -22, 10); //Move backwards 22 inches

                step++; //Move to next step
            }

            if (step == 13) { //Turn 90 degrees
                stepTelemetry(); //Display telemetry
                encoderTurn(.25, -90, 10); //Turn CW 90 Degrees
                step++; //Move to next step
            }

            if (step == 14) { //Start moving back across the line
                if (robot.pos == 1) { //If the skystone was in position 1
                    encoderDrive(1, 62, 10); //Move forward across the line
                    step++; // Move to next step
                }

                else if (robot.pos == 2) { //If the skystone was in position 2
                    encoderDrive(1, 65, 10); //Move forward across the line
                    step++; //Move to next step
                }
            }

            if (step == 15) { //Release the skystone
                stepTelemetry(); //Display Telemetry

                //Release the skystone
                robot.grabStone.setPosition(0.6); //release the grabber

                step++; //Move to next step
            }

            if (step == 16) { //Move backward to the line
                stepTelemetry(); //Display Telemetry

                //Move backward
                encoderDrive(1,-16,10); //Move backward 16 inches

                step++; //Move to next step
            }

            if (step == 17) { //Strafe left

                //Strafe Left to get out of the way
                robot.motorFrontLeft.setPower(-.4); //Set power to strafe left
                robot.motorFrontRight.setPower(.4); //Set power to strafe left
                robot.motorBackLeft.setPower(.4); //Set power to strafe left
                robot.motorBackRight.setPower(-.4); //Set power to strafe left
                sleep(750); //Wait 750 milliseconds

                step++; //move to next step
            }

            if (step == 18) { //Stop motors and end of position 1 and 2
                //Stop all motors
                robot.motorFrontLeft.setPower(0); //Stop all power to the motors
                robot.motorFrontRight.setPower(0); //Stop all power to the motors
                robot.motorBackLeft.setPower(0); //Stop all power to the motors
                robot.motorBackRight.setPower(0); //Stop all power to the motors
                //End of position 1 & 2
            }
        }

    }

    public void BlueBuild(){
        if (step == 1) {                                    //Move forward
            stepTelemetry();                                //Display Telemetry

            encoderDrive(.4, 46, 10); //Move forward 46 inches just before the foundation

            step++;
        }

        if (step == 2) {                                     //Turn counterclockwise 90 degrees
            stepTelemetry();                                 //Display Telemetry

            encoderTurn(.35, 90, 10); //Turn CCW 90 Degrees
            gyroTurn(.1, 90);                   //Make Sure We are perfectly 90 degrees
            gyroHold(.1,90,.25);       //Hold Angle for .25 seconds

            step++;
        }

        if (step == 3) {                                    //Lift to clear foundation
            stepTelemetry();                                //Display Telemetry

            encoderLift(1, 1);           //Move the lift up 2.5 inches to clear the foundation

            step++;
        }

        if (step == 4) {                                    //Start moving against the foundation
            stepTelemetry();                                //Display Telemetry

            encoderDrive(.3,12, 10);  //Move forward 12 inches
            sleep(1000);                         //Wait 1 second

            step++;
        }

        if (step == 5) {                                    //Clamp down on the foundation
            stepTelemetry();                                //Display Telemetry

            robot.foundationMoverL.setPosition(1);          //Set Foundation movers to clamp down on the foundation
            robot.foundationMoverR.setPosition(1);          //Set Foundation movers to clamp down on the foundation

            sleep(1000);                        //Wait 1 second

            step++;
        }

        if (step == 6) {                                      //Turn CounterClockwise 60 degrees
            stepTelemetry();                                  //Display Telemetry

            encoderTurn(.35, 75, 10); //Turn CCW 60 degrees

            step++;
        }

        if (step == 7) {                                    //Move the foundation forward 25 inches
            stepTelemetry();                                //Display Telemetry

            encoderDrive(.5, 14, 10); //Move forward 14 inches to place foundation into zone

            step++;
        }

        if (step == 8) {                                    //Release the foundation
            stepTelemetry();                                //Display Telemetry

            robot.foundationMoverL.setPosition(0);          //Set Foundation movers to release the foundation
            robot.foundationMoverR.setPosition(0);          //Set Foundation movers to release the foundation

            step++;
        }

        if (step == 9) {                                   //Move backwards
            stepTelemetry();                                //Display Telemetry

            encoderDrive(.5,-22,10);  //Move backwards 18 inches

            step++;
        }

        if (step == 10) {                                   //Turn to 90 degrees from starting position
            stepTelemetry();                                //Display Telemetry

            encoderTurn(.35, -75, 10);

            step++;
        }

        if (step == 11) { //Strafe right
            stepTelemetry();

            robot.motorFrontRight.setPower(.7);
            robot.motorFrontLeft.setPower(-.7);
            robot.motorBackLeft.setPower(.7);
            robot.motorBackRight.setPower(-.7);
            sleep(1500);
            robot.motorFrontRight.setPower(0);
            robot.motorFrontLeft.setPower(0);
            robot.motorBackLeft.setPower(0);
            robot.motorBackRight.setPower(0);

            step++;
        }

        if (step == 12) {                                   //Park on line
            stepTelemetry();                                //Display Telemetry

            encoderDrive(1, -28, 10); //Move backwards 28 inches

            step++;
        }

        if (step == 13) {                                   //Make sure motors are stopped, and end autonomous
            stepTelemetry();                                //Display Telemetry

            robot.motorFrontRight.setPower(0);              //Set motor power to stop
            robot.motorFrontLeft.setPower(0);               //Set motor power to stop
            robot.motorBackLeft.setPower(0);                //Set motor power to stop
            robot.motorBackRight.setPower(0);               //Set motor power to stop
            /** End of autonomous **/
        }
    }

    public void BlueLoad(){
        while (opModeIsActive()) {

            if (step == 1) { //Move forward and scan the first block
                stepTelemetry(); //Display telemetry

                //Open the grabber
                robot.grabStone.setPosition(.6); //Set the grabber to open position

                //Move Forward
                encoderDrive(0.2, 12, 10);  // Forward 12 Inches with 10 Sec timeout

                //Move Slide Forward
                robot.slide.setPower(1); //Move Slide Forward
                sleep(200); //Wait 200 Milliseconds
                robot.slide.setPower(0); //Stop Moving Slide Forward

                if (robot.tfod != null) {
                    robot.tfod.activate();
                }

                sleep(2000);

                //Start Scanning
                robot.pos++; //Tells code that it is checking position 1
                scan(); //Scan for skystone

                step++; //Next Step
            }

            if (step == 2 && !robot.Skystone) { //If the first block wasn't the skystone, move to the second block and scan it
                stepTelemetry(); //Display Telemetry

                //Setting skystone position for later
                robot.pos++; //If we didn't see the skystone in position 1, move to next position

                //Strafe Right to next block
                robot.motorFrontLeft.setPower(.6); //Set the motors to strafe right
                robot.motorFrontRight.setPower(-.6); //Set the motors to strafe right
                robot.motorBackLeft.setPower(-.6); //Set the motors to strafe right
                robot.motorBackRight.setPower(.6); //Set the motors to strafe right
                sleep(500); //Wait 500 milliseconds
                robot.motorFrontLeft.setPower(0); //Stop all power to the motors
                robot.motorFrontRight.setPower(0); //Stop all power to the motors
                robot.motorBackLeft.setPower(0); //Stop all power to the motors
                robot.motorBackRight.setPower(0); //Stop all power to the motors

                //Scan for second skystone
                sleep(robot.scanTime); //Wait 2 seconds to give vuforia time to identify the skystone
                scan(); //Scan for Skystone

                step++; //Next Step
            }

            if (step == 3 && !robot.Skystone) { //If the first two blocks weren't the skystone, it must be the third. Move and grab it
                stepTelemetry(); //Display Telemetry

                //Setting skystone position for later
                robot.pos++; //If we didn't see the skystone, move to next position

                //Strafe Right to next block
                robot.motorFrontLeft.setPower(.6); //Set the motors to strafe right
                robot.motorFrontRight.setPower(-.6); //Set the motors to strafe right
                robot.motorBackLeft.setPower(-.6); //Set the motors to strafe right
                robot.motorBackRight.setPower(.6); //Set the motors to strafe right
                sleep(600); //Wait for 600 milliseconds
                robot.motorFrontLeft.setPower(0); //Stop all power to the motors
                robot.motorFrontRight.setPower(0); //Stop all power to the motors
                robot.motorBackLeft.setPower(0); //Stop all power to the motors
                robot.motorBackRight.setPower(0); //Stop all power to the motors

                //Set skystone as true, which also moves us on to the next step
                robot.Skystone = true; //If position 1 and 2 are not skystone, then it must be position 3

            }

            if (step > 1 && step < 4 && robot.Skystone) { //If skystone is true
                step = 4; //If the skystone is found, move on to grabbing
            }

            if (step == 4) { //Grabbing the first skystone
                stepTelemetry(); //Display Telemetry

                //Move forward to grab skystone
                encoderDrive(.2, 18, 10); //Moves forward 18 inches to the block

                //Grab skystone
                robot.grabStone.setPosition(0.0); //Grab the Skystone
                sleep(300); //Wait 300 milliseconds

                //Move the lift up
                encoderLift(1, 1.25); //Lift up the lift 1.25"
                sleep(300); //Wait 300 milliseconds

                step++; //Move to the next step
            }

            if (step == 5) { //Move backwards with the skystone
                stepTelemetry(); //Display Telemetry

                //Move backwards
                encoderDrive(.6, -18, 10); //Move backwards 18 inches

                step++; //Move to the next step
            }

            if (step == 6) { //Turn 90 degrees clockwise
                stepTelemetry(); //Display Telemetry

                //Turn Clockwise
                encoderTurn(.25, 90, 10); //Turn CCW 90 Degrees

                step++; //Move to the next step
            }

            if (step == 7) { //Run across the line
                stepTelemetry(); //Display Telemetry

                //Set distances needed to be moved by each position
                if (robot.pos == 1) { //If the skystone was in position 1
                    encoderDrive(1, 35, 10); //Run forward 35 inches at speed of 1
                    step++; //Move to the next step
                }

                if (robot.pos == 2) { //If the skystone was in position 2
                    encoderDrive(1, 39, 10); //Run forward 39 inches at speed of 1
                    step++; //Move to the next step
                }

                if (robot.pos == 3) { //If the skystone was in position 3
                    encoderDrive(1, 51, 10); //Run forward 51 inches at speed of 1
                    step++; //Move to the next step
                }
            }

            if (step == 8) { //Drop off the first skystone
                stepTelemetry(); //Display telemetry

                //Place the skystone
                robot.grabStone.setPosition(0.6); //Release the skystone

                step++; //Move to next step
            }

            if (step == 9) { //Run back to the second skystone
                stepTelemetry(); //Display telemetry
                if (robot.pos == 1) { //If the skystone was in position 1
                    encoderDrive(.6,-62,10); //Move backwards 62 inches to second skystone
                    step++;
                }
                if (robot.pos == 2) { //If the skystone was in position 2
                    encoderDrive(.6,-64,10); //Move backwards 58 inches to second skystone
                    step++;
                }
                if (robot.pos == 3) { //If the skystone was in position 3
                    //encoderDrive(.6,-75,10); //Use in case we want second skystone on pos 3
                    step++; //Move to next step
                }
            }

            if (step == 10 && (robot.pos == 1 || robot.pos == 2)) { //Turn toward the second skystone
                stepTelemetry(); //Display telemetry

                //Drive the lift up
                encoderLift(1, -1); //Drop the lift 1"

                //Turn 90 degrees counterclockwise
                encoderTurn(.25, -90, 10); //Turn CW 90 Degrees
                gyroTurn(0.1, 0); //Use gyro to make sure we are at the right angle
                gyroHold(0.1, 0, 0.5); //Hold the angle for .5 seconds

                step++; //Move to next step
            }

            if (step == 10 && robot.pos == 3) { //Move backwards to park
                stepTelemetry(); //Display telemetry

                //Move backwards
                encoderDrive(1, -16, 10); //Move Backwards 16 inches

                //Strafe Right to get out of the way
                robot.motorFrontLeft.setPower(.4); //Set power to strafe right
                robot.motorFrontRight.setPower(-.4); //Set power to strafe right
                robot.motorBackLeft.setPower(-.4); //Set power to strafe right
                robot.motorBackRight.setPower(.4); //Set power to strafe right
                sleep(500); //Wait 500 milliseconds

                step++; //Move to next step
            }

            if (step == 11 && robot.pos==3) { //Stop strafing and end position 3
                //Stop Strafing
                robot.motorFrontLeft.setPower(0); //Stop all power to the motors
                robot.motorFrontRight.setPower(0); //Stop all power to the motors
                robot.motorBackLeft.setPower(0); //Stop all power to the motors
                robot.motorBackRight.setPower(0); //Stop all power to the motors
                //End of position 3
            }

            if (step == 11 && (robot.pos == 1 || robot.pos == 2)) { //Grab the second skystone
                stepTelemetry(); //Display telemetry

                //Drive forward
                encoderDrive(.2, 22, 10); //Moves forward 22 inches to the block

                //Grab the skystone
                robot.grabStone.setPosition(0.0); //Grab the Skystone
                sleep(300); //Wait 300 milliseconds

                //Drive the lift up
                encoderLift(1, 1.25); //Lift up the lift 1.25"
                sleep(300); //Wait 300 milliseconds

                step++; //Move to next step
            }

            if (step == 12) { //Move backwards with skystone
                stepTelemetry(); //Display telemetry

                //Move backward
                encoderDrive(.6, -22, 10); //Move backwards 22 inches

                step++; //Move to next step
            }

            if (step == 13) { //Turn 90 degrees
                stepTelemetry(); //Display telemetry
                encoderTurn(.25, 90, 10); //Turn CCW 90 Degrees
                step++; //Move to next step
            }

            if (step == 14) { //Start moving back across the line
                if (robot.pos == 1) { //If the skystone was in position 1
                    encoderDrive(1, 62, 10); //Move forward across the line
                    step++; // Move to next step
                }

                else if (robot.pos == 2) { //If the skystone was in position 2
                    encoderDrive(1, 65, 10); //Move forward across the line
                    step++; //Move to next step
                }
            }

            if (step == 15) { //Release the skystone
                stepTelemetry(); //Display Telemetry

                //Release the skystone
                robot.grabStone.setPosition(0.6); //release the grabber

                step++; //Move to next step
            }

            if (step == 16) { //Move backward to the line
                stepTelemetry(); //Display Telemetry

                //Move backward
                encoderDrive(1,-16,10); //Move backward 16 inches

                step++; //Move to next step
            }

            if (step == 17) { //Strafe left

                //Strafe Right to get out of the way
                robot.motorFrontLeft.setPower(.4); //Set power to strafe right
                robot.motorFrontRight.setPower(-.4); //Set power to strafe right
                robot.motorBackLeft.setPower(-.4); //Set power to strafe right
                robot.motorBackRight.setPower(.4); //Set power to strafe right
                sleep(750); //Wait 750 milliseconds

                step++; //move to next step
            }

            if (step == 18) { //Stop motors and end of position 1 and 2
                //Stop all motors
                robot.motorFrontLeft.setPower(0); //Stop all power to the motors
                robot.motorFrontRight.setPower(0); //Stop all power to the motors
                robot.motorBackLeft.setPower(0); //Stop all power to the motors
                robot.motorBackRight.setPower(0); //Stop all power to the motors
                //End of position 1 & 2
            }
        }

    }

    public void scan() {
        if (robot.tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    if (recognition.getLabel().equals(Galileo_Hardware.LABEL_SECOND_ELEMENT)) {
                        robot.Skystone = true;
                    }
                }
                telemetry.update();
            }
        }
    }

    //Telemetry
    private void stepTelemetry() {
        telemetry.addData("Current step: ", step);
        telemetry.addData("Skystone Position: ", robot.pos);
        telemetry.update();
    }

    //Encoder Voids
    public void encoderDrive(double speed, double Inches, double timeoutS) {

        //Create our target variables
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Math to calculate each target position for the motors
            newFrontLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (Inches * Galileo_Hardware.COUNTS_PER_INCH);
            newFrontRightTarget = robot.motorFrontRight.getCurrentPosition() + (int) (Inches * Galileo_Hardware.COUNTS_PER_INCH);
            newBackLeftTarget = robot.motorBackLeft.getCurrentPosition() + (int) (Inches * Galileo_Hardware.COUNTS_PER_INCH);
            newBackRightTarget = robot.motorBackRight.getCurrentPosition() + (int) (Inches * Galileo_Hardware.COUNTS_PER_INCH);

            //Set Target Positions to respective motors
            robot.motorFrontLeft.setTargetPosition(newFrontLeftTarget);
            robot.motorFrontRight.setTargetPosition(newFrontRightTarget);
            robot.motorBackLeft.setTargetPosition(newBackLeftTarget);
            robot.motorBackRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorFrontLeft.setPower(speed);
            robot.motorFrontRight.setPower(speed);
            robot.motorBackLeft.setPower(speed);
            robot.motorBackRight.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy() && robot.motorBackLeft.isBusy() && robot.motorBackRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Motor Paths", "Running at %7d : %7d : %7d : %7d", //Tells us where we are
                        robot.motorFrontLeft.getCurrentPosition(), //Front Left Position
                        robot.motorFrontRight.getCurrentPosition(), //Front Right Position
                        robot.motorBackLeft.getCurrentPosition(), //Back Left Position
                        robot.motorBackRight.getCurrentPosition()); //Back Right Position
                telemetry.update();
            }

            // Stop all motion;
            robot.motorFrontLeft.setPower(0);
            robot.motorFrontRight.setPower(0);
            robot.motorBackLeft.setPower(0);
            robot.motorBackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderLift(double liftSpeed, double Inches) {  // Creates a void that the code can run at any time, and creates two doubles: "liftSpeed" and "levels"
        int newLiftTarget;                                      // Creates the integer "newLiftTarget"

        if (opModeIsActive()) {     // Do the following after the start button has been pressed and until the stop button is pressed
            newLiftTarget = (robot.lift.getCurrentPosition() + (int) (Inches * Galileo_Hardware.COUNTS_PER_LIFT_INCH));

            robot.lift.setTargetPosition(newLiftTarget);

            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.lift.setPower(liftSpeed);

            while (opModeIsActive() && robot.lift.isBusy()) {
                telemetry.addData("lift position", robot.lift.getCurrentPosition());
                telemetry.update();
            }

            robot.lift.setPower(0);
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderTurn(double speed, double angle, double timeoutS) {
        //Create our target variables
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        //Create our new circumference variables
        double c = 60.35; //Circumference of arc created by robot wheels (Radius is 9.605")
        double ANGLE_RATIO = angle / 360; //Ratio of angle relative to entire circle
        double CIRCUMFERENCE_OF_ANGLE = c * ANGLE_RATIO; //Circumference of Angle
        int COUNTS_PER_DISTANCE = (int) ((CIRCUMFERENCE_OF_ANGLE * Galileo_Hardware.COUNTS_PER_INCH) * 1.305);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Math to calculate each target position for the motors
            newFrontLeftTarget = robot.motorFrontLeft.getCurrentPosition() - COUNTS_PER_DISTANCE;
            newFrontRightTarget = robot.motorFrontRight.getCurrentPosition() + COUNTS_PER_DISTANCE;
            newBackLeftTarget = robot.motorBackLeft.getCurrentPosition() - COUNTS_PER_DISTANCE;
            newBackRightTarget = robot.motorBackRight.getCurrentPosition() + COUNTS_PER_DISTANCE;

            //Set Target Positions to respective motors
            robot.motorFrontLeft.setTargetPosition(newFrontLeftTarget);
            robot.motorFrontRight.setTargetPosition(newFrontRightTarget);
            robot.motorBackLeft.setTargetPosition(newBackLeftTarget);
            robot.motorBackRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorFrontLeft.setPower(speed);
            robot.motorFrontRight.setPower(speed);
            robot.motorBackLeft.setPower(speed);
            robot.motorBackRight.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy() && robot.motorBackLeft.isBusy() && robot.motorBackRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("FLM: Path2", "Running at %7d", //Tells us where we are
                        robot.motorFrontLeft.getCurrentPosition()); //Front Left Position
                telemetry.addData("FRM: Path2", "Running at %7d", //Tells us where we are
                        robot.motorFrontRight.getCurrentPosition()); //Front Right Position
                telemetry.addData("BLM: Path2", "Running at %7d", //Tells us where we are
                        robot.motorBackLeft.getCurrentPosition()); //Back Left Position
                telemetry.addData("BRM: Path2", "Running at %7d", //Tells us where we are
                        robot.motorBackRight.getCurrentPosition()); //Back Right Position
                telemetry.update();
            }

            // Stop all motion;
            robot.motorFrontLeft.setPower(0);
            robot.motorFrontRight.setPower(0);
            robot.motorBackLeft.setPower(0);
            robot.motorBackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    //Gyro Turning and initialization
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, Galileo_Hardware.P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, Galileo_Hardware.P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.motorFrontLeft.setPower(0);
        robot.motorBackLeft.setPower(0);
        robot.motorFrontRight.setPower(0);
        robot.motorBackRight.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= Galileo_Hardware.HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.motorFrontLeft.setPower(leftSpeed);
        robot.motorBackLeft.setPower(leftSpeed);
        robot.motorFrontRight.setPower(rightSpeed);
        robot.motorBackRight.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.angles.firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robot.gravity = robot.imu.getGravity();
            }
        });
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(robot.angles.angleUnit, robot.angles.firstAngle);
                    }
                });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}