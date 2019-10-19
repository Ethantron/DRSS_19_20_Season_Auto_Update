package org.firstinspires.ftc.teamcode.Test.Auto_Tests;

import android.graphics.Color;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "Stone_With_Sensors_Test", group= "Autonomous")
public class Stone_With_Sensors_Test extends LinearOpMode {
    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;
    double step = 0;
    double stonecount = 0;

    boolean Skystone = false;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final String VUFORIA_KEY =
            "ASHBoLr/////AAABmbIUXlLiiEgrjVbqu8Iavlg6iPFigYso/+BCZ9uMzyAZFoo9CIzpV818SAqrjzuygz3hCeLW/ImK3xMH7DalGMwavqetwXS9Jw4I+rff2naxgV7n+EtYFvdCkUJDHfHVq1A4mhxDHgrjWZEqnLmZk25ppnIizQ0Ozcq4h6UmrWndEVEz8eKcCgn+IuglCEoEswvNBRAaKm/TAlpxLRNC6jQkZdJUh/TGYT05g9YCZo4+1ugmx01jrPCyHQVPVoeXm6VebLIuP7sNPw7njYzmVi2ffV5bYc4vf5kc5l5JwhBdPqnxuMfDLnHWaCkAO1UlVWqy2eY7/4b6iUYI2yN16ZKswSzLMmMNtPBu7e9HhKxA";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    double centered = 0; //Senses whether or not robot is centered
    double Power = .2; //Sets Motor Power
    double Range = 8; //Change this to change the range of degrees (Tolerance)
    double RangeDiv = Range / 2; //Evenly splits the range
    double WantedAngle = 0; //Wanted Angle
    double RangePlus = WantedAngle + RangeDiv; //adds tolerance to Wanted Angle
    double RangeMinus = WantedAngle - RangeDiv; //subtracts tolerance from Wanted Angle
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    ColorSensor sensorColor;
    ColorSensor color2;
    DistanceSensor sensorDistance;

    @Override
    public void runOpMode() {
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");

        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        //Stops and Resets Encoders
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
/*
        //Tells Robots to Reset Encoders
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER); */

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        composeTelemetry();

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        // Color sensor initialization
        sensorColor = hardwareMap.get(ColorSensor.class, "color_sensor");
        color2 = hardwareMap.get(ColorSensor.class, "color2");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color_sensor");
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;

        /* Wait for the game to begin */
        telemetry.addData("Status: ", "Initialized");
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 100);

        while (opModeIsActive()) {
            telemetry.update();

            if (angles.firstAngle > RangeMinus && angles.firstAngle < RangePlus) { //Stops Robot if centered
                motorFrontRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackRight.setPower(0);
                motorBackLeft.setPower(0);
                centered = 1;
                telemetry.addData("Robot is", "Centered! :)");
            }

            else { //allows robot to adjust if not centered
                centered = 0;
                telemetry.addData("Robot is", "Not Centered! :(");
                telemetry.addData("Adjusting","Robot");
            }

            if (angles.firstAngle < WantedAngle && centered == 0) { //adjust robots by turning right
                motorFrontRight.setPower(-Power);
                motorFrontLeft.setPower(Power);
                motorBackRight.setPower(-Power);
                motorBackLeft.setPower(Power);
            }

            if (angles.firstAngle > WantedAngle && centered == 0) { //adjust robots by turning left
                motorFrontRight.setPower(Power);
                motorFrontLeft.setPower(-Power);
                motorBackRight.setPower(Power);
                motorBackLeft.setPower(-Power);
            }
        }

        if (step == 0) {
            encoderDrive(0.6, 0.6, 0.6, 0.6, 20, 20, 20, 20,  5);
            step++;
        }

        if (step == 1) {
            scan();
            sleep(2000);
            step++;
        }

        if (step == 2){
            sleep(1500);
            if (Skystone){
                rungoodstep();
            }
            else {
                runbadstep();
            }
        }

        while (opModeIsActive()){
            // Color Sensor Code

            // Convert the RGB values to HSV values.
            // Multiply by the SCALE_FACTOR.
            // Cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (color2.red() * SCALE_FACTOR),
                    (int) (color2.green() * SCALE_FACTOR),
                    (int) (color2.blue() * SCALE_FACTOR),
                    hsvValues);
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            // Send the info back to driver station using telemetry function.
            telemetry.addData("Step: ", step);
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();

            // Move forward
            if (step == 3){
                motorFrontRight.setPower(.2);
                motorFrontLeft.setPower(.2);
                motorBackLeft.setPower(.2);
                motorBackRight.setPower(.2);
            }

            // Does it see the line?
            while (step == 3 && opModeIsActive()){
                Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);
                Color.RGBToHSV((int) (color2.red() * SCALE_FACTOR),
                        (int) (color2.green() * SCALE_FACTOR),
                        (int) (color2.blue() * SCALE_FACTOR),
                        hsvValues);

                // Send the info back to driver station using telemetry function.
                telemetry.addData("Step: ", step);
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();

                if (hsvValues[0] > 150 ){ // Checks if it is red or blue
                    step++;
                }
            }

            if (step == 4){
                motorFrontRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                step++;
            }

            if (step == 5){
                motorFrontRight.setPower(-.2);
                motorFrontLeft.setPower(-.2);
                motorBackLeft.setPower(-.2);
                motorBackRight.setPower(-.2);
                sleep(100);
                step++;
            }

            if (step == 6){
                motorFrontRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
            }
        }
    }
    private void initVuforia () {
        // Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /*
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod () {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void scan (){
        if (tfod != null) {
            tfod.activate();
        }
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
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
                    if (recognition.getLabel() == LABEL_SECOND_ELEMENT){
                        Skystone = true;
                    }
                }
                telemetry.update();
            }
        }
        sleep(1500);
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void rungoodstep(){
            encoderDrive(0.6, 0.6, 0.6, 0.6, 5, 5, 5, 5,  .2);
            step++;
    }

    private void runbadstep(){
        if (stonecount < 2) {
                encoderDrive(0.4, 0.7, 0.7, 0.4, 5, -5, -5, 5,  0.1);
                step = 1;
                stonecount++;
        } else if (stonecount > 2) {
            rungoodstep();
        }
    }

    public void encoderDrive(double FLspeed, double FRspeed, double BLspeed, double BRspeed,
                             double FL, double FR, double BL, double BR, double timeoutS) {

        //Create our target variables
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Math to calculate each target position for the motors
            newFrontLeftTarget = motorFrontLeft.getCurrentPosition() + (int)(FL * COUNTS_PER_INCH);
            newFrontRightTarget = motorFrontRight.getCurrentPosition() + (int)(FR * COUNTS_PER_INCH);
            newBackLeftTarget = motorBackLeft.getCurrentPosition() + (int)(BL * COUNTS_PER_INCH);
            newBackRightTarget = motorBackRight.getCurrentPosition() + (int)(BR * COUNTS_PER_INCH);

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
            motorFrontLeft.setPower(FLspeed);
            motorFrontRight.setPower(FRspeed);
            motorBackLeft.setPower(BLspeed);
            motorBackRight.setPower(BRspeed);

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
                telemetry.addData("Current Motor Target",  "Running at %7d : %7d : %7d : %7d", //Tells us where we are
                        motorFrontLeft.getTargetPosition(), //Front Left Position
                        motorFrontRight.getTargetPosition(), //Front Right Position
                        motorBackLeft.getTargetPosition(), //Back Left Position
                        motorBackRight.getTargetPosition()); //Back Right Position

                telemetry.addData("Current Motor Positions",  "Running at %7d : %7d : %7d : %7d", //Tells us where we are
                        motorFrontLeft.getCurrentPosition(), //Front Left Position
                        motorFrontRight.getCurrentPosition(), //Front Right Position
                        motorBackLeft.getCurrentPosition(), //Back Left Position
                        motorBackRight.getCurrentPosition()); //Back Right Position
                telemetry.update();
            }

            // Stop all motion;
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);

            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}