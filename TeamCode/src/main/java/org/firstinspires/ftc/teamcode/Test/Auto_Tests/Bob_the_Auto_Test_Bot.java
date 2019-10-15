package org.firstinspires.ftc.teamcode.Test.Auto_Tests;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;
import java.util.Locale;

@Autonomous(name = "Bob_the_Auto_Test_Bot", group= "Auto_Tests")
public class Bob_the_Auto_Test_Bot extends LinearOpMode {
    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;
    public DcMotor lift;
    public DcMotor slide;
    public Servo grabL;
    public Servo grabR;
    public Servo grabStone;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private DistanceSensor sensorRange;
    DigitalChannel DS1;
    ColorSensor sensorColor;
    ColorSensor color2;
    double step = 0;
    double goodstep = 0;
    double badstep = 0;
    double count = 0;
    double Left = 0;
    double centered = 0; //Senses whether or not robot is centered
    double Power = .2; //Sets Motor Power
    double Range = 8; //Change this to change the range of degrees (Tolerance)
    double RangeDiv = Range / 2; //Evenly splits the range
    double WantedAngle = 0; //Wanted Angle
    double RangePlus = WantedAngle + RangeDiv; //adds tolerance to Wanted Angle
    double RangeMinus = WantedAngle - RangeDiv; //subtracts tolerance from Wanted Angle
    boolean Skystone = false;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    private static final String VUFORIA_KEY =
            "ASHBoLr/////AAABmbIUXlLiiEgrjVbqu8Iavlg6iPFigYso/+BCZ9uMzyAZFoo9CIzpV818SAqrjzuygz3hCeLW/ImK3xMH7DalGMwavqetwXS9Jw4I+rff2naxgV7n+EtYFvdCkUJDHfHVq1A4mhxDHgrjWZEqnLmZk25ppnIizQ0Ozcq4h6UmrWndEVEz8eKcCgn+IuglCEoEswvNBRAaKm/TAlpxLRNC6jQkZdJUh/TGYT05g9YCZo4+1ugmx01jrPCyHQVPVoeXm6VebLIuP7sNPw7njYzmVi2ffV5bYc4vf5kc5l5JwhBdPqnxuMfDLnHWaCkAO1UlVWqy2eY7/4b6iUYI2yN16ZKswSzLMmMNtPBu7e9HhKxA";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");

        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);

        lift = hardwareMap.dcMotor.get("LT");
        slide = hardwareMap.dcMotor.get("SL");

        lift.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);

        grabL = hardwareMap.servo.get("GL");
        grabR = hardwareMap.servo.get("GR");
        grabStone = hardwareMap.servo.get("GS");

        DS1 = hardwareMap.get(DigitalChannel.class, "DS1");
        DS1.setMode(DigitalChannel.Mode.INPUT);

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        sensorColor = hardwareMap.get(ColorSensor.class, "color_sensor");
        color2 = hardwareMap.get(ColorSensor.class, "color2");

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
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

        /* Wait for the game to begin */
        telemetry.addData("Status: ", "Initialized");
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 100);

        while (opModeIsActive()) {
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

        while (goodstep != 1){
            lift.setPower(0.1);
        }

        if (step == 0) {
            AllAhead();
            sleep(175);
            step++;
        }

        if (step == 1) {
            AllStop();
            sleep(100);
            step++;
        }

        if (step == 2) {
            scan();
            sleep(2000);
            step++;
        }

        if (step == 3){
            sleep(1500);
            if (Skystone){
                rungoodstep();
            }
            else {
                badstep = 0;
                runbadstep();
            }
        }

        if (step == 4){
            AllBack();
            sleep(300);
            AllStop();
            WantedAngle = 90;
            sleep(100);
            step++;
        }

        if (step == 5){
            AllAhead();
            sleep(500);
            step++;
        }

        if (step == 6){
            AllStop();
            WantedAngle = 65;
            sleep (100);
            step++;
        }

        if (step == 7){
            if (sensorRange.getDistance(DistanceUnit.INCH) <= 42){
                if (DS1.getState()) {
                    grabL.setPosition(0);
                    grabR.setPosition(90);
                }
                AllAhead();
                sleep(500);
                AllStop();
                WantedAngle = 90;
                step++;
            } else{
                WantedAngle = 115;
                sleep(200);
                Left = 1;
                AllAhead();
                sleep(500);
                AllStop();
                WantedAngle = 90;
                step++;
            }
        }

        if (step == 8){
            grabStone.setPosition(90);
            if (DS1.getState() && Left == 0){
                grabR.setPosition(0);
                sleep(100);
                AllRight();
                sleep(500);
                AllStop();
                grabR.setPosition(90);
            }
            step++;
        }
        
        if (step == 9){
            if (!DS1.getState() && Left == 0){
                WantedAngle = -115;
            } else {
                WantedAngle = -90;
            }
            sleep(300);
            motorFrontRight.setPower(.2);
            motorFrontLeft.setPower(.2);
            motorBackLeft.setPower(.2);
            motorBackRight.setPower(.2);
        }

        while (step == 9){
            if (opModeIsActive()){
                Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);
                Color.RGBToHSV((int) (color2.red() * SCALE_FACTOR),
                        (int) (color2.green() * SCALE_FACTOR),
                        (int) (color2.blue() * SCALE_FACTOR),
                        hsvValues);

                // Send the info back to driver station using telemetry function.
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();
            }
            //
            if (hsvValues[0] > 150 ){ // Checks if it is red or blue
                step++;
            }
        }

        if (step == 10){
            AllStop();
            step++;
        }

        if (step == 11){
            motorFrontRight.setPower(-.5);
            motorFrontLeft.setPower(-.5);
            motorBackLeft.setPower(-.5);
            motorBackRight.setPower(-.5);
            sleep(200);
            step++;
        }

        if (step == 12){
            AllStop();
        }
    }

    private void initVuforia () {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

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

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

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
        if (goodstep == 0) {
            AllAhead();
            sleep(200);
            goodstep++;
        }

        if (goodstep == 1){
            AllStop();
            grabStone.setPosition(0);
            sleep(200);
            goodstep++;
        }

        if (goodstep == 2){
            lift.setPower(.5);
            step++;
            goodstep++;
        }
    }

    private void runbadstep(){
        if (count < 2) {
            if (badstep == 0) {
                AllLeft();
                sleep(100);
                badstep++;
            }
            if (badstep == 1) {
                AllStop();
                step = 2;
                count++;
            }
        } else if (count > 2) {
            rungoodstep();
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
    
    private void AllAhead(){
        motorFrontRight.setPower(.6);
        motorFrontLeft.setPower(.6);
        motorBackLeft.setPower(.6);
        motorBackRight.setPower(.6);
    }
    
    private void AllStop(){
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
    
    private void AllBack(){
        motorFrontRight.setPower(-.6);
        motorFrontLeft.setPower(-.6);
        motorBackLeft.setPower(-.6);
        motorBackRight.setPower(-.6);
    }
    
    private void AllLeft(){
        motorFrontRight.setPower(.7);
        motorFrontLeft.setPower(-.4);
        motorBackLeft.setPower(.7);
        motorBackRight.setPower(-.4);
    }
    
    private void AllRight(){
        motorFrontRight.setPower(-.7);
        motorFrontLeft.setPower(.4);
        motorBackLeft.setPower(-.7);
        motorBackRight.setPower(.4);
    }
    
    private void TurnLeft(){
        motorFrontRight.setPower(.6);
        motorFrontLeft.setPower(-.6);
        motorBackLeft.setPower(-.6);
        motorBackRight.setPower(.6);
    }
    
    private void TurnRight(){
        motorFrontRight.setPower(-.6);
        motorFrontLeft.setPower(.6);
        motorBackLeft.setPower(.6);
        motorBackRight.setPower(-.6);
    }
}