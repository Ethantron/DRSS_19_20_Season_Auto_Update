package org.firstinspires.ftc.teamcode.Test.Auto_Tests;

// Imports for the code
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous(name = "Please_Park", group= "Auto_Tests")
public class Please_park extends LinearOpMode {

    // Robot definitions
    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    double step = 0;

    @Override
    public void runOpMode() {

        // Drive train initialization
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");

        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        // Color sensor initialization
        sensorColor = hardwareMap.get(ColorSensor.class, "color_sensor");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color_sensor");

        // Color Sensor Values
        // "hsvValues" is an array that will hold the hue, saturation, and value information
        float hsvValues[] = {0F, 0F, 0F};

        // "values" is a reference to the hsvValues array
        final float values[] = hsvValues;

        // Sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attenuate the measured values.
        final double SCALE_FACTOR = 255;

        telemetry.addData("Status: ", "Initialized");
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();
        }

        while(!(hsvValues[0] > 100)){
            motorFrontRight.setPower(.2);
            motorFrontLeft.setPower(.2);
            motorBackLeft.setPower(.2);
            motorBackRight.setPower(.2);
        }
    }
}
