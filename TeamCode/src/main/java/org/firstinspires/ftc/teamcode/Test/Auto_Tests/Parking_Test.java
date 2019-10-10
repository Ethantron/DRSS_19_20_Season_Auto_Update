// Defines where the code is in the project
package org.firstinspires.ftc.teamcode.Test.Auto_Tests;

// Imports for the code
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

// Defines the codes name
@Autonomous(name = "Parking_Test", group= "Auto_Tests")
public class Parking_Test extends LinearOpMode{

    // Robot definitions
    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    double step = 0;

    @Override
    public void runOpMode(){

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

        while (opModeIsActive()){
            // Color Sensor Code

            // Convert the RGB values to HSV values.
            // Multiply by the SCALE_FACTOR.
            // Cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            // Send the info back to driver station using telemetry function.
            telemetry.addData("Step: ", step);
            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();

            // Move forward
            if (step == 0){
                motorFrontRight.setPower(.3);
                motorFrontLeft.setPower(.3);
                motorBackLeft.setPower(.3);
                motorBackRight.setPower(.3);
            }

            // Does it see the line?
            while (step == 0){
                if (opModeIsActive()){
                        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                                (int) (sensorColor.green() * SCALE_FACTOR),
                                (int) (sensorColor.blue() * SCALE_FACTOR),
                                hsvValues);

                        // Send the info back to driver station using telemetry function.
                        telemetry.addData("Step: ", step);
                        telemetry.addData("Alpha", sensorColor.alpha());
                        telemetry.addData("Red  ", sensorColor.red());
                        telemetry.addData("Green", sensorColor.green());
                        telemetry.addData("Blue ", sensorColor.blue());
                        telemetry.addData("Hue", hsvValues[0]);
                        telemetry.update();
                }
                if (hsvValues[0] > 100 || sensorColor.blue() > 100){ // Checks if it is red or blue
                    step++;
                }
            }

            if (step == 2){
                motorFrontRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
            }
        }
    }
}

