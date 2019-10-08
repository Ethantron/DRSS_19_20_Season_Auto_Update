package org.firstinspires.ftc.teamcode.Test.Auto_Tests;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Parking_Test", group= "Auto_Tests")
public class Parking_Test extends LinearOpMode {

    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    double step = 0;

    @Override
    public void runOpMode() {

        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);

        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);

        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);

        motorBackRight = hardwareMap.dcMotor.get("BR");
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        sensorColor = hardwareMap.get(ColorSensor.class, "color_sensor");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color_sensor");

        //Color Sensor Values
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        waitForStart();

        while (opModeIsActive()) {
            //Color Sensor Code

            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("Step: ", step);
            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();

            //Orient the Robot (Gyro Code)
            if (step == 0) {
                //Turn using Gyro
                step++;
            }

            //Move forward
            if (step == 1) {
                motorFrontRight.setPower(.3);
                motorFrontLeft.setPower(.3);
                motorBackLeft.setPower(.3);
                motorBackRight.setPower(.3);
            }

            //Does it see the line?
            while (step == 1) {
                if (opModeIsActive()) {
                        // convert the RGB values to HSV values.
                        // multiply by the SCALE_FACTOR.
                        // then cast it back to int (SCALE_FACTOR is a double)
                        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                                (int) (sensorColor.green() * SCALE_FACTOR),
                                (int) (sensorColor.blue() * SCALE_FACTOR),
                                hsvValues);

                        // send the info back to driver station using telemetry function.
                        telemetry.addData("Step: ", step);
                        telemetry.addData("Alpha", sensorColor.alpha());
                        telemetry.addData("Red  ", sensorColor.red());
                        telemetry.addData("Green", sensorColor.green());
                        telemetry.addData("Blue ", sensorColor.blue());
                        telemetry.addData("Hue", hsvValues[0]);
                        telemetry.update();

                }
                if (hsvValues[0] > 100 || sensorColor.blue() > 100) { //Checks if it is red or blue
                    step++;
                }
            }

            if (step == 2) {
                motorFrontRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
            }


        }

    }
}

