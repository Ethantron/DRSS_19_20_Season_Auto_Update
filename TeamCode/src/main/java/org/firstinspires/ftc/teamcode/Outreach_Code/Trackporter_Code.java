package org.firstinspires.ftc.teamcode.Outreach_Code;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Trackporter_Code", group = "Outreach_Code")
public class Trackporter_Code extends OpMode {

    Servo leftMotor;
    Servo rightMotor;

    Servo pan;
    Servo tilt;

    DcMotor light;

    boolean smoothToggle = true;
    boolean speed = false;

    double valAdjust1 = 0;
    double valAdjust2 = 0;

    public void init(){
        leftMotor = hardwareMap.servo.get("Servoleft");
        leftMotor.setDirection(Servo.Direction.FORWARD);

        rightMotor = hardwareMap.servo.get("Servoright");
        rightMotor.setDirection(Servo.Direction.FORWARD);

        pan = hardwareMap.servo.get("Pan");

        tilt = hardwareMap.servo.get("Tilt");

        light = hardwareMap.dcMotor.get("Light");
        light.setDirection(DcMotor.Direction.FORWARD);

        leftMotor.setPosition(0.5);
        rightMotor.setPosition(0.5);

        pan.setPosition(0.5);

        telemetry.addData("Left Motor: ", "Initialized");
        telemetry.addData("Right Motor: ", "Initialized");
        telemetry.addData("Tilt servo: ", "Initialized");
        telemetry.addData("Left Motor: ", "Initialized");
        telemetry.addData("Left Motor: ", "Initialized");
        telemetry.addData("Left Motor: ", "Initialized");
    }
}
