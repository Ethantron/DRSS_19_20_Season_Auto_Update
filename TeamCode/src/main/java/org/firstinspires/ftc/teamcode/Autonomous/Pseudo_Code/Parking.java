package org.firstinspires.ftc.teamcode.Autonomous.Pseudo_Code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Parking", group= "Pseudo_Code")
public class Parking extends LinearOpMode {

    ElapsedTime ResetTime = new ElapsedTime();

    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;
    double step = 0;
    double notblack = 0;


    @Override
    public void runOpMode() {

        ResetTime.reset(); //Resets timer to make sure we get accurate time reading

        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);

        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);

        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);

        motorBackRight = hardwareMap.dcMotor.get("BR");
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        //Orient the Robot (Gyro Code)
        if (step == 0) {
            //Turn using Gyro
            step++;
        }

        //Move forward
        if (step == 1) {
            motorFrontRight.setPower(.6);
            motorFrontLeft.setPower(.6);
            motorBackLeft.setPower(.6);
            motorBackRight.setPower(.6);
            sleep(100);
            step++;
        }

        //Start looking for line
        if (step == 2) {
            //Scan w/ Color Sensor
            step++;
        }

        //Does it see the line?
        if (step == 3) {
            ResetTime.reset(); //Reset Timer so we know when we have been scanning too long
            if (notblack >= 200 && ResetTime.seconds() <= 3) { //Checks if it is red or blue
                motorFrontRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
            }
            else if (ResetTime.seconds() > 3) { //If not red or blue and timer has gone for longer than 3 seconds
                    motorFrontRight.setPower(0);
                    motorFrontLeft.setPower(0);
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(0);
            }
        }
    }
}
