package org.firstinspires.ftc.teamcode.Outreach_Code;

		import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
		import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
		import com.qualcomm.robotcore.hardware.DcMotor;
		import com.qualcomm.robotcore.hardware.DcMotorSimple;
		import com.qualcomm.robotcore.hardware.GyroSensor;
		import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Tiny_Tank_Auto", group= "Outreach_Robots")
public class Tiny_Tank_Auto extends LinearOpMode {
	double step = 0;
	Servo Servoleft;
	Servo Servoright;
	DcMotor Light;
	Servo Pan;
	Servo Tilt;
	GyroSensor Gyro;

	int heading = 0;
	int STHEAD;

	public void runOpMode() {

		Servoleft = hardwareMap.servo.get("Servoleft");
		Servoleft.setDirection(Servo.Direction.FORWARD);

		Servoright = hardwareMap.servo.get("Servoright");
		Servoright.setDirection(Servo.Direction.FORWARD);

		Light = hardwareMap.dcMotor.get("RedLight");
		Light.setDirection(DcMotorSimple.Direction.FORWARD);

		Pan = hardwareMap.servo.get("Pan"); //Continuous Rotation Servo
		Tilt = hardwareMap.servo.get("Tilt"); //180 Degree Servo

		Tilt.setPosition(0.5); //Sets servo into "Zero" position
		Pan.setPosition(0.5);
		Servoright.setPosition(0.5);
		Servoleft.setPosition(0.5);
		Light.setPower(-1);

/*        Gyro = hardwareMap.gyroSensor.get("gyro");
        // Calibrate the gyro
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        Gyro.calibrate();
        // While the gyro is calibrating wait
        while (Gyro.isCalibrating())  {
            sleep(50);
            idle();
        }
        // Tell us when the gyro is done calibrating
        telemetry.addData(">", "Gyro Calibrated.");
        STHEAD = Gyro.getHeading(); //Stores calibrated heading
*/
		waitForStart();

		go();

 /*       while (opModeIsActive()) {
            // Get the current heading of the robot
            heading = Gyro.getHeading();
            telemetry.addData("Heading=",heading);
            telemetry.addData("STHEAD=",STHEAD);
            telemetry.update();
        } */
	}
	public void go(){
		//Square
        /*if (step == 0) { //States that if step = 0, then run this
            Servoleft.setPosition(0);//Set Left Motor to full Power
            Servoright.setPosition(0);//Set Right Motor to full Power
            sleep(3500); //Wait for robot to move to position
            Servoleft.setPosition(.45); //Set Left Motor to partial power to prevent dragging
            Servoright.setPosition(0); //Set Right Motor to full Power
            sleep(2200); //Wait for robot to move to position
            step++; //Move to step 1
        }
        if (step == 1) { //States that if step = 1, then run this
            sleep(1); //Wait 1 millisecond
            step = 0; //Set to step 0, restart loop
        } */

		//Spiral
		if (step == 0) {
			Servoleft.setPosition(0);
			Servoright.setPosition(.35);
			sleep(1000000);
			step++;
		}

		if (step == 1) {
			sleep(1);
			step--;
			go();
		}

		// Compass Control
       /* if (step == 0) {
            Servoleft.setPosition(0);//Set Left Motor to full Power
            Servoright.setPosition(0);//Set Right Motor to full Power
            sleep(3500); //Wait for robot to move to position
            step++;
        }
        if (step == 1) {
            Servoleft.setPosition(.45); //Set Left Motor to partial power to prevent dragging
            Servoright.setPosition(0); //Set Right Motor to full Power
            if (heading == (STHEAD + 90)) {
                Servoleft.setPosition(.5);
                Servoright.setPosition(.5);
                step++;
            }
        }
        if (step == 2) {
            Servoleft.setPosition(.45); //Set Left Motor to partial power to prevent dragging
            Servoright.setPosition(0); //Set Right Motor to full Power
            if (heading == (STHEAD + 180)) {
                Servoleft.setPosition(.5);
                Servoright.setPosition(.5);
                step ++;
            }
        }
        if (step == 3) {
            Servoleft.setPosition(.45); //Set Left Motor to partial power to prevent dragging
            Servoright.setPosition(0); //Set Right Motor to full Power
            if (heading == (STHEAD + 270)) {
                Servoleft.setPosition(.5);
                Servoright.setPosition(.5);
                step ++;
            }
            if (step == 4) {
                Servoleft.setPosition(.45); //Set Left Motor to partial power to prevent dragging
                Servoright.setPosition(0); //Set Right Motor to full Power
                if (heading == (STHEAD)) {
                    Servoleft.setPosition(.5);
                    Servoright.setPosition(.5);
                    step ++;
                }
            }*/
	}

}