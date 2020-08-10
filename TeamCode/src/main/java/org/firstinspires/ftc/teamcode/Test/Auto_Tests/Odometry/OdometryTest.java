package org.firstinspires.ftc.teamcode.Test.Auto_Tests.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class OdometryTest extends LinearOpMode{

	private RunOdometry OD = new RunOdometry();
	private double step = 1;

	@Override
	public void runOpMode() throws InterruptedException{
		OD.init(hardwareMap);

		waitForStart();

		if (step == 1) {
			OD.runToPosition(10,10,1);
			step++;
		}

		if (step == 2) {
			OD.setMotorPower(0, 0, 0, 0);
			step++;
		}
	}
}
