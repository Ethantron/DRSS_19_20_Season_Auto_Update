Autonomous directions reference sheet.



Stop:

        motorFrontRight.setPower(0);

        motorFrontLeft.setPower(0);

        motorBackLeft.setPower(0);

        motorBackRight.setPower(0);



Forward:

        motorFrontRight.setPower(-.6);

        motorFrontLeft.setPower(.6);

        motorBackLeft.setPower(.6);

        motorBackRight.setPower(-.6);



Reverse:

        motorFrontRight.setPower(.6);

        motorFrontLeft.setPower(-.6);

        motorBackLeft.setPower(-.6);

        motorBackRight.setPower(.6);



Turn Left:

        motorFrontRight.setPower(.6);

        motorFrontLeft.setPower(.6);

        motorBackLeft.setPower(.6);

        motorBackRight.setPower(.6);



Turn Right:

        motorFrontRight.setPower(-.6);

        motorFrontLeft.setPower(-.6);

        motorBackLeft.setPower(-.6);

        motorBackRight.setPower(-.6);



Strafe Left:

        motorFrontRight.setPower(-.7);

        motorFrontLeft.setPower(-.4);

        motorBackLeft.setPower(.7);

        motorBackRight.setPower(.4);



Strafe Right:

        motorFrontRight.setPower(.7);

        motorFrontLeft.setPower(.4);

        motorBackLeft.setPower(-.7);

        motorBackRight.setPower(-.4);



Timing Reference:

        ResetTime.reset();

        while (opModeIsActive() && (ResetTime.seconds() < 1)) {

            telemetry.addData("Where", "Step : 1 %2.5f S Elapsed", ResetTime.seconds());

            telemetry.update();

        }



Reset Time:

    ElapsedTime ResetTime = new ElapsedTime();



Spinner:

    Out:

        spinner.setPower(1);



    In:

        spinner.setPower(-1);



Arm:

    Up:

        shoulder.setPower(-.75);

        elbow.setPower(-.75);



    Down:

        shoulder.setPower(.75);

        elbow.setPower(.75);



Lift:

    Up:

        LiftArm.setPower(1);



    Down:

        LiftArm.setPower(-1);



Robert

     Full Speed 90* Turn

        Left.setPower(1);

        Right.setPower(-1);

        ResetTime.reset();

                while (opModeIsActive() && (ResetTime.seconds() < .4)) {

                    telemetry.addData("Where", "Step 2: %2.5f S Elapsed", ResetTime.seconds());

                    telemetry.update();

        }