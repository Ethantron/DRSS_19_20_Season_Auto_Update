package org.firstinspires.ftc.teamcode.Unused_Codes.Gyro;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@Disabled
@Autonomous (name = "Gyro_Turning", group = "Auto_Test")
public class Gyro_Turning extends LinearOpMode {

    //Motor Initialization
    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;

    double Power = .2; //Sets Motor Power
    double Range = 8; //Change this to change the range of degrees (Tolerance)
    double RangeDiv = Range / 2; //Evenly splits the range
    double WantedAngle = 0; //Wanted Angle
    double RangePlus = WantedAngle + RangeDiv; //adds tolerance to Wanted Angle
    double RangeMinus = WantedAngle - RangeDiv; //subtracts tolerance from Wanted Angle

    double step = 1;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    @Override
    public void runOpMode() {

        // Drivetrain initialization
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");

        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        //Set up IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();

        telemetry.addData("Status: ", "Initialized");
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 100);

        // Loop and update the dashboard
        while (opModeIsActive()) {

           // gyroTurn();

                if (step == 1) { //Turn 90 degrees
                    WantedAngle = 90;
                    gyroTurn();
                    step++;
                }

                if (step == 2) { //Move forward for 1 second
                    motorFrontRight.setPower(Power);
                    motorFrontLeft.setPower(Power);
                    motorBackRight.setPower(Power);
                    motorBackLeft.setPower(Power);
                    sleep(1000);
                    step++;
                }

                if (step == 3) { //Stop Motors
                    motorFrontRight.setPower(0);
                    motorFrontLeft.setPower(0);
                    motorBackRight.setPower(0);
                    motorBackLeft.setPower(0);
                }
        }
    }

    public void gyroTurn(/*double angle*/) {
       // WantedAngle = angles.firstAngle + angle; //Turns specified angle from current angle

        if (angles.firstAngle < RangeMinus) { //adjust robots by turning right
            motorFrontRight.setPower(-Power);
            motorFrontLeft.setPower(Power);
            motorBackRight.setPower(-Power);
            motorBackLeft.setPower(Power);
        }

        if (angles.firstAngle > RangePlus) { //adjust robots by turning left
            motorFrontRight.setPower(Power);
            motorFrontLeft.setPower(-Power);
            motorBackRight.setPower(Power);
            motorBackLeft.setPower(-Power);
        }

        while (opModeIsActive() && (angles.firstAngle < RangeMinus && angles.firstAngle > RangePlus)) { //While the robot is not centered
            telemetry.addData("Turning to: ", WantedAngle); //Displays the angle we are trying to turn to
            telemetry.update();
        }
        //Moves on once the robot is centered

        while (opModeIsActive() && (angles.firstAngle >= RangeMinus && angles.firstAngle <= RangePlus)) {
            //Stop Turning
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorBackRight.setPower(0);
            motorBackLeft.setPower(0);
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