package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous ;
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

@Autonomous(name="EjemploIMU")
@Disabled
public class EjemploIMU extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    HardwareOmniWheels robotDrive = new HardwareOmniWheels();   // Use a Omni Drive Train's hardware

    @Override
    public void runOpMode() {
        robotDrive.init(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        composeIMUTelemetry(); //set up telemetry for gyro
//All code above is begun upon init; all code below is begun upon start
        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        while (angles.firstAngle < 26){
            TurnRight(.5);
            sleep(1000);
        }
    }


    public void composeIMUTelemetry() {

// At the beginning of each telemetry update, grab a bunch of data
// from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
// Acquiring the angles is relatively expensive; we don't want
// to do that in each of the three items that need that info, as that's
// three times the necessary expense.
            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                });
    }

//----------------------------------------------------------------------------------------------
// Formatting
//----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit , angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    double DRIVE_POWER = .7;

    public void DriveForward (double power)
    {
        robotDrive.frontLeftMotor.setPower(-power);
        robotDrive.frontRightMotor.setPower(-power);
        robotDrive.backLeftMotor.setPower(-power);
        robotDrive.backRightMotor.setPower(-power);
    }
    public void TurnRight (double power)
    {
        robotDrive.frontLeftMotor.setPower(power);
        robotDrive.frontRightMotor.setPower(-power);
        robotDrive.backLeftMotor.setPower(power);
        robotDrive.backRightMotor.setPower(-power);
    }
    public void TurnLeft (double power)
    {
        robotDrive.frontLeftMotor.setPower(-power);
        robotDrive.frontRightMotor.setPower(power);
        robotDrive.backLeftMotor.setPower(-power);
        robotDrive.backRightMotor.setPower(power);
    }
    public void StopDriving ()
    {
        DriveForward(0);
    }

    public void DriveForwardTime (double power, long time) throws InterruptedException
    {
        DriveForward(power);
        Thread.sleep(time);
    }
    public void TurnRightTime (double power, long time) throws InterruptedException
    {
        TurnRight(power);
        Thread.sleep(time);
    }
    public void TurnLeftTime (double power, long time) throws InterruptedException
    {
        TurnLeft(power);
        Thread.sleep(time);
    }
}

