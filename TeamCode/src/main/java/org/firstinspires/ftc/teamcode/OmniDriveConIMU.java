/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="OmniDriveConIMU")
//@Disabled
public class OmniDriveConIMU extends LinearOpMode
{

    /* Declare OpMode members. */
    HardwareOmniWheels robotDrive           = new HardwareOmniWheels();   // Use a Omni Drive Train's hardware
    HardwareServo servo = new HardwareServo();
    HardwareElevador elevador = new HardwareElevador();
    HardwareDisparador disparador = new HardwareDisparador();
    HardwarePelotota pelotota = new HardwarePelotota();
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;


    @Override
    public void runOpMode() throws InterruptedException
    {
        // Declares variables used on the program
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        robotDrive.init(hardwareMap);
        elevador.init(hardwareMap);
        disparador.init(hardwareMap);
        servo.init(hardwareMap);
        pelotota.init(hardwareMap);

        double position = 0.0;
        double positionSR = 0.0;
        double positionSL = 0.0;
        double positionSA = 0.0;

        // Send telemetry message to signify robot waiting;

        telemetry.addData("here comes dat bot", "Oh hey, waddup");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        // run until the end of the match (driver presses STOP)

        while (opModeIsActive())
        {
            if (gamepad2.x)
            {
                position = 1.0;

            }
            else if (gamepad2.b)
            {
                position = -0.15;
            } else
            {
                position = 0.0;
            }

            telemetry.addData("disparador: ", position) ;
            disparador.disparadorMotor.setPower(position);

            //para elevar la pelota grande
            {
                double elevadorDerecho = -gamepad2.right_stick_y;
                double elevadorIzquierdo = -gamepad2.left_stick_y;
                double direccionDerecho = 0;
                double direccionIzquierdo =0 ;


                if ( Math.abs(gamepad2.right_stick_y)> .07);
                {
                    direccionDerecho = elevadorDerecho;
                }

                if ( Math.abs(gamepad2.left_stick_y)> .07);
                {
                    direccionIzquierdo = elevadorIzquierdo;
                }

                if ( Math.abs(gamepad2.right_stick_y)< .07);
                {
                    direccionDerecho = direccionDerecho;
                }

                if ( Math.abs(gamepad2.left_stick_y)< .07);
                {
                    direccionIzquierdo= direccionIzquierdo;
                }


                telemetry.addData("elevador telescopico derecho: ", direccionDerecho);
                telemetry.addData("elevador telescopico izquierdo: ", direccionIzquierdo);

                pelotota.PL.setPower(direccionIzquierdo);
                pelotota.PR.setPower(direccionDerecho);

            }


            double banda_arriba = gamepad1.right_trigger;
            double banda_abajo = -gamepad1.left_trigger;
            double banda_direccion = 0;

            if(gamepad1.right_trigger > .01)
            {
               banda_direccion = banda_arriba;
            }

            else if(gamepad1.left_trigger > .1)
            {
                banda_direccion = banda_abajo;
            }
            else
            {
                banda_direccion=banda_direccion;
            }

            elevador.elevadorMotor.setPower(banda_direccion);
            telemetry.addData("banda", banda_direccion);

            double          pinzasOffset      = 0;                       // Servo mid position
            final double    velocidadpinzas      = 0.02 ;                   // sets rate to move servo

            if (gamepad2.right_bumper)
            {
                positionSL = .9;
                positionSR = .1;
                positionSA = .9;
                servo.SL.setPosition(positionSL);
                servo.SR.setPosition(positionSR);
                servo.SA.setPosition(positionSA);
            }
            else if (gamepad2.left_bumper)
            {
                positionSL = .1;
                positionSR = .9;
                positionSA = .1;
                servo.SL.setPosition(positionSL);
                servo.SR.setPosition(positionSR);
                servo.SA.setPosition(positionSA);
            }

            telemetry.addData("servoR: ", positionSR) ;
            telemetry.addData("servoL: ", positionSL) ;
            telemetry.addData("servoA: ", positionSA) ;
            //  Sets the turbo mode for the motors to normal when the right bumper is not pressed
            //  or to max speed (turbo) when it is pressed

            double turbo = 0;
            if (gamepad1.right_bumper)
            {
                robotDrive.turbo = 1;
                turbo = 1;
            }

            else
            {
                robotDrive.turbo = .5;

                turbo = .5;
            }

            telemetry.addData("velocidad", turbo);

            // Sets the joystick values to variables for better math understanding
            // The Y axis goes

            robotDrive.y1 = gamepad1.left_stick_y;
            robotDrive.x1 = gamepad1.left_stick_x;
            robotDrive.x2 = gamepad1.right_stick_x;
            double y1 = gamepad1.left_stick_y;
            double x1 = gamepad1.left_stick_x;
            double x2 = gamepad1.right_stick_x;



            // sets the math necessary to control the motors to variables
            // The left stick controls the axial movement
            // The right sick controls the rotation

            robotDrive.frontRightPower = robotDrive.y1 - robotDrive.x2 - robotDrive.x1;
            robotDrive.backRightPower = robotDrive.y1 - robotDrive.x2 + robotDrive.x1;
            robotDrive.frontLeftPower = robotDrive.y1 + robotDrive.x2 + robotDrive.x1;
            robotDrive.backLeftPower = robotDrive.y1 + robotDrive.x2 - robotDrive.x1;
            double frontRightPower  = y1 - x2 - x1;
            double backRightPower   = y1 - x2 + x1;
            double frontLeftPower   = y1 + x2 + x1;
            double backLeftPower    = y1 + x2 - x1;

            // Normalize the values so neither exceed +/- 1.0

            robotDrive.max = Math.max(Math.abs(robotDrive.frontRightPower), Math.max(Math.abs(robotDrive.backRightPower),
            Math.max(Math.abs(robotDrive.frontLeftPower), Math.abs(robotDrive.backLeftPower))));
            double max = Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backRightPower),
            Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower))));
//
            if (robotDrive.max > 1.0)
            {
            robotDrive.frontRightPower /= robotDrive.max;
            robotDrive.backRightPower /= robotDrive.max;
            robotDrive.frontLeftPower /= robotDrive.max;
            robotDrive.backLeftPower /= robotDrive.max;
            }
            if (max > 1.0)
            {
            frontRightPower /= max;
            backRightPower  /= max;
            frontLeftPower  /= max;
            backLeftPower   /= max;
            }

            // sets the speed for the motros with the turbo multiplier
//
            robotDrive.frontRightPower *= robotDrive.turbo;
            robotDrive.backRightPower *= robotDrive.turbo;
            robotDrive.frontLeftPower *= robotDrive.turbo;
            robotDrive.backLeftPower *= robotDrive.turbo;
            frontRightPower *= turbo;
            backRightPower  *= turbo;
            frontLeftPower  *= turbo;
            backLeftPower   *= turbo;

//
            robotDrive.frontRightMotor.setPower(robotDrive.frontRightPower);
            robotDrive.backRightMotor.setPower(robotDrive.backRightPower);
            robotDrive.frontLeftMotor.setPower(robotDrive.frontLeftPower);
            robotDrive.backLeftMotor.setPower(robotDrive.backLeftPower);
            telemetry.addData("front right:", frontRightPower);
            telemetry.addData("back right:", backRightPower);
            telemetry.addData("front left:", frontLeftPower);
            telemetry.addData("back left:", backLeftPower);

            // Send telemetry message to signify robot running;

            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.

            robotDrive.waitForTick(40);

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
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            gravity  = imu.getGravity();
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
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
