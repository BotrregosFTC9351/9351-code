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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


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

@Autonomous(name="AutonomoODS")
public class AutonomoODS extends LinearOpMode
{

    /* Declare OpMode members. */
    HardwareOmniWheels robotDrive           = new HardwareOmniWheels();   // Use a Omni Drive Train's hardware
    HardwareServo servo = new HardwareServo();
    HardwareElevador elevador = new HardwareElevador();
    HardwareDisparador disparador = new HardwareDisparador();

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Declares variables used on the program
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        final double PERFECT_COLOR_VALUE = 0.2;
        OpticalDistanceSensor ODS = null;

        robotDrive.init(hardwareMap);
        elevador.init(hardwareMap);
        disparador.init(hardwareMap);
        servo.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Color Value", ODS.getLightDetected());

        while (true) {
            double correction = (PERFECT_COLOR_VALUE - ODS.getLightDetected());
            // Sets the powers so they are no less than .075 and apply to correction
            if (correction <= 0) {
                robotDrive.backLeftPower = .075d - correction;
                robotDrive.frontLeftPower = .075d - correction;
                robotDrive.frontRightPower = .075d;
                robotDrive.backRightPower = .075d;
            } else {
                robotDrive.backLeftPower = .075d;
                robotDrive.frontLeftPower = .075d;
                robotDrive.frontRightPower = .075d + correction;
                robotDrive.backRightPower = .075d + correction;
            }
            // Sets the powers to the motors
            robotDrive.frontLeftMotor.setPower(robotDrive.frontLeftPower);
            robotDrive.backLeftMotor.setPower(robotDrive.backLeftPower);
            robotDrive.frontRightMotor.setPower(robotDrive.frontRightPower);
            robotDrive.backRightMotor.setPower(robotDrive.backRightPower);
        }

    }
    double DRIVE_POWER = 1.0;

    public void DriveForward (double power)
    {
        robotDrive.frontLeftMotor.setPower(power);
        robotDrive.frontRightMotor.setPower(power);
        robotDrive.backLeftMotor.setPower(power);
        robotDrive.backRightMotor.setPower(power);
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



}

