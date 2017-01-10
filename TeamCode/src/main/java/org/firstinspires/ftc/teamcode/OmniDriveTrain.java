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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

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

@TeleOp(name="OmniDriveTrain")
@Disabled
public class OmniDriveTrain extends LinearOpMode
{

    /* Declare OpMode members. */
    HardwareOmniWheels robotDrive           = new HardwareOmniWheels();   // Use a Omni Drive Train's hardware
    HardwareServo servo = new HardwareServo();
    HardwareElevador elevador = new HardwareElevador();
    HardwareDisparador disparador = new HardwareDisparador();
    HardwarePelotota pelotota = new HardwarePelotota();

    double          pinzasOffset      = 0;                       // Servo mid position
    final double    velocidadpinzas      = 0.02 ;                   // sets rate to move servo

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

        // Send telemetry message to signify robot waiting;

        telemetry.addData("here comes dat bot", "Oh hey, waddup");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        // run until the end of the match (driver presses STOP)

        while (opModeIsActive())
        {
            //Sets the turbo mode for the motors to normal when the right bumper is not pressed
            // or to max speed (turbo) when it is pressed

            if (gamepad2.a)
            {
                Dispara(1, 1440);
            }

            // encoderDrive(.5, 12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
            // encoderDrive(.5, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

            //para elevar la pelota grande
            {
                double subir = gamepad2.right_trigger;
                double bajar = -gamepad2.left_trigger;
                double direccion = 0;


                if ( gamepad2.right_trigger < .01  && gamepad2.left_trigger < .01)
                {
                    direccion = direccion;
                }

                if (gamepad2.right_trigger > .01 && gamepad2.left_trigger < .01)
                {
                    direccion = subir;
                }

                if (gamepad2.right_trigger < .01 && gamepad2.left_trigger > .01)
                {
                    direccion = bajar;
                }

                if (gamepad2.right_trigger > .1 && gamepad2.left_trigger > .1)
                {
                    direccion = direccion;
                }

                pelotota.PL.setPower(direccion);
                pelotota.PR.setPower(direccion);

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



            // Use gamepad left & right Bumpers to open and close the claw
            if (gamepad2.right_bumper)
                pinzasOffset += velocidadpinzas;
            else if (gamepad2.left_bumper)
                pinzasOffset -= velocidadpinzas;

            // Move both servos to new position.  Assume servos are mirror image of each other.
            pinzasOffset = Range.clip(pinzasOffset, -0.5, 0.5);
            servo.SR.setPosition(servo.Arm_Max/2 + pinzasOffset);
            servo.SL.setPosition(servo.Arm_Max/2 - pinzasOffset);

            if (gamepad1.right_bumper)
            {
                robotDrive.turbo = 1;
            }

            else
            {
                robotDrive.turbo = .5;
            }

            // Sets the joystick values to variables for better math understanding
            // The Y axis goes

            robotDrive.y1 = gamepad1.left_stick_y;
            robotDrive.x1 = gamepad1.left_stick_x;
            robotDrive.x2 = gamepad1.right_stick_x;

            // sets the math necessary to control the motors to variables
            // The left stick controls the axial movement
            // The right sick controls the rotation

            robotDrive.frontRightPower = robotDrive.y1 - robotDrive.x2 - robotDrive.x1;
            robotDrive.backRightPower = robotDrive.y1 - robotDrive.x2 + robotDrive.x1;
            robotDrive.frontLeftPower = robotDrive.y1 + robotDrive.x2 + robotDrive.x1;
            robotDrive.backLeftPower = robotDrive.y1 + robotDrive.x2 - robotDrive.x1;

            // Normalize the values so neither exceed +/- 1.0

            robotDrive.max = Math.max(Math.abs(robotDrive.frontRightPower), Math.max(Math.abs(robotDrive.backRightPower),
            Math.max(Math.abs(robotDrive.frontLeftPower), Math.abs(robotDrive.backLeftPower))));

            if (robotDrive.max > 1.0)
            {
            robotDrive.frontRightPower /= robotDrive.max;
            robotDrive.backRightPower /= robotDrive.max;
            robotDrive.frontLeftPower /= robotDrive.max;
            robotDrive.backLeftPower /= robotDrive.max;
            }

            // sets the speed for the motros with the turbo multiplier

            robotDrive.frontRightPower *= robotDrive.turbo;
            robotDrive.backRightPower *= robotDrive.turbo;
            robotDrive.frontLeftPower *= robotDrive.turbo;
            robotDrive.backLeftPower *= robotDrive.turbo;

            robotDrive.frontRightMotor.setPower(robotDrive.frontRightPower);
            robotDrive.backRightMotor.setPower(robotDrive.backRightPower);
            robotDrive.frontLeftMotor.setPower(robotDrive.frontLeftPower);
            robotDrive.backLeftMotor.setPower(robotDrive.backLeftPower);

            // Send telemetry message to signify robot running;

            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.

            robotDrive.waitForTick(40);

        }
    }

    public void Dispara (double power, int distance)
    {
        disparador.disparadorMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        disparador.disparadorMotor.setTargetPosition(distance);
        disparador.disparadorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        disparador.disparadorMotor.setPower(power);

        while (disparador.disparadorMotor.isBusy())
        {

        }

        disparador.disparadorMotor.setPower(0);
        disparador.disparadorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
