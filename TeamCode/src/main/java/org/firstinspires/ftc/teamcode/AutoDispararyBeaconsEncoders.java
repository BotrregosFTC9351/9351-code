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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoDispararyBeaconsEncoders", group="Pushbot")
public class AutoDispararyBeaconsEncoders extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareOmniWheels         robotDrive   = new HardwareOmniWheels();   // Use a Pushbot's hardware

    private ElapsedTime     runtime = new ElapsedTime();

    ColorSensor sensorRGB;
    DeviceInterfaceModule cdim;

    DcMotor disparadorMotor = null;


    static final int LED_CHANNEL = 6;

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    static final double     COUNTS_PER_MOTOR_REVD    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTIOND   = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHESD   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCHD       = (COUNTS_PER_MOTOR_REVD * DRIVE_GEAR_REDUCTIOND) /
            (WHEEL_DIAMETER_INCHESD * 3.1415);
    static final double     DRIVE_SPEEDD             = 0.6;
    static final double     TURN_SPEEDD            = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robotDrive.init(hardwareMap);

        robotDrive.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotDrive.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotDrive.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotDrive.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robotDrive.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotDrive.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotDrive.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotDrive.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        float hsvValues[] = {0F,0F,0F};
        final float values[] = hsvValues;
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);
        boolean bPrevState = false;
        boolean bCurrState = false;
        boolean bLedOn = true;
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
        sensorRGB = hardwareMap.colorSensor.get("adargbsensor");
        cdim.setDigitalChannelState(LED_CHANNEL, bLedOn);

        disparadorMotor = hardwareMap.dcMotor.get("disp");
        disparadorMotor.setDirection(DcMotor.Direction.FORWARD);
        disparadorMotor.setPower(0);
        disparadorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  -.4, -.4,  -.37, -.37, 1.0);  // S1: Forward 3 Inches with 2 Sec timeout
        disparadorMotor.setPower(1);
        sleep(700);
        disparadorMotor.setPower(0);
        encoderDrive(TURN_SPEED,   .35, -.35, -.32, .32, 1.0);  // S2: Slide Left
        encoderDrive(DRIVE_SPEED, -.45, -.45,  -.4, -.4, .95);  // S3: Forward 3 Inches with 2 Sec timeout
        encoderDrive(TURN_SPEED,   -.20, -.20, .20, .20, .85);  // S2: Turn Right 3 Inches with 2 Sec timeout
        encoderDrive(DRIVE_SPEED, .4, .4, .39, .39, 1.2);  // S3: Reverse 3 Inches with 2 Sec timeout
        Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });
        sleep(2001 );
        if (hsvValues[0] > 187 && hsvValues[0] < 230) {
            encoderDrive(DRIVE_SPEED, .2, .2, .15, .15, 1.0);  // S3: Reverse 3 Inches with 2 Sec timeout
        } else {
            encoderDrive(TURN_SPEED,   .30, -.30, -.27, .27, 1.0);  // S2: Slide Left
            encoderDrive(DRIVE_SPEED, .2, .2, .15, .15, 1.0);  // S3: Reverse 3 Inches with 2 Sec timeout
            sleep(500);
        }
        encoderDrive(DRIVE_SPEED, -.4, -.4,  -.35, -.35, 1.0);  // S3: Forward 3 Inches with 2 Sec timeout
        encoderDrive(TURN_SPEED,   .6, -.6, -.55, .55, 2.0);  // S2: Slide Left
        encoderDrive(DRIVE_SPEED, .35, .35, .32, .32, 1.0);  // S3: Reverse 3 Inches with 2 Sec timeout
        if (hsvValues[0] > 187 && hsvValues[0] < 230) {
            encoderDrive(DRIVE_SPEED, .4, .4, .35, .35, 1.0);  // S3: Reverse 3 Inches with 2 Sec timeout
        } else {
            encoderDrive(TURN_SPEED,   .30, -.30, -.27, .27, 1.0);  // S2: Slide Left
            encoderDrive(DRIVE_SPEED, .2, .2, .15, .15, 1.0);  // S3: Reverse 3 Inches with 2 Sec timeout
            sleep(500);
        }
    }


    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double backLeftInches, double frontLeftInches,  double backRightInches, double frontRightInches,
                             double timeoutS) {
        int newBackLeftTarget;
        int newFrontLeftTarget;
        int newBackRightTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = robotDrive.backLeftMotor.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            newFrontLeftTarget = robotDrive.frontLeftMotor.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = robotDrive.backRightMotor.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);
            newFrontRightTarget = robotDrive.frontRightMotor.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            robotDrive.backLeftMotor.setTargetPosition(newBackLeftTarget);
            robotDrive.frontLeftMotor.setTargetPosition(newFrontLeftTarget);
            robotDrive.backRightMotor.setTargetPosition(newBackRightTarget);
            robotDrive.frontRightMotor.setTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            robotDrive.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotDrive.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotDrive.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotDrive.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robotDrive.backLeftMotor.setPower(Math.abs(speed));
            robotDrive.backRightMotor.setPower(Math.abs(speed));
            robotDrive.frontLeftMotor.setPower(Math.abs(speed));
            robotDrive.frontRightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robotDrive.backLeftMotor.isBusy() && robotDrive.backRightMotor.isBusy() && robotDrive.frontRightMotor.isBusy() )) {
            }

            // Stop all motion;
            robotDrive.backLeftMotor.setPower(0);
            robotDrive.backRightMotor.setPower(0);
            robotDrive.frontLeftMotor.setPower(0);
            robotDrive.frontRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robotDrive.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robotDrive.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robotDrive.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robotDrive.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(250);   // optional pause after each move
        }
    }
}
