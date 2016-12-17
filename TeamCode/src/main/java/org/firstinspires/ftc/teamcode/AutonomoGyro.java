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

import android.graphics.Color;

import com.qualcomm.hardware.hitechnic.HiTechnicNxtGyroSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 */

@Autonomous(name="Autonomo Gyro")
public class AutonomoGyro extends LinearOpMode
{

    /* Declare OpMode members. */
    HardwareOmniWheels robotDrive = new HardwareOmniWheels();   // Use a Omni Drive Train's hardware
    HardwareSensores sensores = new HardwareSensores();
    GyroSensor Gyro;
    HardwareServo servo = new HardwareServo();
    HardwareElevador elevador = new HardwareElevador();
    HardwareDisparador disparador = new HardwareDisparador();
    OpticalDistanceSensor odsSensor;  // Hardware Device Object
    ColorSensor colorSensor;  // Hardware Device Object
    LightSensor lightSensor;  // Hardware Device Object


       @Override
       public void runOpMode() throws InterruptedException {

       ElapsedTime period = new ElapsedTime();

       boolean bLedOnLL = true;
       boolean bLedOnOD = true;
       float hsvValues[] = {0F, 0F, 0F};
       boolean bLedOnCOLOR = true;
       double whiteLineIntensityValue = 23.6; //Replace accordingly
       double blackMatIntensityValue = 1.24;  //And here
       double targetIntensity = whiteLineIntensityValue + blackMatIntensityValue / 2;

       sensores.init(hardwareMap);
       robotDrive.init(hardwareMap);

       Gyro = hardwareMap.gyroSensor.get("giroscopioSensor");
       odsSensor = hardwareMap.opticalDistanceSensor.get("sensorOD");
       colorSensor = hardwareMap.colorSensor.get("sensorColor");
       lightSensor = hardwareMap.lightSensor.get("sensorLight");

       waitForStart();
       while (opModeIsActive())
       {
          while (colorSensor.alpha() < 5)
          {
              DriveForward(DRIVE_POWER);
              sleep(100);
          }

          StopDriving();

          while (odsSensor.getLightDetected() > 0.0189)
          { //value continuously checked

            if (Math.abs(colorSensor.alpha() - targetIntensity) <= 5)
            {
                DriveForward(DRIVE_POWER);
                sleep(100);
            }
            else if (colorSensor.alpha() > targetIntensity)
            {
                TurnLeft(DRIVE_POWER);
                sleep(100);
            }

            else if (colorSensor.alpha() < targetIntensity)
            {
                TurnRight(DRIVE_POWER);
                sleep(100);
            }
            else
            {
                System.out.println("Error");
                sleep(100);
                break;
               }
           }
          StopDriving();
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

