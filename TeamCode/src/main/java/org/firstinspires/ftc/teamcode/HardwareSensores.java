/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.hitechnic.HiTechnicNxtGyroSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.LegacyModulePortDevice;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Optical Distance Sensor
 * It assumes that the ODS sensor is configured with a name of "sensor_ods".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@Disabled
public class HardwareSensores {
  public OpticalDistanceSensor odsSensor;  // Hardware Device Object
  public ColorSensor colorSensor;  // Hardware Device Object
  public LightSensor lightSensor;  // Hardware Device Object
  public GyroSensor Gyro;


  HardwareMap hwMap           =  null;
  private ElapsedTime period  = new ElapsedTime();

  public float hsvValues[] = {0F,0F,0F};
  public final float values[] = hsvValues;
  public final View relativeLayout = ((Activity) hwMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);
  public boolean bPrevStateCOLOR = false;
  public boolean bCurrStateCOLOR = false;
  public boolean bLedOnCOLOR = true;
  public double whiteLineIntensityValue = 23.6; //Replace accordingly
  public double blackMatIntensityValue = 1.24;  //And here
  public double targetIntensity = whiteLineIntensityValue + blackMatIntensityValue / 2;
  public boolean bPrevStateLL = false;
  public boolean bCurrStateLL = false;
  public boolean bLedOnLL = true;
  public boolean bLedOnOD = true;


  public HardwareSensores()
  {
  }

  public void init(HardwareMap ahwMap)
  {

    hwMap=ahwMap;

    Gyro = ahwMap.gyroSensor.get("giroscopioSensor");

    odsSensor = ahwMap.opticalDistanceSensor.get("sensorOD");
    colorSensor = ahwMap.colorSensor.get("sensorColor");
    lightSensor = ahwMap.lightSensor.get("sensorLight");
    lightSensor.enableLed(bLedOnLL);
    colorSensor.enableLed(bLedOnCOLOR);
    lightSensor.enableLed(bLedOnOD);

  }

  public void waitForTick(long periodMs) throws InterruptedException {

    long remaining = periodMs - (long) period.milliseconds();

    // sleep for the remaining portion of the regular cycle period.
    if (remaining > 0)
      Thread.sleep(remaining);

    // Reset the cycle clock for the next pass.
    period.reset();
  }
}