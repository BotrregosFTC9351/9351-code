package org.firstinspires.ftc.teamcode;

import android.view.ViewDebug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.VuforiaPoseMatrix;

import java.text.DecimalFormat;
import java.util.concurrent.DelayQueue;
import java.util.concurrent.TimeUnit;

/**
 * Created by saulo on 12/10/2016.
 */
@TeleOp(name="vuforia tutorial")
@Disabled
public class vuforiaOP extends LinearOpMode {

    //HardwareOmniWheels robot = new HardwareOmniWheels ();

    @Override
    public void runOpMode() throws InterruptedException {

        //robot.init(hardwareMap);

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AWmM31n/////AAAAGQGLpAwyxEQRgx0+yduk09BKNJJgdFC4Ti1nfZAP8PSlO1C3uNWOPW/0eQefL19+S2VeUjx7Z3ZQGmYYLkqQfRiG2SB//RpH7+NIYK1TEkGG9pyiqYrlY4FJvkOOQ/bRdJSw7CjpRKaRqVUznO1srjlbu5HU3bDpgEYSzKJxmKHmcPMwAT5fmGFJHRw/KmMSmb4i9JpCd5nN9+rFTqRl2bFMnxjLqzCfwV+Dv8fiiZumZMLNEsvpi8fyAV1wRwz5vq44r595p28zsuA97E19hqe+KBlxOdomtIkZM9ThnUkaQZiH53WkuagKRigvRnhRBP4lX5W92nue5qfBp0Fnt5DEuW0oj6YEho4ulCK92sOu" ;
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");

        beacons.get(0).setName("wheels");
        beacons.get(1).setName("tools");
        beacons.get(2).setName("lego");
        beacons.get(3).setName("gears");

        double IntroSpeed = -0.2;
        double IdleSpeed = 0.0;
        double RotationSpeed = -0.15;
        double LowSpeed = -0.1;

        waitForStart();

        beacons.activate();
        while(opModeIsActive()){
            for (VuforiaTrackable beac : beacons){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)beac.getListener()).getPose();
                OpenGLMatrix rawpose = ((VuforiaTrackableDefaultListener)beac.getListener()).getRawPose();


                DecimalFormat formatter = new DecimalFormat("#0.00");

                //robot.backLeftMotor.setPower(IntroSpeed);
                //robot.backRightMotor.setPower(-IntroSpeed);
                //robot.frontLeftMotor.setPower(-IntroSpeed);
                //robot.frontRightMotor.setPower(IntroSpeed);
                telemetry.addData ("speed: ", IntroSpeed);
                telemetry.update();


                if(pose != null) {

                    VectorF translation = pose.getTranslation();
                    double angulo = 90*(pose.get(0,0));
                    double signo = (pose.get(2,0));
                    float transX = translation.get(0);
                    float transZ = translation.get(2);
                    if(signo<0){
                        angulo=-angulo;
                    }

                    if (transZ==500){
                        telemetry.addData("new speed: ", LowSpeed);
                        telemetry.update();
                    }
                    else if (transZ>500){
                        telemetry.addData ("stop ",IdleSpeed);
                        telemetry.update();
                    }

                   // telemetry.addData("esta es la distancia en x: ", transX);
                   // telemetry.addData("esta es la distancia en z: ", transZ);
                   // telemetry.addData("este es el angulo: ", formatter.format(angulo));
                    // telemetry.addData("",pose);
                    //telemetry.addData("", "");
                    // telemetry.addData("",rawpose);
                    //telemetry.update();

                        //robot.backLeftMotor.setPower(LowSpeed);
                        //robot.backRightMotor.setPower(-LowSpeed);
                        //robot.frontLeftMotor.setPower(-LowSpeed);
                        //robot.frontRightMotor.setPower(LowSpeed);
                        //telemetry.addData("new speed: ", LowSpeed);
                        //telemetry.update();

                        //robot.backLeftMotor.setPower(IdleSpeed);
                        //robot.backRightMotor.setPower(IdleSpeed);
                        //robot.frontLeftMotor.setPower(IdleSpeed);
                        //robot.frontRightMotor.setPower(IdleSpeed);
                        //telemetry.addData ("stop ",IdleSpeed);
                        //telemetry.update();

                }
            }
            telemetry.update();
        }
    }
}
