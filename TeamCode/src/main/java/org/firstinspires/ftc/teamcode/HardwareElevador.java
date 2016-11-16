package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class declares and initializes all hardware needed to control a Omni Drive Train
 * Motor channel:  Front Right Motor:        "fr"
 * Motor channel:  Back Right Motor:        "br"
 * Motor channel:  Front Left Motor:        "fl"
 * Motor channel:  Back Left Motor:        "bl"
 *
 */
public class HardwareElevador
{
    /* Public OpMode members. */
    public DcMotor elevadorMotor = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareElevador()
    {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap)
    {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        elevadorMotor = hwMap.dcMotor.get("elev");

        elevadorMotor.setDirection(DcMotor.Direction.FORWARD);

        elevadorMotor.setPower(0.0);

        elevadorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException
    {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
