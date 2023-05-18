package org.firstinspires.ftc.teamcode.components;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/*
 * https://www.youtube.com/watch?v=FmNkfdY2lnw
 * https://fab.cba.mit.edu/classes/864.17/people/copplestone/final_project/index.html
 * https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling
 * https://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/s2003/es89kh98/es89kh98/index.htm
 */

public class Pendulum {

    private LinearOpMode op;
    private HardwareMap hw;
    private DcMotorEx encoder;

    private long curFrameTime;
    private long prevFrameTime;
    private long delta;

    private double omega;
    private double prevOmega;
    private double alpha;

    private final int TICKS_PER_REV = 8192;

    public Pendulum(LinearOpMode op)
    {
        this.op = op;
        this.hw = op.hardwareMap;

        encoder = hw.get(DcMotorEx.class, "pendulum");

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void reset()
    {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update()
    {
        curFrameTime = SystemClock.elapsedRealtime();
        delta = curFrameTime - prevFrameTime;

        omega = calculateVelocity();
        alpha = (omega - prevOmega) / delta;

        prevFrameTime = curFrameTime;
        prevOmega = omega;
    }

    public int getPositionTicks()
    {
        return encoder.getCurrentPosition();
    }

    public int getVelocityTicks()
    {
        return encoder.getCurrentVelocity();
    }

    public double getTheta()
    {
        return (getPositionTicks() / TICKS_PER_REV) * 2 * Math.PI;
    }

    public double getOmega()
    {
        return (getVelocityTicks() / TICKS_PER_REV) * 2 * Math.PI;
    }

    public double getAlpha()
    {
        return (alpha / TICKS_PER_REV) * 2 * Math.PI;
    }
}