package org.firstinspires.ftc.teamcode.components;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Cart {

    private LinearOpMode op;
    private HardwareMap hw;
    private DcMotorEx cartMotor;

    private double curFrameTime;
    private double prevFrameTime;
    private double delta;

    private double position;
    private double velocity;
    private double prevVelocity;
    private double acceleration;

    private double target;

    private PIDController pid;

    private final int MAXIMUM_VELOCITY = 100;
    private final double MIN_WORKING_THETA;
    private final double MAX_WORKING_THETA;

    public static double P;
    public static double I;
    public static double D;

    private TrackingState trackingState;

    public Cart(LinearOpMode op)
    {
        this.op = op;
        this.hw = op.hardwareMap;
        this.cartMotor = op;

        cart = hw.get(DcMotorEx.class, "cart");
        cart.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cart.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cart.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // set to follow vel?
        
        pid = new PIDController(P, I, D);
        pid.setOutputBounds(-MAXIMUM_VELOCITY, MAXIMUM_VELOCITY);
        pid.setInputBounds(MIN_WORKING_THETA, MAX_WORKING_THETA);

        trackingState = velocity;
    }

    public void setTarget(double target)
    {
        this.target = target;
    }

    public void setTrackingState(TrackingState newState)
    {
        this.trackingState = newState;
        if(newState == TrackingState.FOLLOW_POSITION)
        {
            cart.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else
        {
            cart.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void update()
    {
        curFrameTime = SystemClock.elapsedRealtime();
        delta = curFrameTime - prevFrameTime;

        velocity = calculateVelocity();
        acceleration = (velocity - prevVelocity) / delta;

        prevFrameTime = curFrameTime;
        prevVelocity = velocity;

        // TODO: control method
        switch(trackingState)
        {
            case FOLLOW_POSITION:
            {
                break;
            }

            case FOLLOW_VELOCITY:
            {
                break;
            }

            case FOLLOW_ACCELERATION:
            {
                break;
            }
        }
    }

    public double getPosition()
    {
        return cartMotor.getCurrentPosition();
    }
    
    private double calculateVelocity()
    {
        return cartMotor.getCurrentVelocity();
    }

    private double calculateAcceleration()
    {
        return acceleration;
    }
}

public enum TrackingState {
    FOLLOW_VELOCITY,
    FOLLOW_ACCELERATION,
    FOLLOW_POSITION
}