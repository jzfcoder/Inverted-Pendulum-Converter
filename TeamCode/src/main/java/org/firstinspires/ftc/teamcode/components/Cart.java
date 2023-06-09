package org.firstinspires.ftc.teamcode.components;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.TrackingState;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@SuppressWarnings({"FieldCanBeLocal", "FieldMayBeFinal"})
public class Cart {

    private HardwareMap hw;
    private DcMotorEx cart;

    public static int maxVelocity = 3000;
    public static int minVelocity = -maxVelocity;

    private double curFrameTime;
    private double prevFrameTime;
    private double delta;

    private double velocity;
    private double prevVelocity;
    private double acceleration;

    private double targetPosition;
    private double targetVelocity;
    private double targetAcceleration;

    private TrackingState trackingState;

    private static Logger logger;

    public Cart(LinearOpMode op, Logger logger)
    {
        this.hw = op.hardwareMap;

        cart = hw.get(DcMotorEx.class, "cart");
        cart.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        cart.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        cart.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // set to follow vel?
        cart.setDirection(DcMotorEx.Direction.REVERSE);
        
        trackingState = TrackingState.FOLLOW_VELOCITY;
        this.logger = logger;
    }

    public Cart(LinearOpMode op)
    {
        this.hw = op.hardwareMap;

        cart = hw.get(DcMotorEx.class, "cart");
        cart.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        cart.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        cart.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // set to follow vel?
        cart.setDirection(DcMotorEx.Direction.REVERSE);

        trackingState = TrackingState.FOLLOW_VELOCITY;
    }

    public void setTarget(TrackingState newState, double target)
    {
        this.trackingState = newState;
        switch(trackingState) {
            case FOLLOW_POSITION:
                {
                    targetPosition = target;
                    cart.setTargetPosition((int) targetPosition);
                    cart.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                }
                break;

            case FOLLOW_VELOCITY:
                {
                    cart.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    targetVelocity = target;
                }
                break;

            case FOLLOW_ACCELERATION:
                {
                    cart.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    targetAcceleration = target;
                    targetVelocity = calculateVelocity();
                }
                break;
        }

    }

    public void updateTargetVelocity(double vel)
    {
        this.targetVelocity = vel;
    }

    public void updateTargetAcceleration(double a)
    {
        this.targetAcceleration = a;
    }

    public void zero()
    {
        DcMotor.RunMode mode = cart.getMode();
        cart.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cart.setMode(mode);
    }

    public void update()
    {
        if(cart.getCurrentPosition() > 17 && (targetPosition > 17 || targetVelocity > 0))
        {
            return;
        }
        if(cart.getCurrentPosition() < -17 && (targetPosition < 17 || targetVelocity < 0))
        {
            return;
        }

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
                cart.setTargetPosition((int) targetPosition);
                cart.setPower(1);
                break;
            }

            case FOLLOW_VELOCITY:
            {
                cart.setPower(targetVelocity);
                break;
            }

            case FOLLOW_ACCELERATION:
            {
                cart.setPower(targetVelocity < 0 ? Math.max(targetVelocity, minVelocity) : Math.min(targetVelocity, maxVelocity));
                targetVelocity = targetVelocity + (delta * targetAcceleration);
                break;
            }
        }
    }

    public TrackingState getTrackingState() { return trackingState; }

    public double calculateAcceleration() { return acceleration; }

    public double getPosition()
    {
        return cart.getCurrentPosition();
    }
    
    public double calculateVelocity()
    {
        return cart.getVelocity();
    }
}