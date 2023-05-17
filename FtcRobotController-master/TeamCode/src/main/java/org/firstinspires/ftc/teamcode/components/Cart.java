package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Cart {
    private LinearOpMode op;
    private HardwareMap hw;
    private DcMotor cartMotor;

    private double position;
    private double velocity;
    private double acceleration;

    public Cart(LinearOpMode op)
    {
        this.op = op;
        this.hw = op.hardwareMap;
        this.cartMotor = op;

        cart = hw.get(DcMotor.class, "cart");
        cart.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cart.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        cart.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double getPosition()
    {
        return cartMotor.getCurrentPosition();
    }
    
    private double calculateVelocity()
    {
        return cartMotor.getCurrentVelocity();
    }

    private double calculateAcceleration()
    {
        return 0;
    }

    
}