package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Cart;
import org.firstinspires.ftc.teamcode.util.TrackingState;

@TeleOp
public class TrackTester extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        Cart cart = new Cart(this);
        // + x is to the right
        // 0 deg is down
        // cart start pos in center
        // check max range

        waitForStart();

        cart.setTarget(TrackingState.FOLLOW_POSITION, 100);

        while(cart.getPosition() < 100)
        {
            cart.update();
        }

        cart.setTarget(TrackingState.FOLLOW_VELOCITY, -100);

        while(Math.abs(cart.calculateVelocity()) < 100)
        {
            cart.update();
        }

        cart.setTarget(TrackingState.FOLLOW_POSITION, 0);

        while(cart.getPosition() < 0)
        {
            cart.update();
        }

        cart.setTarget(TrackingState.FOLLOW_VELOCITY, 100);

        while(Math.abs(cart.calculateVelocity()) < 100)
        {
            cart.update();
        }

        cart.setTarget(TrackingState.FOLLOW_ACCELERATION, -10);

        while(Math.abs(cart.calculateVelocity()) > 0)
        {
            cart.update();
        }

        cart.setTarget(TrackingState.FOLLOW_POSITION, 0);

        while(cart.getPosition() > 0)
        {
            cart.update();
        }
    }
}

