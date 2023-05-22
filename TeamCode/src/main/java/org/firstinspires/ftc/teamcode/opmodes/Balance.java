package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.components.Pendulum;
import org.firstinspires.ftc.teamcode.components.Cart;

public class Balance extends LinearOpMode
{

    private final double CART_MASS = 1.0; // kg
    private final double PENDULUM_MASS = 1.0; // kg
    private final double COEFFICIENT_OF_FRICTION_FOR_CART = 0.1; // N*m / sec
    private final double LENGTH_TO_PENDULUM_COM = 0.3; // m
    private final double MOMENT_OF_INERTIA = 0.006; // kg * m ^ 2
    private final double ENERGY_LOSS_CONSTANT = 4;
    private final int REQUIRED_TANGENTIAL_VELOCITY = 2 * Math.sqrt(9.81 * LENGTH_TO_PENDULUM_COM);

    private double force; // N
    private int cartPos; // m
    private double theta; // rad (0 is down)

    public static double topThreshold = Math.toRadians(135);
    public static double bottomThreshold = -topThreshold;

    private State state = WAIT;

    @Override
    public void runOpMode()
    {
        Cart cart = new Cart(this);
        Pendulum pendulum = new Pendulum(this);

        double currentVelocity = 0;

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.b)
            {
                state = WAIT;
            }
            switch(state)
            {
                case WAIT:
                {
                    if(gamepad1.a)
                    {
                        state = INITIAL;
                    }
                    break;
                }

                case LEFT_INITIAL:
                {
                    // cart set follow accel
                    currentVelocity = LENGTH_TO_PENDULUM_COM * LENGTH_TO_PENDULUM_COM;
                    if(Math.abs(currentVelocity) == REQUIRED_TANGENTIAL_VELOCITY + ENERGY_LOSS_CONSTANT)
                    {
                        state = FLIP;
                    }
                    else if(Math.abs(currentVelocity) < 0.1)
                    {
                        state = RIGHT_INITIAL;
                    }
                    // TODO: SET CONSTANTS
                    else if(cart.getPosition() > 100 || cart.getPosition < 0)
                    {
                        state = RIGHT_INITIAL;
                    }
                    break;
                }
                
                case RIGHT_INITIAL:
                {
                    // cart set follow accel
                    currentVelocity = LENGTH_TO_PENDULUM_COM * LENGTH_TO_PENDULUM_COM;
                    if(Math.abs(currentVelocity) == REQUIRED_TANGENTIAL_VELOCITY + ENERGY_LOSS_CONSTANT)
                    {
                        state = FLIP;
                    }
                    else if(Math.abs(currentVelocity) < 0.1)
                    {
                        state = LEFT_INITIAL;
                    }
                    // TODO: SET CONSTANTS
                    else if(cart.getPosition() > 100 || cart.getPosition < 0)
                    {
                        state = LEFT_INITIAL;
                    }
                    break;
                }

                case BRAKE:
                {
                    if(pendulum.getTheta() > bottomThreshold && pendulum.getTheta() < topThreshold)
                    {
                        state = BALANCE;
                    }
                    break;
                }

                case FLIP:
                {
                    state = BRAKE;
                    break;
                }
                
                case BALANCE:
                {
                    break;
                }
            }
        }
    }
}

private enum State {
    WAIT,
    INITIAL,
    BRAKE,
    FLIP,
    BALANCE,
}