package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.util.State.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Pendulum;
import org.firstinspires.ftc.teamcode.components.Cart;
import org.firstinspires.ftc.teamcode.util.TrackingState;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.State;

@SuppressWarnings({"FieldCanBeLocal"})
@TeleOp
public class Balance extends LinearOpMode
{

    private final double LENGTH_TO_PENDULUM_COM = 0.3; // m
    private final double ENERGY_LOSS_CONSTANT = 4;
    private final double REQUIRED_TANGENTIAL_VELOCITY = 2 * Math.sqrt(9.81 * LENGTH_TO_PENDULUM_COM);

    public static double topThreshold = Math.toRadians(135);
    public static double bottomThreshold = -topThreshold;

    public static double maxVel = 1;
    public static double minVel = -1;

    // TODO:
    public static double farRight = 100;
    public static double farLeft = -farRight;

    // TODO:
    public static double P = 0.08;
    public static double I = 0.01;
    public static double D = 0;

    private State state = LEFT_INITIAL;
    private PIDController pid;

    @Override
    public void runOpMode()
    {
        Cart cart = new Cart(this);
        Pendulum pendulum = new Pendulum(this);

        pid = new PIDController(P, I, D);
        pid.setInputBounds(false, bottomThreshold, topThreshold);
        pid.setOutputBounds(minVel, maxVel);

        double currentVelocity;
        int cartSwitch;

        waitForStart();

        cart.setTarget(TrackingState.FOLLOW_ACCELERATION, 1);
        cartSwitch = (int) (cart.getPosition()) + (int) ((farLeft - (int) cart.getPosition()) / 2);

        while(opModeIsActive())
        {
            cart.update();
            pendulum.update();
            switch(state)
            {
                case LEFT_INITIAL:
                {
                    // cart set follow accel
                    if(cart.getPosition() < cartSwitch)
                    {
                        cart.updateTargetAcceleration(1);
                    }

                    currentVelocity = cart.calculateVelocity() * LENGTH_TO_PENDULUM_COM;
                    if(Math.abs(currentVelocity) == REQUIRED_TANGENTIAL_VELOCITY + ENERGY_LOSS_CONSTANT)
                    {
                        state = FLIP;
                    }
                    else if(cart.getPosition() > farRight || cart.getPosition() < farLeft || Math.abs(currentVelocity) < 0.1)
                    {
                        cart.setTarget(TrackingState.FOLLOW_ACCELERATION, 1);
                        cartSwitch = (int) (cart.getPosition()) + (int) ((farRight - (int) cart.getPosition()) / 2);
                        state = RIGHT_INITIAL;
                    }
                    break;
                }
                
                case RIGHT_INITIAL:
                {
                    // cart set follow accel
                    if(cart.getPosition() > cartSwitch)
                    {
                        cart.updateTargetAcceleration(-1);
                    }

                    currentVelocity = cart.calculateVelocity() * LENGTH_TO_PENDULUM_COM;
                    if(Math.abs(currentVelocity) == REQUIRED_TANGENTIAL_VELOCITY + ENERGY_LOSS_CONSTANT)
                    {
                        state = FLIP;
                    }
                    else if(cart.getPosition() > farRight || cart.getPosition() < farLeft || Math.abs(currentVelocity) < 0.1)
                    {
                        cart.setTarget(TrackingState.FOLLOW_ACCELERATION, 1);
                        cartSwitch = (int) (cart.getPosition()) + (int) ((farLeft - (int) cart.getPosition()) / 2);
                        state = LEFT_INITIAL;
                    }
                    break;
                }

                case BRAKE:
                {
                    cart.setTarget(TrackingState.FOLLOW_POSITION, cart.getPosition());
                    if(pendulum.getTheta() > bottomThreshold && pendulum.getTheta() < topThreshold)
                    {
                        state = BALANCE;
                        cart.setTarget(TrackingState.FOLLOW_VELOCITY, 0);
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
                    cart.updateTargetVelocity(pid.calculateCorrection(Math.toRadians(180) - pendulum.getTheta()));
                    break;
                }
            }
        }
    }
}

