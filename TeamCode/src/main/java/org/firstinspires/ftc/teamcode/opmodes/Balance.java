package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.util.State.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Pendulum;
import org.firstinspires.ftc.teamcode.components.Cart;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.TrackingState;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.State;

@SuppressWarnings({"FieldCanBeLocal"})
@Config
@TeleOp
public class Balance extends LinearOpMode
{

    private final double LENGTH_TO_PENDULUM_COM = 0.3; // m
    private final double ENERGY_LOSS_CONSTANT = 4;
    private final double REQUIRED_TANGENTIAL_VELOCITY = 2 * Math.sqrt(9.81 * LENGTH_TO_PENDULUM_COM);

    public static double topThreshold = 180;
    public static double bottomThreshold = -topThreshold;

    public static double maxVel = 0.75;
    public static double minVel = -0.75 ;

    // TODO:
    public static double farRight = 17;
    public static double farLeft = -farRight;

    // TODO:
    public static double P = 0.005;
    public static double I = 0;
    public static double D = 0;

    public static boolean balance = false;

    private State state = BALANCE;
    private PIDController pid;
    private Telemetry dashTelemetry;

    @Override
    public void runOpMode()
    {
        dashTelemetry = FtcDashboard.getInstance().getTelemetry();
        Logger logger = new Logger(this);
        Cart cart = new Cart(this, logger);
        Pendulum pendulum = new Pendulum(this);

        pid = new PIDController(P, I, D);
        pid.setInputBounds(true, bottomThreshold, topThreshold);
        pid.setOutputBounds(minVel, maxVel);
        pid.setSetPoint(0);

        double currentVelocity;
        int cartSwitch;

        waitForStart();

        cart.setTarget(TrackingState.FOLLOW_VELOCITY, 0);
//        cart.setTarget(TrackingState.FOLLOW_ACCELERATION, 1);
        cartSwitch = (int) (cart.getPosition()) + (int) ((farLeft - (int) cart.getPosition()) / 2);

        while(opModeIsActive())
        {
            dashTelemetry.addData("mode", cart.getTrackingState());
            dashTelemetry.addData("current vel", cart.calculateVelocity());
            dashTelemetry.addData("target vel", -pid.calculateCorrection(Math.toDegrees(pendulum.getTheta())));
            dashTelemetry.addData("pendulum raw", pendulum.getTheta());
            dashTelemetry.addData("pendulum", Math.toDegrees(pendulum.getTheta()));

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
                    dashTelemetry.update();
                    double angle = Math.toDegrees(pendulum.getTheta());
                    if((angle > 135 || angle < -135) && balance)
                    {
                        cart.updateTargetVelocity(-pid.calculateCorrection(angle));
                    }
                    else
                    {
                        cart.updateTargetVelocity(0);
                    }
                    break;
                }
            }
        }
    }
}

