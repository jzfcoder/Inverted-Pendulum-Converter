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
    private double force; // N
    private int cartPos; // m
    private double theta; // rad (0 is down)

    @Override
    public void runOpMode()
    {

        waitForStart();
    }
}