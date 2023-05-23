package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TrackTester extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        // + x is to the right
        // 0 deg is down
        // cart start pos in center
        // check max range

        waitForStart();

        while(opModeIsActive())
        {
            telemetry.addData("caption", "thing");
            telemetry.update();
        }
    }
}

