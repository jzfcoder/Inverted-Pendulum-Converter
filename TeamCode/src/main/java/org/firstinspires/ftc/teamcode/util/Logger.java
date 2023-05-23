package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class Logger implements Runnable {

    private LinearOpMode op;
    private Telemetry telemetry;

    private ArrayList<String[]> feed = new ArrayList<>();

    public Logger(LinearOpMode op)
    {
        this.op = op;
        this.telemetry = op.telemetry;
    }

    public void addData(String[] input)
    {
        if(input.length != 2)
        {
            return;
        }
        feed.add(input);
    }

    @Override
    public void run()
    {
        for(String[] n : feed)
        {
            telemetry.addData(n[0], n[1]);
        }
    }
}
