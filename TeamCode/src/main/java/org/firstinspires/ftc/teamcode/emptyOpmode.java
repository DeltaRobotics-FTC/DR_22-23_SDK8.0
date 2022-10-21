package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="emptyOpmode")
@Disabled

public class emptyOpmode extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {

        waitForStart();

        while (opModeIsActive())
        {

        }
    }
}
