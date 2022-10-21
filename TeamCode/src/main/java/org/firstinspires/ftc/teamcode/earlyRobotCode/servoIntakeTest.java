package org.firstinspires.ftc.teamcode.earlyRobotCode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="servoIntakeTest")
@Disabled

public class servoIntakeTest extends LinearOpMode
{

    public CRServo intake0 = null;
    public CRServo intake1 = null;

    @Override
    public void runOpMode() throws InterruptedException
    {
        intake0 = hardwareMap.crservo.get("intake0");
        intake1 = hardwareMap.crservo.get("intake1");

        waitForStart();

        while (opModeIsActive())
        {

            intake0.setPower(gamepad1.left_stick_y);
            intake1.setPower(gamepad1.right_stick_y);

        }
    }
}
