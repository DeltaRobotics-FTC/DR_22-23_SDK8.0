package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="mecanumDrive")

public class mecanumDrive extends LinearOpMode{


        public DcMotor motorRF = null;
        public DcMotor motorLF = null;
        public DcMotor motorRB = null;
        public DcMotor motorLB = null;

        double speed = 1;
        double zScale = 1;


        public void runOpMode() throws InterruptedException {


            motorRF = hardwareMap.dcMotor.get("motorRF");
            motorLF = hardwareMap.dcMotor.get("motorLF");
            motorRB = hardwareMap.dcMotor.get("motorRB");
            motorLB = hardwareMap.dcMotor.get("motorLB");

            motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            waitForStart();

            while (opModeIsActive()) {

                motorRF.setPower(speed*((-gamepad1.right_stick_y - gamepad1.right_stick_x) - (zScale * -gamepad1.left_stick_x)));
                motorRB.setPower(speed*(-(-gamepad1.right_stick_x + gamepad1.right_stick_y) - (zScale * -gamepad1.left_stick_x)));
                motorLB.setPower(speed*((gamepad1.right_stick_y + gamepad1.right_stick_x) - (zScale * -gamepad1.left_stick_x)));
                motorLF.setPower(speed*((-gamepad1.right_stick_x + gamepad1.right_stick_y)) - (zScale * -gamepad1.left_stick_x));

                telemetry.addData("motorRFPower", motorRF.getPower());
                telemetry.addData("motorRBPower", motorRB.getPower());
                telemetry.addData("motorLBPower", motorLB.getPower());
                telemetry.addData("motorLFPower", motorLF.getPower());

                telemetry.update();

            }
        }



}
