package org.firstinspires.ftc.teamcode.steamboatWillie;


//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.RobotHardware;


@TeleOp(name="willieFirstTeleOp")
//@Disabled

public class willieFirstTeleOp extends LinearOpMode
{
    public DcMotor motorRF = null;
    public DcMotor motorLF = null;
    public DcMotor motorRB = null;
    public DcMotor motorLB = null;
    public DcMotor Slide0 = null;
    public DcMotor Slide1 = null;
    //public Servo Claw0 = null;
    //public Servo Claw1 = null;

    int slidePosDown = 0;
    int slidePosJunction = 50;
    int slidePosLow = 266;
    int slidePosMid = 533;
    int slidePosTall = 800;
    int slidePosTarget;

    public double slidePower = 0;

    int slidePose0 =0;
    int slidePose1 =0;


    @Override
    public void runOpMode() throws InterruptedException
    {
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        //RobotHardware robot = new RobotHardware(hardwareMap);

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //ElapsedTime whatever = new ElapsedTime();



        motorRF = hardwareMap.dcMotor.get("motorRF");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorLB = hardwareMap.dcMotor.get("motorLB");
        Slide0 = hardwareMap.dcMotor.get("Slide0");
        Slide1 = hardwareMap.dcMotor.get("Slide1");
        //Claw0 = hardwareMap.servo.get("Claw0");
        //Claw1 = hardwareMap.servo.get("Claw1");

        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRF.setDirection(DcMotorSimple.Direction.REVERSE);
        Slide0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Slide0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide0.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive())
        {
            //Code goes here
            if (gamepad1.a) {
                motorRF.setPower(.5 * (gamepad1.left_stick_y + gamepad1.left_stick_x));
                motorRB.setPower(.5 * (-(gamepad1.left_stick_y + gamepad1.left_stick_x)));
                motorLB.setPower(.5 * (gamepad1.left_stick_y - gamepad1.left_stick_x));
                motorLF.setPower(.5 * (-(gamepad1.left_stick_y - gamepad1.left_stick_x)));
            }

            else if (gamepad1.y) {
                motorRF.setPower(.1 * (gamepad1.left_stick_y + gamepad1.left_stick_x));
                motorRB.setPower(.1 * (-(gamepad1.left_stick_y + gamepad1.left_stick_x)));
                motorLB.setPower(.1 * (gamepad1.left_stick_y - gamepad1.left_stick_x));
                motorLF.setPower(.1 * (-(gamepad1.left_stick_y - gamepad1.left_stick_x)));
            }

            else {
                motorRF.setPower(1 * (gamepad1.left_stick_y + gamepad1.left_stick_x));
                motorRB.setPower(1 * (-(gamepad1.left_stick_y + gamepad1.left_stick_x)));
                motorLB.setPower(1 * (gamepad1.left_stick_y - gamepad1.left_stick_x));
                motorLF.setPower(1 * (-(gamepad1.left_stick_y - gamepad1.left_stick_x)));
            }

            //if (gamepad1.left_bumper) {
            //    Claw0.setPosition(.6);
            //    Claw1.setPosition(.4);
            //}
            //else if (gamepad1.right_bumper){
            //    Claw0.setPosition(.3);
            //    Claw1.setPosition(.7);
            //}

/*
            if ((Slide1.getCurrentPosition()+Slide0.getCurrentPosition())/2 > 0){
                if(gamepad2.dpad_up){
                    slidePose1 -= 12;
                    slidePose0 -= 12;
                }
            }

            else if ((Slide1.getCurrentPosition()+Slide0.getCurrentPosition())/2 < 4300){
                if(gamepad2.dpad_down){
                    slidePose1 += 12;
                    slidePose0 += 12;
                }
            }
            */
            //else {
                if(gamepad2.dpad_up){
                    slidePose1 -= 12;
                    slidePose0 -= 12;
                }
                else if(gamepad2.dpad_down){
                    slidePose1 += 12;
                    slidePose0 += 12;
                }
            //}


            if(gamepad2.dpad_left){
                slidePose1 += 1;
                slidePose0 -= 1;
            }
            else if(gamepad2.dpad_right){
                slidePose1 -= 1;
                slidePose0 += 1;
            }

            /*
            if(gamepad2.a)
            {
                slidePose1 = -4300;
                slidePose0 = -4300;
            }

            if(gamepad2.b)
            {
                slidePose1 = 0;
                slidePose0 = 0;
            }

            if(gamepad2.x)
            {
                slidePose1 = -4000;
                slidePose0 = -3600;
            }

            if(gamepad2.y)
            {
                slidePose1 = -2300;
                slidePose0 = -2300;
            }
            */



            Slide1.setTargetPosition(slidePose1);
            Slide1.setPower(1);
            Slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide0.setTargetPosition(slidePose0);
            Slide0.setPower(1);
            Slide0.setMode(DcMotor.RunMode.RUN_TO_POSITION);



/*
            slidePower = gamepad2.left_stick_y ;


            if (Slide1.getCurrentPosition() < 0 && slidePower < 0){
                Slide1.setPower(0);
                Slide0.setPower(0);
            }

            else if (Slide1.getCurrentPosition() > 800 && slidePower > 0){
                Slide1.setPower(0);
                Slide0.setPower(0);
            }
            else {

                Slide1.setPower(slidePower);
                Slide0.setPower(slidePower);
                Slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Slide0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           }






/*
            if (gamepad2.a){
                slidePosTarget = 1;
            }
            else if (gamepad2.b){
                slidePosTarget = 2;
            }
            else if (gamepad2.x){
                slidePosTarget = 3;
            }
            else if (gamepad2.y){
                slidePosTarget = 4;
            }
            else if (gamepad2.right_bumper){
                slidePosTarget = 5;
            }
            else{
                slidePosTarget = 0;
            }

            if(slidePosTarget == 1)
            {
                if(Slide1.getCurrentPosition() > slidePosDown)
                {
                    while (Slide1.getCurrentPosition() > slidePosDown)
                    {
                        Slide1.setPower(-.5);
                        Slide0.setPower(-.5);
                        Slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Slide0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                }

                if(Slide1.getCurrentPosition() < slidePosDown)
                {
                    while (Slide1.getCurrentPosition() < slidePosDown)
                    {
                        Slide1.setPower(0.25);
                        Slide0.setPower(0.25);
                        Slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Slide0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                }
            }

            if(slidePosTarget == 2)
            {
                if(Slide1.getCurrentPosition() > slidePosMid)
                {
                    while (Slide1.getCurrentPosition() > slidePosMid)
                    {
                        Slide1.setPower(-.5);
                        Slide0.setPower(-.5);
                        Slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Slide0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                }

                if(Slide1.getCurrentPosition() < slidePosMid)
                {
                    while (Slide1.getCurrentPosition() < slidePosMid)
                    {
                        Slide1.setPower(.25);
                        Slide0.setPower(.25);
                        Slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Slide0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                }
            }

            if(slidePosTarget == 3)
            {
                if(Slide1.getCurrentPosition() > slidePosJunction)
                {
                    while (Slide1.getCurrentPosition() > slidePosJunction)
                    {
                        Slide1.setPower(-.5);
                        Slide0.setPower(-.5);
                        Slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Slide0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                }

                if(Slide1.getCurrentPosition() < slidePosJunction)
                {
                    while (Slide1.getCurrentPosition() < slidePosJunction)
                    {
                        Slide1.setPower(.25);
                        Slide0.setPower(.25);
                        Slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Slide0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                }
            }

            if(slidePosTarget == 4)
            {
                if(Slide1.getCurrentPosition() > slidePosLow)
                {
                    while (Slide1.getCurrentPosition() > slidePosLow)
                    {
                        Slide1.setPower(-.5);
                        Slide0.setPower(-.5);
                        Slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Slide0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                }

                if(Slide1.getCurrentPosition() < slidePosLow)
                {
                    while (Slide1.getCurrentPosition() < slidePosLow)
                    {
                        Slide1.setPower(.25);
                        Slide0.setPower(.25);
                        Slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Slide0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                }
            }
*/

            /*
             if(gamepad2.dpad_up){
                 Slide1.setPower(-0.5);
                 Slide0.setPower(0.5);
             }
             if(gamepad2.dpad_down){
                 Slide1.setPower(0.5);
                 Slide0.setPower(-0.5);
             }

             */


            telemetry.addData("Slide0Encoder",Slide0.getCurrentPosition());
            telemetry.addData("Slide1Encoder",Slide1.getCurrentPosition());
            telemetry.addData("Slide0Power",Slide0.getPower());
            telemetry.addData("Slide1Power",Slide1.getPower());

            telemetry.update();

            //Slide0.setTargetPosition(20000);
            //Slide1.setTargetPosition(20000);
            //Slide0.setPower(.7);
            //Slide1.setPower(.7);
            //Slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //Slide0.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
    }
/*
    void forward (int distance, double power) {
        RobotHardware robot = new RobotHardware(hardwareMap);

        robot.motorLB.setTargetPosition(distance);

        robot.motorLB.setPower(power);

        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }*/
}