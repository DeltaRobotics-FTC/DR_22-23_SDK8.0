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
    public Servo Claw0 = null;
    public Servo Claw1 = null;

    int slidePosDown = 40;
    int slidePosJunction = 50;
    int slidePosLow = 375;
    int slidePosMid = 600;
    int slidePosTall = 835;
    int slidePosTarget;

    public double slidePower = 0;

    int slidePose0 =0;
    int slidePose1 =0;

    int setJoysticks = 0;

    boolean DpadLeftToggle = true;
    boolean DpadRightToggle = true;


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
        Claw0 = hardwareMap.servo.get("Claw0");
        Claw1 = hardwareMap.servo.get("Claw1");

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
            /*
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
            */
            if (gamepad1.a)
            {
                setJoysticks = 1;
            }
            else if (gamepad1.b)
            {
                setJoysticks = 2;
            }
            else if (gamepad1.x)
            {
                setJoysticks = 3;
            }

            if (setJoysticks == 1) {
                if (gamepad1.left_stick_button) {
                    motorRF.setPower(1 * (gamepad1.right_stick_y + gamepad1.left_stick_x * .1));
                    motorRB.setPower(1 * (-(gamepad1.right_stick_y + gamepad1.left_stick_x * .1)));
                    motorLB.setPower(1 * (gamepad1.right_stick_y - gamepad1.left_stick_x * .1));
                    motorLF.setPower(1 * (-(gamepad1.right_stick_y - gamepad1.left_stick_x * .1)));
                } else {
                    motorRF.setPower(.75 * (gamepad1.right_stick_y + gamepad1.left_stick_x * .5));
                    motorRB.setPower(.75 * (-(gamepad1.right_stick_y + gamepad1.left_stick_x * .5)));
                    motorLB.setPower(.75 * (gamepad1.right_stick_y - gamepad1.left_stick_x * .5));
                    motorLF.setPower(.75 * (-(gamepad1.right_stick_y - gamepad1.left_stick_x * .5)));
                }
            }

            else if (setJoysticks == 2) {
                if (gamepad1.right_stick_button) {
                    motorRF.setPower(1 * (gamepad1.right_stick_y + gamepad1.right_stick_x * .1));
                    motorRB.setPower(1 * (-(gamepad1.right_stick_y + gamepad1.right_stick_x * .1)));
                    motorLB.setPower(1 * (gamepad1.right_stick_y - gamepad1.right_stick_x * .1));
                    motorLF.setPower(1 * (-(gamepad1.right_stick_y - gamepad1.right_stick_x * .1)));
                } else {
                    motorRF.setPower(.75 * (gamepad1.right_stick_y + gamepad1.right_stick_x * .5));
                    motorRB.setPower(.75 * (-(gamepad1.right_stick_y + gamepad1.right_stick_x * .5)));
                    motorLB.setPower(.75 * (gamepad1.right_stick_y - gamepad1.right_stick_x * .5));
                    motorLF.setPower(.75 * (-(gamepad1.right_stick_y - gamepad1.right_stick_x * .5)));
                }
            }


            else {
                if (gamepad1.left_stick_button) {
                    motorRF.setPower(1 * (gamepad1.left_stick_y + gamepad1.left_stick_x * .1));
                    motorRB.setPower(1 * (-(gamepad1.left_stick_y + gamepad1.left_stick_x * .1)));
                    motorLB.setPower(1 * (gamepad1.left_stick_y - gamepad1.left_stick_x * .1));
                    motorLF.setPower(1 * (-(gamepad1.left_stick_y - gamepad1.left_stick_x * .1)));
                } else {
                    motorRF.setPower(.75 * (gamepad1.left_stick_y + gamepad1.left_stick_x * .5));
                    motorRB.setPower(.75 * (-(gamepad1.left_stick_y + gamepad1.left_stick_x * .5)));
                    motorLB.setPower(.75 * (gamepad1.left_stick_y - gamepad1.left_stick_x * .5));
                    motorLF.setPower(.75 * (-(gamepad1.left_stick_y - gamepad1.left_stick_x * .5)));
                }
            }

            if (gamepad1.left_bumper) {
                Claw0.setPosition(.5);
                Claw1.setPosition(.5);
            }
            else if (gamepad1.right_bumper){
                Claw0.setPosition(.3);
                Claw1.setPosition(.7);
            }

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
                    slidePose1 += 5;
                    slidePose0 += 5;
                }
                else if(gamepad2.dpad_down){
                    slidePose1 -= 5;
                    slidePose0 -= 5;
                }
            //}

/*
            if(gamepad2.dpad_left){
                slidePose1 += 1;
                slidePose0 -= 1;
            }
            else if(gamepad2.dpad_right){
                slidePose1 -= 1;
                slidePose0 += 1;
            }

 */

            if (gamepad2.dpad_left && DpadLeftToggle)
            {
                slidePose1 += 1;
                slidePose0 -= 1;

                DpadLeftToggle = false;
            }
            else if (!gamepad2.dpad_left && !DpadLeftToggle)
            {
                DpadLeftToggle = true;
            }

            if (gamepad2.dpad_right && DpadRightToggle)
            {
                slidePose1 -= 1;
                slidePose0 += 1;

                DpadRightToggle = false;
            }
            else if (!gamepad2.dpad_right && !DpadRightToggle)
            {
                DpadRightToggle = true;
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

*/





            if (gamepad2.a){
                slidePosTarget = 1; // down
            }
            else if (gamepad2.b){
                slidePosTarget = 2; // mid
            }
            else if (gamepad2.x){
                slidePosTarget = 5; // tall
            }
            else if (gamepad2.y){
                slidePosTarget = 4; // low
            }
            else if (gamepad2.right_bumper){
                slidePosTarget = 3; // ground junction
            }
            else{
                slidePosTarget = 0;
            }



            if(slidePosTarget == 1)
            {
                slidePose1 = slidePosDown;
                slidePose0 = slidePosDown;
            }

            else if(slidePosTarget == 2)
            {
                slidePose1 = slidePosMid;
                slidePose0 = slidePosMid;
            }

            else if(slidePosTarget == 3)
            {
                slidePose1 = slidePosJunction;
                slidePose0 = slidePosJunction;
            }

            else if(slidePosTarget == 4)
            {
                slidePose1 = slidePosLow;
                slidePose0 = slidePosLow;
            }

            else if(slidePosTarget == 5)
            {
                slidePose1 = slidePosTall;
                slidePose0 = slidePosTall;
            }
            else {

                if(Slide1.getCurrentPosition() > slidePose1)
                {
                    //while (Slide1.getCurrentPosition() > slidePosTall)
                    //{
                    Slide1.setTargetPosition(slidePose1);
                    Slide1.setPower(-0.75);
                    Slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //}
                }

                if(Slide1.getCurrentPosition() < slidePose1)
                {
                    //while (Slide1.getCurrentPosition() < slidePosTall)
                    //{
                    Slide1.setTargetPosition(slidePose1);
                    Slide1.setPower(.75);
                    Slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //}
                }

                if(Slide0.getCurrentPosition() > slidePose0)
                {
                    //while (Slide1.getCurrentPosition() > slidePosTall)
                    //{
                    Slide0.setTargetPosition(slidePose0);
                    Slide0.setPower(-0.75);
                    Slide0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //}
                }

                if(Slide0.getCurrentPosition() < slidePose0)
                {
                    //while (Slide1.getCurrentPosition() < slidePosTall)
                    //{
                    Slide0.setTargetPosition(slidePose0);
                    Slide0.setPower(.75);
                    Slide0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //}
                }
            }


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