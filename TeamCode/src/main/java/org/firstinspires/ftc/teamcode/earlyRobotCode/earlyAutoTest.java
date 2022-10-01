package org.firstinspires.ftc.teamcode.earlyRobotCode;
        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
        import org.firstinspires.ftc.teamcode.RobotHardware;
        import org.firstinspires.ftc.teamcode.pinkCode.ContourPipeline;
        import org.opencv.core.Scalar;
        import org.openftc.easyopencv.OpenCvCamera;
        import org.openftc.easyopencv.OpenCvCameraFactory;
        import org.openftc.easyopencv.OpenCvCameraRotation;

        import java.sql.Time;
        import java.lang.Math;


@Autonomous(name="earlyAutoTest")
//@Disabled

public class earlyAutoTest extends LinearOpMode
{
    BNO055IMU imu;
    Orientation angles;
    double driveTimeVar;

    @Override
    public void runOpMode() throws InterruptedException
    {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        while (!isStarted())
        {
            // camera code goes here
        }

        //ElapsedTime driveTime = new ElapsedTime();

        waitForStart();
        //drive code here
        //drive(1, 1, 1000);
        //sleep(1000);
        betterPivot(-90);
        //sleep(1000);
        //encoderDriveForward(1000, 1);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        while(true){

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("current angle" , angles.firstAngle);
            telemetry.update();
        }
    }


    public void betterPivot(int angle)
    {
        RobotHardware robot = new RobotHardware(hardwareMap);
        ElapsedTime PIDLoopTime = new ElapsedTime();
        double OldPIDLoopTime = 0;
        double ARL = 0;//angle rate limited
        double rateTime = 2;//time to make the turn
        double rate = angle / rateTime;
        double deltaT = 0;//time it takes to make one loop dt
        double turnpower;
        double angleError;
        double Kp = 0.1;//proportional gain


        double I = 0;
        int J = 0;


        while (angle > 180)
        {
            angle -= 360;
        }
        while (angle < -180)
        {
            angle += 360;
        }

        while (I == 0)
        {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            deltaT = PIDLoopTime.nanoseconds() - OldPIDLoopTime;

            //PID math
            //rate limit
            if(Math.abs(ARL - angle) < rate * deltaT) {
                ARL = angle;
            } else if ((angle - ARL) > 0) {
                ARL = ARL + (rate * deltaT);
            } else {
                ARL = ARL - (rate * deltaT);
            }

            angleError = ARL - angles.firstAngle;

            turnpower = angleError * Kp;

            OldPIDLoopTime = PIDLoopTime.nanoseconds();


            robot.motorRF.setPower(turnpower);
            robot.motorRB.setPower(turnpower);
            robot.motorLB.setPower(-turnpower);
            robot.motorLF.setPower(-turnpower);

            telemetry.addData("Left", I);
            telemetry.addData("current angle" , angles.firstAngle);
            telemetry.addData("target angle" , angle);
            telemetry.addData("J val", J);
            telemetry.addData("angle error", angleError);
            telemetry.addData("turn power", turnpower);

            telemetry.update();




            if(angles.firstAngle > angle - 2 && angles.firstAngle < angle + 2)
            {
                J++;

                telemetry.addData("Dead", I);
                telemetry.addData("current angle" ,angles.firstAngle);
                telemetry.addData("target angle" , angle);
                telemetry.addData("J val", J);
                telemetry.addData("angle error", angleError);
                telemetry.addData("turn power", turnpower);
                telemetry.update();
            }
            else
            {
                J = 0;
            }

            if (J == 10)
            {
                robot.motorRF.setPower(0);
                robot.motorRB.setPower(0);
                robot.motorLB.setPower(0);
                robot.motorLF.setPower(0);

                I = 1.5;

                telemetry.addData("Dead", I);
                telemetry.addData("current angle" ,angles.firstAngle);
                telemetry.addData("target angle" , angle);
                telemetry.addData("J val", J);
                telemetry.addData("angle error", angleError);
                telemetry.addData("turn power", turnpower);
                telemetry.update();
            }


        }


    }


    public void  betterDrive(int angle, double PowerX, double PowerY, double speed, double time)
    {
        RobotHardware robot = new RobotHardware(hardwareMap);
        ElapsedTime driveTime = new ElapsedTime();

        driveTimeVar = driveTime.milliseconds();

        while ((driveTimeVar + time) > driveTime.milliseconds())
        {

            double turnPower;

            while (angle > 180)
            {
                angle -= 360;
            }
            while (angle < -180)
            {
                angle += 360;
            }


            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            turnPower = ((angle - angles.firstAngle) / angle) + .2;

            robot.motorRF.setPower(speed * ((PowerY - PowerX) - (turnPower)));
            robot.motorRB.setPower(speed * (-(-PowerX - PowerY) - (turnPower)));
            robot.motorLB.setPower(speed * ((PowerY - PowerX) - (turnPower)));
            robot.motorLF.setPower(speed * ((PowerX + PowerY)) - (turnPower));

            telemetry.addData("motorRF Power", robot.motorRF.getPower());
            telemetry.addData("motorRB Power", robot.motorRB.getPower());
            telemetry.addData("motorLB Power", robot.motorLB.getPower());
            telemetry.addData("motorLF Power", robot.motorLF.getPower());
            telemetry.update();
        }

        stop(1);
    }

    public void  drive(double Power, double speed, double time)
    {
        RobotHardware robot = new RobotHardware(hardwareMap);
        ElapsedTime driveTime = new ElapsedTime();

        driveTimeVar = driveTime.milliseconds();

        while ((driveTimeVar + time) > driveTime.milliseconds())
        {
            robot.motorRF.setPower(speed * Power);
            robot.motorRB.setPower(speed * Power);
            robot.motorLB.setPower(speed * Power);
            robot.motorLF.setPower(speed * Power);

            telemetry.addData("motorRF Power", robot.motorRF.getPower());
            telemetry.addData("motorRB Power", robot.motorRB.getPower());
            telemetry.addData("motorLB Power", robot.motorLB.getPower());
            telemetry.addData("motorLF Power", robot.motorLF.getPower());
            telemetry.addData("time", driveTime.milliseconds());
            telemetry.update();
        }

        stop(1);
    }



    public void stop(int hi)
    {
        RobotHardware robot = new RobotHardware(hardwareMap);

        robot.motorRF.setPower(0);
        robot.motorRB.setPower(0);
        robot.motorLB.setPower(0);
        robot.motorLF.setPower(0);
    }



    void encoderDriveForward (int distance, double power) {

        RobotHardware robot = new RobotHardware(hardwareMap);

        robot.motorRF.setTargetPosition(distance + robot.motorRF.getCurrentPosition());
        robot.motorRB.setTargetPosition(distance + robot.motorRB.getCurrentPosition());
        robot.motorLF.setTargetPosition(distance + robot.motorLF.getCurrentPosition());
        robot.motorLB.setTargetPosition(distance + robot.motorLB.getCurrentPosition());

        robot.motorRF.setPower(power);
        robot.motorRB.setPower(power);
        robot.motorLB.setPower(power);
        robot.motorLF.setPower(power);

        robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }



}
