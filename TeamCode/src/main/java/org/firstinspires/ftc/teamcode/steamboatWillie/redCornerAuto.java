package org.firstinspires.ftc.teamcode.steamboatWillie;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name="redCornerAuto")
//@Disabled

public class redCornerAuto extends LinearOpMode
{
    BNO055IMU imu;
    Orientation angles;
    double driveTimeVar;

    public DcMotor motorRF = null;
    public DcMotor motorLF = null;
    public DcMotor motorRB = null;
    public DcMotor motorLB = null;
    //public DcMotor Slide0 = null;
    //public DcMotor Slide1 = null;
    //public Servo Claw0 = null;
    //public Servo Claw1 = null;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    int slidePose0 =0;
    int slidePose1 =0;

    //for the programmable
    List<String> junctionList = new ArrayList<String>();

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //Slide0 = hardwareMap.dcMotor.get("Slide0");
        //Slide1 = hardwareMap.dcMotor.get("Slide1");
        //Claw0 = hardwareMap.servo.get("Claw0");
        //Claw1 = hardwareMap.servo.get("Claw1");

        //Slide0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
        //Slide0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
        //Slide0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Slide0.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);

            if(gamepad1.a){
                junctionList.add("junction_1()");
            }
            else if(gamepad1.b) {
                junctionList.add("junction_3()");
            }
            else if(gamepad1.x){
                junctionList.add("junction_7()");
            }
            else if(gamepad1.y){
                junctionList.add("junction_9()");
            }
            else if(gamepad1.dpad_up){
                junctionList.add("junction_2()");
            }
            else if(gamepad1.dpad_right){
                junctionList.add("junction_6()");
            }
            else if(gamepad1.dpad_down){
                junctionList.add("junction_8()");
            }
            else if(gamepad1.dpad_left){
                junctionList.add("junction_4()");
            }
            else if(gamepad1.left_bumper){
                junctionList.add("junction_5()");
            }
            else if(gamepad1.back){
                junctionList.remove(junctionList.size() - 1);

            }
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        while(isStopRequested()){

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("current angle" , angles.firstAngle);
            telemetry.update();
        }

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            //trajectory 1 dot
            autoStart();
            programmable();
            //don't move for parking
        }
        else if(tagOfInterest.id == MIDDLE){
            //trajectory 2 dots
            autoStart();
            programmable();
            //parking
            encoderDriveForward(1400,-1);
        }
        else{
            //trajectory 3 dots
            autoStart();
            programmable();
            //parking
            encoderDriveForward(2300,-1);
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        String[] tempArray = new String[junctionList.size()];
        junctionList.toArray(tempArray);
        for(int i = 0; i<junctionList.size(); i++){
            telemetry.addLine("junctions" + tempArray[i]);
        }
    }

    public void liftSlides(int slidePos1, int slidePos0)
    {
        //Slide1.setTargetPosition(slidePos1);
        //Slide1.setPower(1);
        //Slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Slide0.setTargetPosition(slidePos0);
        //Slide0.setPower(1);
        //Slide0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void autoStart()
    {
        drive(0.5,1,500);
        drive(-0.5,1,100);
        //Claw0.setPosition(.6);
        //Claw1.setPosition(.4);
        sleep(500);
        slidePose1 = -200;
        slidePose0 = -200;
        liftSlides(slidePose1, slidePose0);
        sleep(1000);

        // one encoder tick = 0.02426 inches
        // desired distance in inches / 0.02426 inches = encoder ticks needed
        encoderDriveForward(-2000, -0.75);
        sleep(2000);

        betterPivot(45);

        slidePose1 = -4200;
        slidePose0 = -4200;

        liftSlides(slidePose1, slidePose0);
        sleep(2000);

        encoderDriveForward(375, 0.5);
        sleep(2000);

        //Claw0.setPosition(.3);
        //Claw1.setPosition(.7);
        sleep(500);

        encoderDriveForward(-375, -0.5);
        sleep(2000);

        slidePose1 = -100;
        slidePose0 = -100;

        liftSlides(slidePose1, slidePose0);
        sleep(1000);

        betterPivot(-90);
        sleep(2000);

        autoSetPositions(0);

        encoderDriveForward(900, 0.75);
        sleep(2000);

    }

    public void betterPivot(double angle)
    {
        RobotHardware robot = new RobotHardware(hardwareMap);
        double turnpower;
        double angleError = 0;
        double Kp = 0.0125;//proportional gain
        double reset = 0;
        double Ki = 0.0001;
        double Kd = 0.01;


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

        if (angle > 179 || angle < -179)
        {
            angle = 179;
        }

        while (I == 0)
        {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            //PID math
            if (angles.firstAngle >= 0 && angle > 0) {
                angleError = Math.abs(angle) - Math.abs(angles.firstAngle);
                //reset = reset + (Ki * angleError);
                turnpower = (angleError * Kp);// + reset + (Kd * (angleError - oldAngleError));
            }
            else if(angles.firstAngle <= 0 && angle > 0){
                angleError = Math.abs(angle) - Math.abs(angles.firstAngle);
                turnpower = (angleError * Kp);
            }
            else {
                angleError = -Math.abs(angle) + Math.abs(angles.firstAngle);
                //reset = reset - (Ki * angleError);
                turnpower = (angleError * Kp);// + reset + (Kd * (angleError - oldAngleError));
            }

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


            if(angles.firstAngle > angle - 1 && angles.firstAngle < angle + 1)
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

    void encoderDriveForward (int distance, double power)
    {

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

    public void autoSetPositions(int loopOrder)
    {
        if(loopOrder == 0){
            slidePose1 = -2000;
            slidePose0 = -2000;
            liftSlides(slidePose1, slidePose0);

        }else if(loopOrder == 1){
            slidePose1 = -1500;
            slidePose0 = -1500;
            liftSlides(slidePose1, slidePose0);

        }else if(loopOrder == 2){
            slidePose1 = -1000;
            slidePose0 = -1000;
            liftSlides(slidePose1, slidePose0);

        }else if(loopOrder == 3){
            slidePose1 = -500;
            slidePose0 = -500;
            liftSlides(slidePose1, slidePose0);

        } else{
            slidePose1 = -50;
            slidePose0 = -50;
            liftSlides(slidePose1, slidePose0);
        }
    }

    public void runjuctionFunction(int i)
    {
        String[] tempArray = new String[junctionList.size()];
        junctionList.toArray(tempArray);
        if(tempArray[i - 1] == "junction_1"){
            junction_1();
        } else if(tempArray[i - 1] == "junction_2"){
            junction_2();
        } else if(tempArray[i - 1] == "junction_3"){
            junction_3();
        } else if(tempArray[i - 1] == "junction_4"){
            junction_4();
        } else if(tempArray[i - 1] == "junction_5"){
            junction_5();
        } else if(tempArray[i - 1] == "junction_6"){
            junction_6();
        } else if(tempArray[i - 1] == "junction_7"){
            junction_7();
        } else if(tempArray[i - 1] == "junction_8"){
            junction_8();
        } else if(tempArray[i - 1] == "junction_9"){
            junction_9();
        }
    }

    public void programmable()
    {
        int iteration = 1;
        for(;iteration <= junctionList.size(); iteration++) {
            runjuctionFunction(iteration);
            autoSetPositions(iteration);
            encoderDriveForward(400, 1);
        }

    }

    /*
    --junction location key--
        1   2   3
 start^ 4   5   6
        7   8   9
        you stand here -->
    */
    //TODO adjust all time drives based on robot speed
    //TODO adjust all turning angles
    //TODO adjust all slide positions

    // one encoder tick = 0.02426 inches
    // desired distance in inches / 0.02426 inches = encoder ticks needed

    public void junction_1(){
        drive(1,0.5,500);
        sleep(500);
        //Claw0.setPosition(.6);
        //Claw1.setPosition(.4);
        slidePose1 = -3000;
        slidePose0 = -3000;
        liftSlides(slidePose1, slidePose0);
        sleep(1000);
        encoderDriveForward(1400,-1);
        sleep(3000);
        betterPivot(45);
        slidePose1 = -500;
        slidePose0 = -500;
        liftSlides(slidePose1, slidePose0);
        sleep(1000);
        encoderDriveForward(500,1);
        sleep(500);
        //Claw0.setPosition(.3);
        //Claw1.setPosition(.7);
        sleep(500);
        encoderDriveForward(500,-1);
        sleep(500);
        betterPivot(-45);
        encoderDriveForward(1000,1);
        sleep(1500);
    }
    public void junction_2(){

    }
    public void junction_3(){

    }
    public void junction_4(){

    }
    public void junction_5(){

    }
    public void junction_6(){

    }
    public void junction_7(){

    }
    public void junction_8(){

    }
    public void junction_9(){

    }
}
