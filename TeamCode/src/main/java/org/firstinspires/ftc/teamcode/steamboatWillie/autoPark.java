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


@Autonomous(name="autoPark")
//@Disabled

public class autoPark extends LinearOpMode {

    //RobotHardware robot = new RobotHardware(hardwareMap);

    BNO055IMU imu;
    Orientation angles;
    double driveTimeVar;

    public DcMotor motorRF = null;
    public DcMotor motorLF = null;
    public DcMotor motorRB = null;
    public DcMotor motorLB = null;
    //public DcMotor Slide0 = null;
    //public DcMotor Slide1 = null;
    public Servo Claw0 = null;
    public Servo Claw1 = null;

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

    int slidePose0 = 0;
    int slidePose1 = 0;

    boolean backPressed = false;

    //for the programmable
    List<String> junctionList = new ArrayList<String>();

    AprilTagDetection tagOfInterest = null;




    @Override
    public void runOpMode() throws InterruptedException {

        //TODO is there a reason why we never called the motors like this? They are called like this in the TeleOp, so i just added this to try to fix the null pointer

        motorRF = hardwareMap.dcMotor.get("motorRF");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorLB = hardwareMap.dcMotor.get("motorLB");

        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);


        //Slide0 = hardwareMap.dcMotor.get("Slide0");
        //Slide1 = hardwareMap.dcMotor.get("Slide1");
        Claw0 = hardwareMap.servo.get("Claw0");
        Claw1 = hardwareMap.servo.get("Claw1");

        //Slide0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Slide0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Slide0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Slide0.setDirection(DcMotorSimple.Direction.REVERSE);

        //robot.motorLF.setDirection(DcMotorSimple.Direction.REVERSE);
        //robot.motorRF.setDirection(DcMotorSimple.Direction.REVERSE);

        Claw0.setPosition(.5);
        Claw1.setPosition(.5);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);

        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        while (isStopRequested()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("current angle", angles.firstAngle);
            telemetry.update();
        }

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            //trajectory 1 dot
            parkStart();
            encoderDriveForward(-1050,-.5);
            sleep(2000);
        } else if (tagOfInterest.id == MIDDLE) {
            //trajectory 2 dots
            parkStart();
        } else {
            //trajectory 3 dots
            parkStart();
            encoderDriveForward(800,.5);
            sleep(2000);
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        String[] tempArray = new String[junctionList.size()];
        junctionList.toArray(tempArray);
        for (int i = 0; i < junctionList.size(); i++) {
            telemetry.addLine("junctions" + tempArray[i]);
        }
    }

    public void liftSlides(int slidePos1, int slidePos0) {
        //Slide1.setTargetPosition(slidePos1);
        //Slide1.setPower(1);
        //Slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Slide0.setTargetPosition(slidePos0);
        //Slide0.setPower(1);
        //Slide0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void betterPivot(double angle) {

        //TODO I don't know why we call the hardware map multiple times. if anyone has any input, please comment
        //RobotHardware robot = new RobotHardware(hardwareMap);

        double turnpower;
        double angleError = 0;
        double Kp = 0.0125;//proportional gain
        double reset = 0;
        double Ki = 0.0001;
        double Kd = 0.01;


        double I = 0;
        int J = 0;

        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }

        if (angle > 179 || angle < -179) {
            angle = 179;
        }

        while (I == 0) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            //PID math
            if (angles.firstAngle >= 0 && angle > 0) {
                angleError = Math.abs(angle) - Math.abs(angles.firstAngle);
                //reset = reset + (Ki * angleError);
                turnpower = (angleError * Kp);// + reset + (Kd * (angleError - oldAngleError));
            } else if (angles.firstAngle <= 0 && angle > 0) {
                angleError = Math.abs(angle) - Math.abs(angles.firstAngle);
                turnpower = (angleError * Kp);
            } else {
                angleError = -Math.abs(angle) + Math.abs(angles.firstAngle);
                //reset = reset - (Ki * angleError);
                turnpower = (angleError * Kp);// + reset + (Kd * (angleError - oldAngleError));
            }

            motorRF.setPower(turnpower);
            motorRB.setPower(turnpower);
            motorLB.setPower(-turnpower);
            motorLF.setPower(-turnpower);

            telemetry.addData("Left", I);
            telemetry.addData("current angle", angles.firstAngle);
            telemetry.addData("target angle", angle);
            telemetry.addData("J val", J);
            telemetry.addData("angle error", angleError);
            telemetry.addData("turn power", turnpower);

            telemetry.update();


            if (angles.firstAngle > angle - 1 && angles.firstAngle < angle + 1) {
                J++;

                telemetry.addData("Dead", I);
                telemetry.addData("current angle", angles.firstAngle);
                telemetry.addData("target angle", angle);
                telemetry.addData("J val", J);
                telemetry.addData("angle error", angleError);
                telemetry.addData("turn power", turnpower);
                telemetry.update();
            } else {
                J = 0;
            }

            if (J == 10) {
                motorRF.setPower(0);
                motorRB.setPower(0);
                motorLB.setPower(0);
                motorLF.setPower(0);

                I = 1.5;

                telemetry.addData("Dead", I);
                telemetry.addData("current angle", angles.firstAngle);
                telemetry.addData("target angle", angle);
                telemetry.addData("J val", J);
                telemetry.addData("angle error", angleError);
                telemetry.addData("turn power", turnpower);
                telemetry.update();
            }


        }


    }

    public void betterDrive(int angle, double PowerX, double PowerY, double speed, double time) {
        RobotHardware robot = new RobotHardware(hardwareMap);
        ElapsedTime driveTime = new ElapsedTime();

        driveTimeVar = driveTime.milliseconds();

        while ((driveTimeVar + time) > driveTime.milliseconds()) {

            double turnPower;

            while (angle > 180) {
                angle -= 360;
            }
            while (angle < -180) {
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

    public void drive(double Power, double speed, double time) {
        ElapsedTime driveTime = new ElapsedTime();

        driveTimeVar = driveTime.milliseconds();

        while ((driveTimeVar + time) > driveTime.milliseconds()) {
            motorRF.setPower(speed * Power);
            motorRB.setPower(speed * Power);
            motorLB.setPower(speed * Power);
            motorLF.setPower(speed * Power);

            //TODO I don't know why this is here, comment is for future me/justin
            //motorLB.setPower(1);

            telemetry.addData("motorRF Power", motorRF.getPower());
            telemetry.addData("motorRB Power", motorRB.getPower());
            telemetry.addData("motorLB Power", motorLB.getPower());
            telemetry.addData("motorLF Power", motorLF.getPower());
            telemetry.addData("time", driveTime.milliseconds());
            telemetry.update();
        }

        stop(1);
    }

    public void stop(int hi) {

        motorRF.setPower(0);
        motorRB.setPower(0);
        motorLB.setPower(0);
        motorLF.setPower(0);
    }

    void encoderDriveForward(int distance, double power) {

        motorRF.setTargetPosition(distance + motorRF.getCurrentPosition());
        motorRB.setTargetPosition(distance + motorRB.getCurrentPosition());
        motorLF.setTargetPosition(distance + motorLF.getCurrentPosition());
        motorLB.setTargetPosition(distance + motorLB.getCurrentPosition());

        motorRF.setPower(power);
        motorRB.setPower(power);
        motorLB.setPower(power);
        motorLF.setPower(power);

        motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void parkStart() {
        //drive(0.5, 1, 500);
        //drive(-0.5, 1, 100);
        ////Claw0.setPosition(.5);
        ////Claw1.setPosition(.5);
        //sleep(500);
        ////slidePose1 = 40;
        ////slidePose0 = 40;
        ////liftSlides(slidePose1, slidePose0);
        //sleep(1000);

        // one encoder tick = 0.02426 inches
        // desired distance in inches / 0.02426 inches = encoder ticks needed
        encoderDriveForward(-2075, -0.5);
        sleep(2000);

        betterPivot(90);

    }
}
