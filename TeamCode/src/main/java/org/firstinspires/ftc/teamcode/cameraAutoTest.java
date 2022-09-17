//package org.firstinspires.ftc.teamcode;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.pinkCode.ContourPipeline;
//import org.opencv.core.Scalar;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//
//@TeleOp(name="cameraAutotest")
//@Disabled
//
//public class cameraAutoTest extends LinearOpMode
//{
//    private OpenCvCamera webcam;//find webcam statement
//
//    private static final int CAMERA_WIDTH  = 320; // width  of wanted camera resolution
//    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
//
//
//    double barPos = 0;
//
//
//
//    double CrLowerUpdate = 150;
//    double CbLowerUpdate = 120;
//    double CrUpperUpdate = 255;
//    double CbUpperUpdate = 255;
//
//    double lowerruntime = 0;
//    double upperruntime = 0;
//
//    // Pink Range                                      Y      Cr     Cb
//    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 200, 0);
//    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 100);
//
//    @Override
//    public void runOpMode() throws InterruptedException
//    {
//        RobotHardware robot = new RobotHardware(hardwareMap);
//
//        // OpenCV webcam
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        //OpenCV Pipeline
//        ContourPipeline myPipeline;
//        webcam.setPipeline(myPipeline = new ContourPipeline());
//        // Configuration of Pipeline
//        myPipeline.ConfigurePipeline(0, 0,50,40,  CAMERA_WIDTH, CAMERA_HEIGHT);
//        myPipeline.ConfigureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
//        myPipeline.ConfigureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);
//        // Webcam Streaming
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//
//            }
//        });
//
//        telemetry.update();
//
//        while (!isStarted())
//        {
//            if(myPipeline.error){
//                telemetry.addData("Exception: ", myPipeline.debug);
//            }
//            // Only use this line of the code when you want to find the lower and upper values, using Ftc Dashboard (https://acmerobotics.github.io/ftc-dashboard/gettingstarted)
//            // testing(myPipeline);
//
//            // Watch our YouTube Tutorial for the better explanation
//
//            telemetry.addData("RectArea: ", myPipeline.getRectArea());
//            telemetry.update();
//
//            if(myPipeline.getRectArea() > 2000){
//                if(myPipeline.getRectMidpointX() > 200){
//                    AUTONOMOUS_C();
//                    barPos = 1;
//                }
//                else if(myPipeline.getRectMidpointX() > 100){
//                    AUTONOMOUS_B();
//                    barPos = 2;
//                }
//                else {
//                    AUTONOMOUS_A();
//                    barPos = 3;
//                }
//            }
//        }
//
//        waitForStart();
//
//        while (opModeIsActive())
//        {
//
//            if(barPos == 1)
//            {
//                forward(200, .5);
//            }
//            if(barPos == 2)
//            {
//                forward(400, .75);
//            }
//            if(barPos == 3)
//            {
//                forward(600,1);
//            }
//
//        }
//    }
//
//    void forward (int distance, double power) {
//        RobotHardware robot = new RobotHardware(hardwareMap);
//
//        robot.motorRF.setTargetPosition(distance + robot.motorRF.getCurrentPosition());
//        robot.motorRB.setTargetPosition(distance + robot.motorRB.getCurrentPosition());
//        robot.motorLM.setTargetPosition(distance + robot.motorLM.getCurrentPosition());
//        robot.motorLB.setTargetPosition(distance + robot.motorLB.getCurrentPosition());
//
//        robot.motorRF.setPower(power * .75);
//        robot.motorRB.setPower(power * .75);
//        robot.motorLB.setPower(power * .75);
//        robot.motorLF.setPower(power * .75);
//
//        robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//    }
//
//    public void AUTONOMOUS_A(){
//        telemetry.addLine("Autonomous A");
//    }
//    public void AUTONOMOUS_B(){
//        telemetry.addLine("Autonomous B");
//    }
//    public void AUTONOMOUS_C(){
//        telemetry.addLine("Autonomous C");
//    }
//}
