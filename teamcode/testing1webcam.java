//
//package org.firstinspires.ftc.teamcode;
//
//import static org.firstinspires.ftc.teamcode.Var.*;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.opencv.core.Rect;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvWebcam;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.openftc.apriltag.AprilTagDetection;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//import org.openftc.easyopencv.OpenCvInternalCamera2;
//
//import java.util.ArrayList;
//
//@Autonomous
//public class testing1webcam extends LinearOpMode {
//    //de fapt AutonomousA2
//    private OpenCvWebcam webcam;
//    private PachetelNouOpenCV pipeline = new PachetelNouOpenCV();
//    AprilTagDetectionPipeline aprilTagDetectionPipeline;
//    private double height;
//    public double lastTime;
//    static final double FEET_PER_METER = 3.28084;
//    boolean plm = false;
//    String varrez = "Dreapta";
//    static final double COUNTSPERR = 383.6;
//    static final double GEARREDUCTION = 1;
//    static final double DIAMROT = 9.6;
//    static final double COUNTS_PER_CM = (COUNTSPERR*GEARREDUCTION) / (DIAMROT*3.1415);
//    //merge si pentru F2 acest autonom
//    private double crThreshHigh = 150;
//    private double crThreshLow = 120;
//    private double cbThreshHigh = 255;
//    private double cbThreshLow = 255;
//    int currentmotorBL;
//    int currentmotorBR;
//    int currentmotorFL;
//    int currentmotorFR;
//
//    private double lowerRuntime = 0;
//    private double upperRuntime = 0;
//    double fx = 578.272;
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;
//
//    // UNITS ARE METERS
//    double tagsize = 0.166;
//
//    int numFramesWithoutDetection = 0;
//
//    final float DECIMATION_HIGH = 3;
//    final float DECIMATION_LOW = 2;
//    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
//    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
//
//    @Override
//    public void runOpMode() {
//        CV_detectionType = DetectionTypes.DAY_red;
//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), camera);
//        webcam.setPipeline(pipeline);
//        webcam.setMillisecondsPermissionTimeout(2500);
//
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//
//                webcam.startStreaming(Webcam_w, Webcam_h, OpenCvCameraRotation.UPRIGHT);
//            }
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
//
//        telemetry.addLine("Waiting for start");
//        telemetry.update();
//        FtcDashboard.getInstance().startCameraStream(webcam, 60);
////        FtcDashboard.getInstance().startCameraStream(webcam2,60);
//
//        while (!isStarted() && !isStopRequested()) {
//            //ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
////            if(detections.size() == 0)
////            {
////                numFramesWithoutDetection++;
////
////                // If we haven't seen a tag for a few frames, lower the decimation
////                // so we can hopefully pick one up if we're e.g. far back
////                if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
////                {
////                    aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
////                }
////            }
//
//            try {
////                numFramesWithoutDetection = 0;
////
////                // If the target is within 1 meter, turn on high decimation to
////                // increase the frame rate
////                if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
////                {
////                    aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
////                }
////
////                for(AprilTagDetection detection : detections)
////                {
////                    Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
////
////                    telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
////                    telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
////                    telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
////                    telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
////                    telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
////                    telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
////                    telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
////                }
//                double width = pipeline.getRect().width;
//                double x = pipeline.getRect().x;
//                height = pipeline.getRect().height;
//                telemetry.addData("Frame Count", webcam.getFrameCount());
//                telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
////                telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
////                telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
////                telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
////                telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
////                telemetry.addData("Total frame time ms", webcam2.getTotalFrameTimeMs());
////                telemetry.addData("Pipeline time ms", webcam2.getPipelineTimeMs());
////                telemetry.addData("Overhead time ms", webcam2.getOverheadTimeMs());
////                telemetry.addData("Theoretical max FPS", webcam2.getCurrentPipelineMaxFps());
//                telemetry.addData("Rect X", x);
//                telemetry.addData("Rect Y", pipeline.getRect().y);
//                if (x < 300 && x > 200) {
//                    varrez = "Mijloc";
//                    telemetry.addData("caz ", varrez);
//
//                }
//                else if (x < 100){
//                    varrez = "Stanga";
//                    telemetry.addData("caz", varrez);
//                }
//                else{
//                    varrez = "dreapta";
//                    telemetry.addData("caz",varrez);
//                }
//                telemetry.update();
//            }
//            catch (Exception E){
//                telemetry.addData("Webcam error", "Please restart");
//            }
//
//        }
//        webcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
//            @Override
//            public void onClose() {
//
//            }
//        });
//        sleep(500);
//
//
//        FtcDashboard.getInstance().startCameraStream(webcam, 60);
//        while (opModeIsActive()){
//            // Calling getDetectionsUpdate() will only return an object if there was a new frame
//            // processed since the last time we called it. Otherwise, it will return null. This
//            // enables us to only run logic when there has been a new frame, as opposed to the
//            // getLatestDetections() method which will always return an object.
//
//        }
//
//
//
//    }
//
//
//    public double inValues(double value, double min, double max){
//        if(value < min){ value = min; }
//        if(value > max){ value = max; }
//        return value;
//    }
//
//
//
//}
///*              |
//                |
//                |
//                |
//                |
//________________|________________
//                |
//                |
//                |
//                |
//                |
//                |
//                |
//                |
//                |
//                |
//                |
//                |
//
// */
//
//    /*
//    H/W(big=1):1.588
//    H/W(small=3):0.23
//    H/W(medium=2):4.23
//     */