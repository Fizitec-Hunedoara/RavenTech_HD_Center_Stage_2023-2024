//copyright raventech restu sugeti (andrei)
//
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;
import org.openftc.easyopencv.OpenCvCamera;
import
        static org.firstinspires.ftc.teamcode.Var.*;

@Autonomous
public class test2_webcam extends LinearOpMode
{
    WebcamName webcam1;
    WebcamName webcam2;
    OpenCvSwitchableWebcam switchableWebcam;
    public PachetelNouOpenCV pipeline = new PachetelNouOpenCV();


    @Override
    public void runOpMode() throws InterruptedException
    {
        CV_detectionType = DetectionTypes.DAY_White;
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        /**
         * Here we use a special factory method that accepts multiple WebcamName arguments. It returns an
         * {@link OpenCvSwitchableWebcam} which contains a couple extra methods over simply an {@link OpenCvCamera}.
         */
        switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcam1, webcam2);

        switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                if (switchableWebcam.getActiveCamera().toString().equals("webcam2")){
                    switchableWebcam.setPipeline(pipeline);
                    switchableWebcam.startStreaming(Webcam_w, Webcam_h, OpenCvCameraRotation.UPRIGHT);
                }
                else {
                    switchableWebcam.setPipeline(pipeline);
                    switchableWebcam.startStreaming(Webcam_w, Webcam_h, OpenCvCameraRotation.UPSIDE_DOWN);
                }

            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        FtcDashboard.getInstance().startCameraStream(switchableWebcam, 60);

        while (!isStarted() && !isStopRequested())
        {
            try {
//                numFramesWithoutDetection = 0;
//
//                // If the target is within 1 meter, turn on high decimation to
//                // increase the frame rate
//                if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
//                {
//                    aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
//                }
//
//                for(AprilTagDetection detection : detections)
//                {
//                    Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
//
//                    telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//                    telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//                    telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//                    telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//                    telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
//                    telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
//                    telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
//                }
                double width = pipeline.getRect().width;
                double x = pipeline.getRect().x;

                telemetry.addData("Frame Count", switchableWebcam.getFrameCount());
                telemetry.addData("FPS", String.format("%.2f", switchableWebcam.getFps()));
//                telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
//                telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
//                telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
//                telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
//                telemetry.addData("Total frame time ms", webcam2.getTotalFrameTimeMs());
//                telemetry.addData("Pipeline time ms", webcam2.getPipelineTimeMs());
//                telemetry.addData("Overhead time ms", webcam2.getOverheadTimeMs());
//                telemetry.addData("Theoretical max FPS", webcam2.getCurrentPipelineMaxFps());
                telemetry.addData("Rect X", x);
                telemetry.addData("Rect Y", pipeline.getRect().y);
                telemetry.update();
            }
            catch (Exception E){
                telemetry.addData("Webcam error", "Please restart");
            }

            /**
             * To switch the active camera, simply call
             * {@link OpenCvSwitchableWebcam#setActiveCamera(WebcamName)}
             */
            if(gamepad1.a)
            {
                switchableWebcam.setActiveCamera(webcam1);
            }
            else if(gamepad1.b)
            {
                switchableWebcam.setActiveCamera(webcam2);
            }

            sleep(100);
        }
    }

    class SamplePipeline extends OpenCvPipeline
    {
        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);

            return input;
        }
    }
}
