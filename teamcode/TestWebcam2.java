
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class TestWebcam2 extends LinearOpMode {
    //de fapt AutonomousA2
    private OpenCvWebcam webcam;
    private PachetelNouOpenCV pipeline = new PachetelNouOpenCV();
    private double height;
    public double lastTime;

    boolean plm = false;
    String varrez = "Dreapta";
    static final double COUNTSPERR = 383.6;
    static final double GEARREDUCTION = 1;
    static final double DIAMROT = 9.6;
    static final double COUNTS_PER_CM = (COUNTSPERR*GEARREDUCTION) / (DIAMROT*3.1415);
    //merge si pentru F2 acest autonom
    private double crThreshHigh = 150;
    private double crThreshLow = 120;
    private double cbThreshHigh = 255;
    private double cbThreshLow = 255;
    int currentmotorBL;
    int currentmotorBR;
    int currentmotorFL;
    int currentmotorFR;

    private double lowerRuntime = 0;
    private double upperRuntime = 0;

    @Override
    public void runOpMode() {
        CV_detectionType = DetectionTypes.DAY_yellow;
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(Webcam_w, Webcam_h, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {

            }
        });
        telemetry.addLine("Waiting for start");
        telemetry.update();
        FtcDashboard.getInstance().startCameraStream(webcam, 60);

        while (!isStarted() && !isStopRequested()) {

            try {
                double width = pipeline.getRect().width;
                height = pipeline.getRect().height;
                telemetry.addData("Frame Count", webcam.getFrameCount());
                telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
                telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
                telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
                telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
                telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
                telemetry.addData("Rectangle Width:", width);
                telemetry.addData("Rectangle Height:", height);
                telemetry.addData("Rect`angle H/W:", height / width);
                telemetry.addData("Rect X", pipeline.getRect().x);
                telemetry.addData("Rect Y", pipeline.getRect().y);
                if (height < 50) {
                    telemetry.addData("conuri", plm);
                    plm = true;
                }
                telemetry.update();
            }
            catch (Exception E){
                telemetry.addData("Webcam error", "Please restart");
            }

        }



    }


    public double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }



}
/*              |
                |
                |
                |
                |
________________|________________
                |
                |
                |
                |
                |
                |
                |
                |
                |
                |
                |
                |

 */

    /*
    H/W(big=1):1.588
    H/W(small=3):0.23
    H/W(medium=2):4.23
     */