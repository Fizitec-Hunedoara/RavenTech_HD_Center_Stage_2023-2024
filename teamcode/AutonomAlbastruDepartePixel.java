package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.teamcode.Var_BlueDa.CV_detectionType;
import
        static org.firstinspires.ftc.teamcode.Var.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Objects;

@Autonomous
@Disabled

public class AutonomAlbastruDepartePixel extends LinearOpMode {
    double rectx, recty, hperw, x, pidResult;
    String varrez;
    public OpenCvCamera webcam;
    public boolean ceva,altceva=false;
    public PachetelNouOpenCV pipeline = new PachetelNouOpenCV();
    public Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0, 0, 0);
    public SubPrograme p = new SubPrograme(this);
    @Override
    public void runOpMode() throws InterruptedException {
        p.init(hardwareMap);
        CV_detectionType = DetectionTypes.DAY_blue;
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(Webcam_w, Webcam_h, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        telemetry.addLine("waiting for start:");
        telemetry.update();

        FtcDashboard.getInstance().startCameraStream(webcam, 60);
        while (!isStopRequested() && !isStarted()) {
            try {
                rectx = pipeline.getRect().width;
                recty = pipeline.getRect().height;
                hperw = recty / rectx;
                telemetry.addData("rectangle width:", rectx);
                telemetry.addData("rectangle height:", recty);
                telemetry.addData("height / width:", hperw);
                x = pipeline.getRect().x + pipeline.getRect().width / 2.0;
                telemetry.addData("x:", pipeline.getRect().x + pipeline.getRect().width / 2);
                if (x > 500) {
                    varrez = "Dreapta";
                } else if (x > 200 && x < 400) {
                    varrez = "Mijloc";
                } else if (x < 200) {
                    varrez = "Stanga";
                } else {
                    varrez = "Dreapta";
                }
                telemetry.addData("caz:", varrez);
            } catch (Exception E) {
                varrez = "Stanga";
                telemetry.addData("Webcam error:", "please restart");
                telemetry.update();
            }
            telemetry.update();
        }
        PiD.start();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-38.5, 62.73622, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(new Vector2d(16, 38), Math.toRadians(315)))
                .build();

        if (Objects.equals(varrez, "Mijloc")) {
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(-38.5, 34))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-36,57),Math.toRadians(270)))
                    .splineToLinearHeading(new Pose2d(new Vector2d(-60,37),Math.toRadians(178.5)),Math.toRadians(178.5))
                    .build();

        }
        if (Objects.equals(varrez, "Stanga")){
            ts = drive.trajectorySequenceBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(new Vector2d(-28,35.5),Math.toRadians(305)),Math.toRadians(305))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-38,57),Math.toRadians(270)))
                    .splineToLinearHeading(new Pose2d(new Vector2d(-60,37),Math.toRadians(178.5)),Math.toRadians(178.5))
                    .build();

        }
        if (Objects.equals(varrez, "Dreapta")){
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(new Vector2d(-44, 34), Math.toRadians(252)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-36,57),Math.toRadians(230)))
                    .splineToLinearHeading(new Pose2d(new Vector2d(-60,37),Math.toRadians(178.5)),Math.toRadians(178.5))
                    .build();
        }
        drive.followTrajectorySequence(ts);
        altceva=true;
        p.luare();
        p.kdf(1500);
        p.oprire();
        ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(-60,37.5),Math.toRadians(178.5)))
                .back(2.5)
                .lineToLinearHeading(new Pose2d(new Vector2d(-57.5,12),Math.toRadians(180)))
                .lineTo(new Vector2d(30,12))
                .build();
        drive.followTrajectorySequence(ts);

        if (Objects.equals(varrez, "Dreapta")){
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(51.5,46), Math.toRadians(181)))
                    .build();
        } else if (Objects.equals(varrez, "Stanga")) {
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(51.5,35.5), Math.toRadians(181)))
                    .build();
        } else if (Objects.equals(varrez, "Mijloc")) {
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(51.5,34),Math.toRadians(181)))
                    .build();
        }
        drive.followTrajectorySequence(ts);
        p.kdf(400);
        altceva = true;
        p.kobra_kai(830, 4000,15);
        p.Burdu(690,0.7,p.fata_spate);
        altceva = false;
        //p.kdf(300);
        p.kdf(300);

        if (Objects.equals(varrez, "Dreapta")){
            ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(52,46),Math.toRadians(181)))
                    .lineToConstantHeading(new Vector2d(47,46))
                    .lineToLinearHeading(new Pose2d(new Vector2d(47,34), Math.toRadians(181)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(52,34), Math.toRadians(181)))
                    .build();
        } else if (Objects.equals(varrez, "Stanga")) {
            ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(52,35.5),Math.toRadians(181)))
                    .lineToConstantHeading(new Vector2d(47,35.5))
                    .lineToLinearHeading(new Pose2d(new Vector2d(47,47),Math.toRadians(181)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(52,47), Math.toRadians(181)))
                    .build();
        } else if (Objects.equals(varrez, "Mijloc")) {
            ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(52,34),Math.toRadians(181)))
                    .lineToConstantHeading(new Vector2d(47,34))
                    .lineToLinearHeading(new Pose2d(new Vector2d(47,40),Math.toRadians(181)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(52,39.85),Math.toRadians(181)))
                    .build();
        }
        drive.followTrajectorySequence(ts);
        p.kdf(300);
        p.kdf(100);
        altceva = true;
        while (!p.taci_mijloc.getState()){
            p.fata_spate.setPower(-0.65);
        }
        p.slider1.setVelocity(-4000);
        p.slider2.setVelocity(-4000);
        while (!p.taci_dreapta.isPressed() && !p.taci_stanga.isPressed()) {
        }
        p.slider1.setVelocity(0);
        p.slider2.setVelocity(0);
        altceva = false;
        ceva=true;
        if (Objects.equals(varrez, "Mijloc")){
            ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(52.2,40),Math.toRadians(180)))
                    .lineToConstantHeading(new Vector2d(49,40))
                    .lineToConstantHeading(new Vector2d(49,15))
                    .lineToConstantHeading(new Vector2d(60,15))
                    .build();
        }
        if (Objects.equals(varrez, "Dreapta")){
            ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(52.2,33),Math.toRadians(180)))
                    .lineToConstantHeading(new Vector2d(49,33))
                    .lineToConstantHeading(new Vector2d(49,15))
                    .lineToConstantHeading(new Vector2d(60,15))
                    .build();
        }
        if (Objects.equals(varrez, "Stanga")){
            ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(52.2,47),Math.toRadians(180)))
                    .lineToConstantHeading(new Vector2d(49,47))
                    .lineToConstantHeading(new Vector2d(49,15))
                    .lineToConstantHeading(new Vector2d(60,15))
                    .build();
        }
        drive.followTrajectorySequence(ts);



//        else if (varrez == 3) {
//            ts = drive.trajectorySequenceBuilder(startPose)
//                    .lineToLinearHeading(new Pose2d(new Vector2d(15, 48), Math.toRadians(230)))
//                    .lineToLinearHeading(new Pose2d(new Vector2d(16, 34), Math.toRadians(200)))
//                    .build();
//        }
//        drive.followTrajectorySequence(ts);
//        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .splineToLinearHeading(new Pose2d(new Vector2d(53, 28), Math.toRadians(180)), Math.toRadians(0))
//                .build();
//        if (varrez == 1) {
//            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .lineToLinearHeading(new Pose2d(new Vector2d(16, 48), Math.toRadians(270)))
//                    .lineToLinearHeading(new Pose2d(new Vector2d(53, 41), Math.toRadians(180)))
//                    .build();
//        } else if (varrez == 2) {
//            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .lineToLinearHeading(new Pose2d(new Vector2d(53, 34), Math.toRadians(180)))
//                    .build();
//        }
    }
    private final Thread PiD = new Thread(new Runnable() {
        @Override
        public void run() {
            pid.enable();
            while(!isStopRequested()){
                pid.setPID(Config.pstatic, Config.istatic, Config.dstatic);
                if(altceva){
                    ceva = true;
                }
                else{
                    if(ceva){
                        ceva = false;
                        pid.setSetpoint(p.slider1.getCurrentPosition());
                    }
                    pidResult = pid.performPID(p.slider1.getCurrentPosition());
                    p.slider1.setPower(pidResult);
                    p.slider2.setPower(pidResult);
                }
            }
        }
    });
}

