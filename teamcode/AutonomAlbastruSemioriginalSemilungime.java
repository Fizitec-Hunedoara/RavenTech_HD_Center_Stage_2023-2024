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


public class AutonomAlbastruSemioriginalSemilungime extends LinearOpMode {
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
                webcam.startStreaming(Webcam_w, Webcam_h, OpenCvCameraRotation.UPRIGHT);
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
                if (x > 450) {
                    varrez = "Dreapta";
                } else if (x > 220 && x < 450) {
                    varrez = "Mijloc";
                } else if (x < 220) {
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
        Pose2d startPose = new Pose2d(14.783464, 62.73622, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(new Vector2d(16, 38), Math.toRadians(315)))
                .build();

        if (Objects.equals(varrez, "Mijloc")) {
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(15, 32.5))
                    .forward(7)
                    .lineToLinearHeading(new Pose2d(new Vector2d(52,35),Math.toRadians(180)))
                    .build();

        }
        if (Objects.equals(varrez, "Stanga")){
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(new Vector2d(20, 33), Math.toRadians(110)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(14,43),Math.toRadians(110)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(52,43),Math.toRadians(180)))
                    .build();
        }
        if (Objects.equals(varrez, "Dreapta")){
            ts = drive.trajectorySequenceBuilder(startPose)

                    .lineToLinearHeading(new Pose2d(new Vector2d(6,32), Math.toRadians(50)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(11,40),Math.toRadians(90)))
                    .waitSeconds(0.1)
                    .lineToLinearHeading(new Pose2d(new Vector2d(52,30.5), Math.toRadians(180)))
                    .build();
        }
        drive.followTrajectorySequence(ts);
        p.kdf(100);
        altceva = true;
        p.kobra_kai(630, 4000,15);
        p.Burdu(690,0.7,p.fata_spate);
        altceva = false;
        p.kdf(100);
        p.kdf(200);
        altceva = true;
        p.Jupanul();
        while (!p.taci_mijloc.getState()){
            p.fata_spate.setPower(-0.65);
        }
        p.kdf(55);
        p.slider1.setVelocity(-5000);
        p.slider2.setVelocity(-5000);
        while (!p.taci_dreapta.isPressed() && !p.taci_stanga.isPressed()) {

        }
        p.slider1.setVelocity(0);
        p.slider2.setVelocity(0);
        altceva = false;
        ceva=true;

        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(new Vector2d(12,61),Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(new Vector2d(-54.5,61),Math.toRadians(180)))
                .build();
        drive.followTrajectorySequence(ts);
        p.kdf(300);
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())

                .lineToConstantHeading(new Vector2d(-54.5,32.5))
                .lineToLinearHeading(new Pose2d(new Vector2d(-60,32.5),Math.toRadians(180)))
            .build();
        drive.followTrajectorySequence(ts);
        p.setSugatorPower(-0.9);
        p.kdf(100);
        p.kdf(200);
        p.kdf(100);
        p.kdf(200);
        p.kdf(100);


        ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(-60,33),Math.toRadians(181.5)))
                .lineToConstantHeading(new Vector2d(-55,33))
                .lineToLinearHeading(new Pose2d(new Vector2d(-55,10),Math.toRadians(181.5)))
                .lineToConstantHeading(new Vector2d(30,10))
                .build();
        drive.followTrajectorySequence(ts);
        p.setSugatorPower(0);

        if (Objects.equals(varrez, "Stanga")) {
            ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(30,12),Math.toRadians(181.5)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(51,35.5), Math.toRadians(181.5)))
                    .build();
        } else if (Objects.equals(varrez, "Mijloc")) {
            ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(30,12),Math.toRadians(181.5)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(51,35.5),Math.toRadians(181.5)))
                    .build();
        }
        drive.followTrajectorySequence(ts);
        p.kdf(200);
        altceva = true;
        p.kobra_kai(1100, 4000,15);
        altceva = false;
        //p.kdf(300);
        p.kdf(600);
        p.kdf(100);
        altceva = true;
        while (!p.taci_mijloc.getState()){
            p.fata_spate.setPower(-0.65);
        }
        p.Burdu(-5,-0.65,p.fata_spate);
        p.kdf(50);
        while (!p.taci_dreapta.isPressed() && !p.taci_stanga.isPressed()) {
            p.slider1.setVelocity(-5000);
            p.slider2.setVelocity(-5000);
        }
        p.slider1.setVelocity(0);
        p.slider2.setVelocity(0);
        altceva = false;
        ceva=true;

//        else {
//            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .lineToLinearHeading(new Pose2d(new Vector2d(48,32), Math.toRadians(181.5)))
//                    .lineToConstantHeading(new Vector2d(48,60))
//                    .lineToConstantHeading(new Vector2d(56,60))
//                    .build();
//            drive.followTrajectorySequence(ts);
//
//        }

//        if (Objects.equals(varrez, "Mijloc")){
//            ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(52.2,35.2),Math.toRadians(180)))
//                    .lineToConstantHeading(new Vector2d(49,35.2))
//                    .lineToConstantHeading(new Vector2d(49,15))
//                    .lineToConstantHeading(new Vector2d(60,15))
//                    .build();
//        }
//        if (Objects.equals(varrez, "Dreapta")){
//            ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(52.2,46),Math.toRadians(180)))
//                    .lineToConstantHeading(new Vector2d(49,46))
//                    .lineToConstantHeading(new Vector2d(49,15))
//                    .lineToConstantHeading(new Vector2d(60,15))
//                    .build();
//        }
//        if (Objects.equals(varrez, "Stanga")){
//            ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(52.2,35.5),Math.toRadians(180)))
//                    .lineToConstantHeading(new Vector2d(49,35.5))
//                    .lineToConstantHeading(new Vector2d(49,15))
//                    .lineToConstantHeading(new Vector2d(60,15))
//                    .build();
//        }
//        drive.followTrajectorySequence(ts);



//        else if (varrez == 3) {
//            ts = drive.trajectorySequenceBuilder(startPose)
//                    .lineToLinearHeading(new Pose2d(new Vector2d(15, 48), Math.toRadians(230)))
//                    .lineToLinearHeading(new Pose2d(new Vector2d(16, 34), Math.toRadians(200)))
//







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

