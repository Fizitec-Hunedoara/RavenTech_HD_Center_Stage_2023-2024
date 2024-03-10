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

public class AutonomRosuDepartePixel extends LinearOpMode {
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
        CV_detectionType = DetectionTypes.DAY_red;
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
                if (x > 470) {
                    varrez = "Dreapta";
                } else if (x > 250 && x < 470) {
                    varrez = "Mijloc";
                } else if (x < 250) {
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
        Pose2d startPose = new Pose2d(-38.5, -62.73622, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(new Vector2d(16, 38), Math.toRadians(315)))
                .build();

        if (Objects.equals(varrez, "Mijloc")) {
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(-38.5, -32))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-36,-57),Math.toRadians(90)))
                    .splineToLinearHeading(new Pose2d(new Vector2d(-60,-36.5),Math.toRadians(181.5)),Math.toRadians(181.5))
                    .build();

        }
        if (Objects.equals(varrez, "Stanga")){
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(new Vector2d(-44, -34), Math.toRadians(110)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-36,-57),Math.toRadians(110)))
                    .splineToLinearHeading(new Pose2d(new Vector2d(-60,-36.5),Math.toRadians(181.5)),Math.toRadians(181.5))
                    .build();

        }
        if (Objects.equals(varrez, "Dreapta")){
            ts = drive.trajectorySequenceBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(new Vector2d(-28,-37),Math.toRadians(55)),Math.toRadians(55))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-36,-57),Math.toRadians(90)))
                    .splineToLinearHeading(new Pose2d(new Vector2d(-60,-36.5),Math.toRadians(181.5)),Math.toRadians(181.5))
                    .build();
        }
        drive.followTrajectorySequence(ts);
        altceva=true;
        p.intake = false;
        p.luare();
//        if (p.intake){
//             ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(-60, -36.5),Math.toRadians(181.5)))
//                     .back(2)
//                     .forward(2)
//                    .build();
//            drive.followTrajectorySequence(ts);
//        }
        p.intake = false;
        p.kdf(1500);
        p.oprire();
        ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(-60,-36.5),Math.toRadians(181.5)))
                .back(2.5)
                .lineToLinearHeading(new Pose2d(new Vector2d(-57.5,-12),Math.toRadians(180)))
                .lineTo(new Vector2d(26,-12))
                .build();
        drive.followTrajectorySequence(ts);

        if (Objects.equals(varrez, "Dreapta")){
            ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(30,-12),Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(52,-36), Math.toRadians(180)))
                    .build();
        } else if (Objects.equals(varrez, "Stanga")) {
            ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(30,-12),Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(52,-47), Math.toRadians(180)))
                    .build();
        } else if (Objects.equals(varrez, "Mijloc")) {
            ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(30,-12),Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(52,-36),Math.toRadians(180)))
                    .build();
        }
        drive.followTrajectorySequence(ts);
        p.kdf(400);
        altceva = true;
        p.kobra_kai(830, 4000,15);
        p.kdf(50);
        p.Burdu(690,0.7,p.fata_spate);
        p.kdf(50);
        altceva = false;
        //p.kdf(300);
        p.kdf(500);
        altceva = true;
//        while (!p.taci_mijloc.isPressed()){
//            p.fata_spate.setPower(-0.65);
//        }
//        p.slider1
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//       .setVelocity(-4000);
//        p.slider2.setVelocity(-4000);
//        while (!p.taci_dreapta.isPressed() && !p.taci_stanga.isPressed()) {
//        }
//        p.slider1.setVelocity(0);
//        p.slider2.setVelocity(0);
//        altceva = false;
//        ceva=true;
//        p.inchidere();
//        p.kdf(100);
        if (Objects.equals(varrez, "Dreapta")){
            ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(52,-36),Math.toRadians(180)))
                    .lineToConstantHeading(new Vector2d(46,-36))
                    .lineToLinearHeading(new Pose2d(new Vector2d(46,-47.4), Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(52,-47.4), Math.toRadians(180)))
                    .build();
        } else if (Objects.equals(varrez, "Stanga")) {
            ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(52,-46),Math.toRadians(180)))
                    .lineToConstantHeading(new Vector2d(46,-46))
                    .lineToLinearHeading(new Pose2d(new Vector2d(46,-32), Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(52.5,-32), Math.toRadians(180)))
                    .build();
        } else if (Objects.equals(varrez, "Mijloc")) {
            ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(52,-36),Math.toRadians(180)))
                    .lineToConstantHeading(new Vector2d(46,-36))
                    .lineToLinearHeading(new Pose2d(new Vector2d(46,-39.85), Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(52,-39.85),Math.toRadians(180)))
                    .build();
        }
        drive.followTrajectorySequence(ts);
        p.kdf(300);
        p.kdf(350);
        p.kdf(300);
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
        p.kdf(100);
        if (Objects.equals(varrez, "Mijloc")){
            ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(53,-39),Math.toRadians(180)))
                    .lineToConstantHeading(new Vector2d(49,-39))
                    .lineToConstantHeading(new Vector2d(49,-12))
                    .lineToConstantHeading(new Vector2d(60,-12))
                    .build();
        }
        if (Objects.equals(varrez, "Dreapta")){
            ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(53,-47.4),Math.toRadians(180)))
                    .lineToConstantHeading(new Vector2d(49,-47.4))
                    .lineToConstantHeading(new Vector2d(49,-12))
                    .lineToConstantHeading(new Vector2d(60,-12))
                    .build();
        }
        if (Objects.equals(varrez, "Stanga")){
            ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(53,-32),Math.toRadians(180)))
                    .lineToConstantHeading(new Vector2d(49,-32))
                    .lineToConstantHeading(new Vector2d(49,-12))
                    .lineToConstantHeading(new Vector2d(60,-12))
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

