package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.teamcode.Var_BlueDa.CV_detectionType;
import
        static org.firstinspires.ftc.teamcode.Var.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Objects;

@Autonomous
public class AutonomAlbastruAproapePixelPeSubBare extends LinearOpMode {
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
                    .lineTo(new Vector2d(15,39.5))
                    .lineToLinearHeading(new Pose2d(new Vector2d(52,40),Math.toRadians(180)))
                    .build();

        }
        if (Objects.equals(varrez, "Stanga")){
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(new Vector2d(20, 33), Math.toRadians(110)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(14,42),Math.toRadians(110)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(52,45.25),Math.toRadians(180)))
                    .build();
        }
        if (Objects.equals(varrez, "Dreapta")){
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(new Vector2d(6,32), Math.toRadians(50)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(11,40),Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(52,35), Math.toRadians(180)))
                    .build();
        }
        drive.followTrajectorySequence(ts);
        p.galbejor();
        p.kdf(300);
        if (Objects.equals(varrez, "Dreapta")){
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(12,61),Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-54.5,61),Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-54.5,40.5),Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-60,40.5),Math.toRadians(180)))

                    .build();
        } else if (Objects.equals(varrez, "Stanga")) {
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(12,61),Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-54.5,61),Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-54.5,40.5),Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-60,40.5),Math.toRadians(180)))
                    .build();
        } else if (Objects.equals(varrez, "Mijloc")) {
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(12,61),Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-54.5,61),Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-54.5,40.5),Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-60,40.5),Math.toRadians(180)))
                    .build();
        }
        drive.followTrajectorySequence(ts);
        new Thread(this::scuipat).start();
        p.kdf(200);
        p.inchide();
        p.kdf(400);
        p.deschide();

        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(new Vector2d(-52.5, 61), Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(new Vector2d(39, 60.5), Math.toRadians(180)))

                .build();
        drive.followTrajectorySequence(ts);
        new Thread(this::erectie).start();
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                //.lineToLinearHeading(new Pose2d(new Vector2d(50.5,41),Math.toRadians(180)))
                .lineToLinearHeading(
                        new Pose2d(new Vector2d(50.05,41),Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(42, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(50)
                )
                .build();
        if (Objects.equals(varrez, "Stanga")){
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    //.lineToLinearHeading(new Pose2d(new Vector2d(50.5,38),Math.toRadians(180)))
                    .lineToLinearHeading(
                            new Pose2d(new Vector2d(50.05,38),Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(42, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(50)
                    )
                    .build();
        }

        drive.followTrajectorySequence(ts);
        p.spitPixel(500,0.5);
        fata_spate();
        new Thread(this::disfunctieerectila).start();
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeRight(20)
                .build();
        drive.followTrajectorySequence(ts);



//        else if (varrez == 3) {
//            ts = drive.trajectorySequenceBuilder(startPose)
//                    .lineToLinearHeading(new Pose2d(new Vector2d(
//                    15, 48), Math.toRadians(230)))
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
                    if (p.taci_dreapta.isPressed() || p.taci_stanga.isPressed()){
                        p.slider1.setPower(0);
                        p.slider2.setPower(0);
                    }
                    else {
                        pidResult = pid.performPID(p.slider1.getCurrentPosition());
                        p.slider1.setPower(pidResult);
                        p.slider2.setPower(pidResult);
                    }
                }
                telemetry.addData("senzor",p.taci_mijloc.getState());
                telemetry.update();
            }
        }
    });
    public synchronized void erectie(){
        p.kdf(300);
        altceva = true;
        p.kobra_kai_retardat(1000, 6000,15);
        p.kdf(200);
        p.Burdu(625,0.7,p.fata_spate);
        p.kdf(50);
        altceva = false;
        p.kdf(50);

    }
    public synchronized void disfunctieerectila(){
        altceva = true;
        p.slider1.setVelocity(-5000);
        p.slider2.setVelocity(-5000);
        while (!p.taci_dreapta.isPressed() || !p.taci_stanga.isPressed()) {
        }
        p.slider1.setVelocity(0);
        p.slider2.setVelocity(0);
        altceva = false;
        ceva=true;
        p.fata_spate.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        p.fata_spate.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    public synchronized void fata_spate(){
        p.kdf(300);
        while (!p.taci_mijloc.getState()){
            p.fata_spate.setPower(-0.65);
        }
        p.fata_spate.setPower(0);


    }
    public synchronized void schema(){
        altceva = true;
        p.kobra_kai(560, 4000,15);
        p.kdf(300);
        p.Burdu(690,0.75,p.fata_spate);
        altceva = false;
    }

    public synchronized void scuipat(){
        p.setSugatorPower(-0.9);
        p.kdf(3000);
        p.setSugatorPower(0.9);
        p.kdf(2000);
        p.setSugatorPower(0);


    }
}

