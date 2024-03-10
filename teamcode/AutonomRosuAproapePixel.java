package org.firstinspires.ftc.teamcode;



//import static org.firstinspires.ftc.teamcode.Var_BlueDa.CV_detectionType;
import static org.firstinspires.ftc.teamcode.Var.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
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
import org.openftc.easyopencv.OpenCvSwitchableWebcam;

import java.util.Arrays;
import java.util.Objects;

@Autonomous
public class AutonomRosuAproapePixel extends LinearOpMode {
    double rectx, recty, hperw, x, pidResult;
    String varrez;
    WebcamName webcam1;
    WebcamName webcam2;
    OpenCvSwitchableWebcam switchableWebcam;
    public boolean ceva,altceva=false;
    public PachetelNouHistOpenCV pipeline = new PachetelNouHistOpenCV(this);
    public Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0, 0, 0);
    public SubPrograme p = new SubPrograme(this);
    public boolean faza1 = false,faza2 = false,faza0 = false,supt = false;
    @Override
    public void runOpMode() throws InterruptedException {
        p.init(hardwareMap);
        p.initSasiu(hardwareMap);
        CV_detectionType = DetectionTypes.DAY_red;
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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
        //  switchableWebcam.setActiveCamera(webcam1);


        switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                switchableWebcam.setActiveCamera(webcam2);
                CV_detectionType = DetectionTypes.DAY_red;
                switchableWebcam.startStreaming(Webcam_w, Webcam_h, OpenCvCameraRotation.UPRIGHT);


            }



            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        telemetry.addLine("waiting for start:");
        telemetry.update();


        FtcDashboard.getInstance().startCameraStream(switchableWebcam, 60);
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
        Pose2d startPose = new Pose2d(14.783464, -62.73622, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(new Vector2d(15, 63), Math.toRadians(315)))
                .build();
//        TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(
//
//                new TranslationalVelocityConstraint(60),
//
//                new AngularVelocityConstraint(1)
//
//        ));









        if (Objects.equals(varrez, "Mijloc")) {
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(15, -32.5))
                    .lineTo(new Vector2d(15,-39.5))
                    .lineToLinearHeading(new Pose2d(new Vector2d(53,-30),Math.toRadians(180)))
                    .build();

        }
        if (Objects.equals(varrez, "Stanga")){
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(new Vector2d(4,-33.3), Math.toRadians(300)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(11,-40),Math.toRadians(270)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(53,-25), Math.toRadians(180)))
                    .build();
        }
        if (Objects.equals(varrez, "Dreapta")){
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(new Vector2d(19, -33), Math.toRadians(250)))
                    //optiune 1 lenta, dar sigura
//                    .lineToLinearHeading(new Pose2d(new Vector2d(17,52),Math.toRadians(0)))
//                    .turn(Math.toRadians(180))
//                    .waitSeconds(0.2)

                    //optiune 2 mai rapida, dar mai riscanta
                    .lineToLinearHeading(new Pose2d(new Vector2d(14,-36),Math.toRadians(250)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(52,-36),Math.toRadians(180)))
                    .build();
        }
        drive.followTrajectorySequence(ts);
        p.galbejor();
        p.kdf(300);
        new Thread(() ->{
            CV_detectionType = DetectionTypes.DAY_White;
            CV_rect_y2 = 480;
            switchableWebcam.setActiveCamera(webcam1);
            switchableWebcam.startStreaming(Webcam_w, Webcam_h, OpenCvCameraRotation.UPSIDE_DOWN);
        }).start();



        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(false)
                .splineTo(new Vector2d(20,-13),Math.toRadians(180))
                .splineTo(new Vector2d(-25,-13),Math.toRadians(180))

                .splineTo(new Vector2d(-55,-13),Math.toRadians(180))
                .build();
//          if (Objects.equals(varrez, "Stanga")) {
//            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .lineToLinearHeading(new Pose2d(new Vector2d(27, -13), Math.toRadians(180)))
//                    .lineToLinearHeading(new Pose2d(new Vector2d(-55, -13.25), Math.toRadians(180)))
//
//                    .build();
//        } else if (Objects.equals(varrez, "Mijloc")) {
//            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .lineToLinearHeading(new Pose2d(new Vector2d(27, -13), Math.toRadians(180)))
//                    .lineToLinearHeading(new Pose2d(new Vector2d(-55, -13.25), Math.toRadians(180)))
//                    .build();
//        }
        drive.followTrajectorySequence(ts);

        //BAGPULAINVIATA.start();
        //drive.followTrajectorySequence(ts);

        p.timp = System.currentTimeMillis();
        if (pipeline.getRect().x > 340){
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeRight(2.5)
                    .waitSeconds(0.2)
                    .addDisplacementMarker(()->new Thread(() ->{
                        supt = false;
                        scuipat();
                    }).start())
                    .forward(5.5)
                    .build();
        } else if (pipeline.getRect().x < 80) {
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeLeft(2.5)
                    .waitSeconds(0.2)
                    .addDisplacementMarker(()->new Thread(() ->{
                        supt = false;
                        scuipat();
                    }).start())
                    .forward(5.5)
                    .build();
        }
        else {
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(5.5)
                    .addDisplacementMarker(()->new Thread(() ->{
                        supt = false;
                        scuipat();
                    }).start())
                    .waitSeconds(0.2)

                    .build();
        }

        drive.followTrajectorySequence(ts);
        p.kdf(200);
        p.inchide();
        p.kdf(400);
        p.deschide();
        drive.update();
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(true)
                .splineTo(new Vector2d(-20,-13),Math.toRadians(0))
                .splineTo(new Vector2d(25,-12.5),Math.toRadians(0))
                .addDisplacementMarker(()->new Thread(() ->{
                    supt = true;
                    erectie();
                }).start())
                .splineToLinearHeading(new Pose2d(new Vector2d(52, -38), Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                //.turn(Math.toRadians(-10))
                //  .waitSeconds(0.3)
                //.back(3)


                .build();
        drive.followTrajectorySequence(ts);

        p.spitPixel(500,0.5);
        p.kdf(500);
        fata_spate();


        fata_spate();



    }
    private final Thread PiD = new Thread(new Runnable() {
        @Override
        public void run() {
            pid.enable();
            while(!isStopRequested()){
                if (p.taci_dreapta.isPressed() || p.taci_stanga.isPressed()){
                    p.slider1.setPower(0);
                    p.slider2.setPower(0);
                }
                else {
                    pid.setPID(Config.pstatic, Config.istatic, Config.dstatic);
                    if(altceva){
                        ceva = true;
                    }
                    else{
                        if(ceva){
                            ceva = false;
                            pid.setSetpoint(p.slider1.getCurrentPosition());
                        }

                        else {
                            pidResult = pid.performPID(p.slider1.getCurrentPosition());
                            p.slider1.setPower(pidResult);
                            p.slider2.setPower(pidResult);
                        }
                    }
                }

                telemetry.addData("senzor",p.taci_mijloc.getState());
                telemetry.addData("rect x",pipeline.getRect().x);
                telemetry.update();
            }
        }
    });
    private final Thread Sistem = new Thread(new Runnable() {
        @Override
        public void run() {
            if (!faza0){
                while (!p.taci_mijloc.getState()){
                    p.fata_spate.setPower(-0.65);
                }
                p.fata_spate.setPower(0);
                p.slider1.setVelocity(-5000);
                p.slider2.setVelocity(-5000);
                while (!p.taci_dreapta.isPressed() && !p.taci_stanga.isPressed()) {
                }
                p.slider1.setVelocity(0);
                p.slider2.setVelocity(0);
                altceva = false;
                ceva=true;
                faza0 = true;
            }

            if (faza1){
                p.kdf(300);
                altceva = true;
                p.kobra_kai(1300, 4000,15);
                //p.kdf(50);
                //p.Burdu(690,0.7,p.fata_spate);
                p.kdf(50);
                altceva = false;
                p.kdf(100);
                faza1 = false;
            }
            if (faza2){
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
                faza2 = false;

            }


        }
    });
    public synchronized void erectie(){
        p.kdf(300);
        altceva = true;
        p.kobra_kai_retardat(1150, 6000,15);
        //p.kdf(50);
        //p.Burdu(690,0.7,p.fata_spate);
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
        while (!p.taci_mijloc.getState()){
            p.fata_spate.setPower(-0.55);
        }
        p.fata_spate.setPower(0);


    }

    public synchronized void scuipat(){
        p.setSugatorPower(-0.9);
        p.kdf(2500);
        p.setSugatorPower(0.9);
        p.kdf(1700);
        p.setSugatorPower(0);
    }
    public synchronized void schema(){
        altceva = true;
        p.kobra_kai(560, 4000,15);
        p.kdf(300);
        p.Burdu(690,0.75,p.fata_spate);
        altceva = false;
    }

}


