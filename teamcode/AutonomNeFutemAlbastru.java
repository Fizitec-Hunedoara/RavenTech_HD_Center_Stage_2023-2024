package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var.CV_detectionType;
import static org.firstinspires.ftc.teamcode.Var.CV_rect_y2;
import static org.firstinspires.ftc.teamcode.Var.Webcam_h;
import static org.firstinspires.ftc.teamcode.Var.Webcam_w;

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
import org.openftc.easyopencv.OpenCvSwitchableWebcam;

import java.util.Objects;
@Autonomous
public class AutonomNeFutemAlbastru extends LinearOpMode {

//import static org.firstinspires.ftc.teamcode.Var_BlueDa.CV_detectionType;


    double rectx, recty, hperw, x, pidResult;
    String varrez;
    WebcamName webcam1;
    WebcamName webcam2;
    OpenCvSwitchableWebcam switchableWebcam;
    public boolean ceva,altceva=false,supt = false,senzorel = false;
    public PachetelNouOpenCV pipeline = new PachetelNouOpenCV();
    public Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0, 0, 0);
    public SubPrograme p = new SubPrograme(this);
    @Override
    public void runOpMode() throws InterruptedException {
        CV_detectionType = Var.DetectionTypes.DAY_red;
        p.init(hardwareMap);
        p.initSasiu(hardwareMap);


        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
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


        switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
//                if (switchableWebcam.getActiveCamera().toString().equals("webcam2")){
//                    switchableWebcam.setPipeline(pipeline);
//                    switchableWebcam.startStreaming(Webcam_w, Webcam_h, OpenCvCameraRotation.UPSIDE_DOWN);
//                    switchableWebcam.setActiveCamera(webcam2);
//                }
//                else {
//                    switchableWebcam.setPipeline(pipeline);
//                    switchableWebcam.startStreaming(Webcam_w, Webcam_h, OpenCvCameraRotation.UPRIGHT);
//                    switchableWebcam.setActiveCamera(webcam1);
//                }
                switchableWebcam.setPipeline(pipeline);
                switchableWebcam.startStreaming(Webcam_w, Webcam_h, OpenCvCameraRotation.UPRIGHT);
                switchableWebcam.setActiveCamera(webcam2);


            }


            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        telemetry.addLine("waiting for start:");
        telemetry.update();
        //switchableWebcam.setActiveCamera(webcam2);

        //switchableWebcam.startStreaming(Webcam_w, Webcam_h, OpenCvCameraRotation.UPRIGHT);

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
        CV_detectionType = Var.DetectionTypes.DAY_yellow;
        CV_rect_y2 = 480;
        PiD.start();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-38.5, 62.73622, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(new Vector2d(16, 38), Math.toRadians(315)))
                .build();

        if (Objects.equals(varrez, "Mijloc")) {
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(-37, 30))
                    .turn(Math.toRadians(90))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-60,23),Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(40))
                    .build();

        }
        if (Objects.equals(varrez, "Dreapta")) {
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(new Vector2d(-55.5, 27), Math.toRadians(155)))
                    .turn(Math.toRadians(25))
                    .addDisplacementMarker(()->new Thread(() ->{
                        scuipat();
                    }).start())
                    .lineToLinearHeading(new Pose2d(new Vector2d(-60,23),Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(40))
                    .build();

        }
        if (Objects.equals(varrez, "Stanga")) {
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(new Vector2d(-29, 33.5), Math.toRadians(120)))
                    .turn(Math.toRadians(30))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-60,23),Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(40))
                    .build();
        }
        drive.followTrajectorySequence(ts);
        p.kdf(100);
        p.semi_inchis();
        p.kdf(250);
        p.deschide();
        p.kdf(100);

        // altceva=true;
        // p.intake = false;

//            new Thread(() ->{
//                switchableWebcam.setActiveCamera(webcam1);
//            }).start();
        //  p.kdf(300);
        //  p.intake = false;
        if (Objects.equals(varrez, "Stanga")){
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(-56.8,13),Math.toRadians(180)))
                    .setReversed(true)
                    .splineTo(new Vector2d(-20,-13),Math.toRadians(0))
                    .splineTo(new Vector2d(25,-13),Math.toRadians(0))
                    .addDisplacementMarker(()->new Thread(() ->{
                        supt = true;
                        erectie_proasta();
                    }).start())
                    .splineToLinearHeading(new Pose2d(new Vector2d(52, 40), Math.toRadians(180)), Math.toRadians(0),
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30))

//                    .addTemporalMarker(1,0,()->//System.out.println("Imi plac barbatii");
//                            new Thread(this::erectie_proasta).start())


                    .build();
        }
        else {
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(-56.8,13),Math.toRadians(180)))
                    .setReversed(true)
                    .splineTo(new Vector2d(-20,13),Math.toRadians(0))
                    .splineTo(new Vector2d(25,13),Math.toRadians(0))
                    .addDisplacementMarker(()->new Thread(() ->{
                        if (p.taci_outtake.getState()){
                            senzorel = true;
                        }
                        supt = true;
                        erectie_proasta();

                    }).start())

//                    .addTemporalMarker(1,0,()->//System.out.println("Imi plac barbatii");
//                            new Thread(this::erectie_proasta).start())
                    .splineToLinearHeading(new Pose2d(new Vector2d(52, 31), Math.toRadians(180)), Math.toRadians(0),
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30))



                    .build();
        }

        drive.followTrajectorySequence(ts);
        if (senzorel || p.taci_outtake.getState()){
            p.kdf(200);
            p.spitPixel(200,0.5);
            p.kdf(200);
            supt = false;
        }


        if (Objects.equals(varrez, "Stanga")){
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(42, 36), Math.toRadians(180)))
                    .build();
            drive.followTrajectorySequence(ts);
            if (pipeline.getRect().x > 40){
                ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(new Vector2d(52, 31.5), Math.toRadians(180)),
                                SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(40))
                        .build();
            }
            else {
                ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(new Vector2d(52, 33), Math.toRadians(180)),
                                SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(40))
                        .build();
            }

        }
        else if (Objects.equals(varrez, "Mijloc")){
            if (pipeline.getRect().x > 310){
                ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(new Vector2d(52, 34), Math.toRadians(180)),
                                SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(40))
                        .build();
            }
            else {
                ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(new Vector2d(52, 36), Math.toRadians(180)),
                                SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(40))
                        .build();
            }

        }
        else if (Objects.equals(varrez, "Dreapta")){
            if(pipeline.getRect().x>580){
                ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(new Vector2d(52, 44), Math.toRadians(180)),
                                SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(40))
                        .build();
            }
            else {
                ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(new Vector2d(52, 42), Math.toRadians(180)),
                                SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(40))
                        .build();
            }


        }

        drive.followTrajectorySequence(ts);
        p.kdf(100);
        p.spitPixel(500,0.5);
        p.kdf(200);
//            new Thread(this::erectie_proasta).start();
//            //p.kdf(300);
//
//            altceva = true;
//            if (Objects.equals(varrez, "Mijloc")) {
//                ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                        .lineToLinearHeading(new Pose2d(new Vector2d(45, -36), Math.toRadians(180)))
//                        .lineToLinearHeading(new Pose2d(new Vector2d(51, -37), Math.toRadians(180)))
//                        .build();
//                drive.followTrajectorySequence(ts);
//
//            }
//            p.kdf(100);
        new Thread(this::Jupanul).start();
        p.kdf(500);
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(false)
                .splineTo(new Vector2d(20,14),Math.toRadians(180))
                .splineTo(new Vector2d(-25,14),Math.toRadians(180))
                .addDisplacementMarker(()->new Thread(() ->{
                    supt = false;
                    scuipat2();
                }).start())
                .splineTo(new Vector2d(-59.6,14),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .build();
        drive.followTrajectorySequence(ts);

        p.kdf(200);
        p.inchide();
        p.kdf(400);
        p.deschide();

        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(true)
                .splineTo(new Vector2d(-20,13),Math.toRadians(0))
                .splineTo(new Vector2d(25,12.5),Math.toRadians(0))
                .addDisplacementMarker(()->new Thread(() ->{
                    supt = true;
                    erectie_proasta();
                }).start())
                .splineToLinearHeading(new Pose2d(new Vector2d(52, 38), Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                //.turn(Math.toRadians(-10))
                //  .waitSeconds(0.3)
                //.back(3)


                .build();
        drive.followTrajectorySequence(ts);

        p.spitPixel(2000,0.3);
        fata_spate();
//            p.plouat();
//            new Thread(this::scuipat).start();
//            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .splineToConstantHeading(new Vector2d(-57,-16.5),Math.toRadians(180))
//                    .build();
//            drive.followTrajectorySequence(ts);
//            p.kdf(500);
//            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .splineTo(new Vector2d(-40,-12),Math.toRadians(0))
//                    .splineTo(new Vector2d(30,-12),Math.toRadians(0))
//                    .build();
//            drive.followTrajectorySequence(ts);
//            new Thread(this::erectie_proasta);
//            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .splineToLinearHeading(new Pose2d(new Vector2d(50,-31),Math.toRadians(180)),Math.toRadians(0))
//                    .build();
//            drive.followTrajectorySequence(ts);

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
                telemetry.addData("senzor_fata_spate",p.taci_mijloc.getState());
                telemetry.addData("senzor_slider stanga",p.taci_stanga.isPressed());
                telemetry.addData("senzor_slider dreapta",p.taci_dreapta.isPressed());

                telemetry.addData("x",pipeline.getRect().x);
                telemetry.update();

            }
        }
    });
//    private final Thread BAGPULAINVIATA = new Thread(new Runnable() {
//        @Override
//        public void run() {
//            pid.enable();
//            p.timp = System.currentTimeMillis();
//            while (){
//                pid.setPID(Config.pstaticaut,Config.istaticaut,Config.dstaticaut);
//                pid.setSetpoint(320);
//                pidResult = pid.performPID((double) pipeline.getRect().x /2 + (double) pipeline.getRect().y /2);
//                p.miscare(pidResult + 0.3);
//            }
//
//        }
//    });



    public synchronized void erectie(){
        p.kdf(300);
        altceva = true;
        p.kobra_kai_retardat(1150, 2000,15);
        //p.kdf(50);
        //p.Burdu(690,0.7,p.fata_spate);
        p.kdf(300);
        altceva = false;
        p.kdf(50);

    }
    public synchronized void erectie_proasta(){
        p.kdf(100);
        altceva = true;
        p.kobra_kai(900, 5000,15);
        p.kdf(50);
        p.Burdu(400,0.7,p.fata_spate);
        p.kdf(50);
        altceva = false;

    }
    public synchronized void disfunctieerectila(){
        altceva = true;

        while (!p.taci_dreapta.isPressed() || !p.taci_stanga.isPressed() && !isStopRequested()) {
            p.slider1.setVelocity(-5000);
            p.slider2.setVelocity(-5000);
        }
        p.slider1.setVelocity(0);
        p.slider2.setVelocity(0);
        altceva = false;
        ceva=true;
        p.fata_spate.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        p.fata_spate.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    public synchronized void Jupanul(){
        while (!p.taci_mijloc.getState() && !isStopRequested()){
            p.fata_spate.setPower(-0.55);
        }
        p.fata_spate.setPower(0);
        p.kdf(200);
        altceva = true;
        while ((!p.taci_dreapta.isPressed() || !p.taci_stanga.isPressed()) && !isStopRequested()) {
            p.slider1.setVelocity(-5000);
            p.slider2.setVelocity(-5000);
        }
        p.slider1.setVelocity(0);
        p.slider2.setVelocity(0);
        altceva = false;
        ceva=true;
        p.fata_spate.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        p.fata_spate.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    public synchronized void scuipat(){

        //p.plouat();
        p.setOuttake_ejacPower(-0.9);
        p.setSugatorPower(-0.9);

        while (!p.taci_outtake.getState() && !supt){

        }
        p.kdf(400);
        p.setSugatorPower(0.9);
        p.kdf(1000);
        p.setSugatorPower(0);
        p.setOuttake_ejacPower(0);
        // p.sculat();


    }
    public synchronized void scuipat2(){
        p.kdf(1000);
        // p.plouat();
        p.setOuttake_ejacPower(-0.9);
        p.setSugatorPower(-0.9);
        while (!p.taci_outtake.getState() && !supt){

        }
        p.kdf(500);
        p.setSugatorPower(0.9);
        p.kdf(1000);
        p.setSugatorPower(0);
        p.setOuttake_ejacPower(0);
        //  p.sculat();


    }
    public synchronized void fata_spate(){
        p.kdf(200);
        while (!p.taci_mijloc.getState()){
            p.fata_spate.setPower(-0.55);
        }
        p.fata_spate.setPower(0);


    }
}



