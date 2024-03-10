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
import org.openftc.easyopencv.OpenCvSwitchableWebcam;

import java.util.Objects;

@Autonomous
public class AutonomProblemaLa3D extends LinearOpMode {
    double rectx, recty, hperw, x, pidResult;
    String varrez;
    WebcamName webcam1;
    WebcamName webcam2;
    OpenCvSwitchableWebcam switchableWebcam;
    public boolean ceva,altceva=false;
    public PachetelNouOpenCV pipeline = new PachetelNouOpenCV();
    public Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0, 0, 0);
    public SubPrograme p = new SubPrograme(this);
    @Override
    public void runOpMode() throws InterruptedException {
        CV_detectionType = DetectionTypes.DAY_red;
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
            public void onError(int errorCode)
            {
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
        CV_detectionType = DetectionTypes.DAY_yellow;
        PiD.start();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-38.5, -62.73622, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(new Vector2d(16, 38), Math.toRadians(315)))
                .build();

        if (Objects.equals(varrez, "Mijloc")) {
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(-37, -33))
                    .turn(Math.toRadians(-15))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-61,-35.8),Math.toRadians(180)))
                    .build();

        }
        if (Objects.equals(varrez, "Stanga")){
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(new Vector2d(-53,-34),Math.toRadians(205)))
                    .turn(Math.toRadians(-10))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-60.3,-36),Math.toRadians(180)))
                    .build();

        }
        if (Objects.equals(varrez, "Dreapta")){
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(new Vector2d(-29,-33.5),Math.toRadians(240)))
                    .turn(Math.toRadians(-30))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-60.5,-35.7),Math.toRadians(180)))
                    .build();
        }
        drive.followTrajectorySequence(ts);
        altceva=true;
        p.intake = false;
        new Thread(this::scuipat).start();
        p.kdf(300);
//        if (p.intake){
//             ts = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(-60, -36.5),Math.toRadians(181.5)))
//                     .back(2)
//                     .forward(2)
//                    .build();
//            drive.followTrajectorySequence(ts);
//        }
        p.intake = false;
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(new Vector2d(-56.5,-36.5),Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(new Vector2d(-56.5,-12),Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(new Vector2d(36,-12),Math.toRadians(180)))
                .build();
        drive.followTrajectorySequence(ts);
        new Thread(this::erectie_proasta).start();
        //p.kdf(300);

        altceva = true;
        if (Objects.equals(varrez, "Dreapta")){
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(51,-34), Math.toRadians(180)))
                    .build();
            drive.followTrajectorySequence(ts);
            p.kdf(300);
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(40,-37.6),Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(51.25,-43.1),Math.toRadians(180)))
                    .build();
            drive.followTrajectorySequence(ts);
        } else if (Objects.equals(varrez, "Stanga")) {
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(51,-41), Math.toRadians(180)))
                    .build();
            drive.followTrajectorySequence(ts);
            p.kdf(300);
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(40,-37.6),Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(52.25,-33),Math.toRadians(180)))
                    .build();
            drive.followTrajectorySequence(ts);

        } else if (Objects.equals(varrez, "Mijloc")) {
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(50,-32),Math.toRadians(180)))
                    .build();
            drive.followTrajectorySequence(ts);
            p.kdf(300);
//            new Thread (() ->{
//                            p.kobra_kai_cu_ces(1100,2000,15);
//                        }).start();
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(40,-37.6),Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(51.25,-37.75
                    ),Math.toRadians(180)))
                    .build();
            drive.followTrajectorySequence(ts);
            p.kdf(200);
//            if (pipeline.getRect().x < 210){
//                ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                        .lineToLinearHeading(new Pose2d(new Vector2d(51.25,-37.78
//                        ),Math.toRadians(180)))
//                        .build();
//                drive.followTrajectorySequence(ts);
//////                p.spitPixel(400, 0.3);
//////                ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//////                        .strafeRight(2)
//////                        .build();
//////                drive.followTrajectorySequence(ts);
//////
//            }
//            else{
//                ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                        .lineToLinearHeading(new Pose2d(new Vector2d(51,-37.23),Math.toRadians(180)))
//                        .build();
//                drive.followTrajectorySequence(ts);
////                ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
////                       .lineToLinearHeading(new Pose2d(new Vector2d(51,-35.7),Math.toRadians(180)))
//////////                        .addDisplacementMarker(() ->new Thread (() ->{
//////////                            Jupanul();
//////////                        }).start())
////                        .build();
//////                ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//////                        .lineToLinearHeading(new Pose2d(new Vector2d(51,-31),Math.toRadians(180)))
//////                        .build();
//////                drive.followTrajectorySequence(ts);
////
//////                ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//////                        .strafeLeft(6)
//////                        .build();
//////                drive.followTrajectorySequence(ts);
//            }

        }
        p.kdf(300);
        new Thread (() ->{
            Jupanul();
            switchableWebcam.setActiveCamera(webcam1);
            CV_detectionType = DetectionTypes.DAY_White;
            switchableWebcam.startStreaming(Webcam_w, Webcam_h, OpenCvCameraRotation.UPSIDE_DOWN);
                        }).start();

        //new Thread(this::disfunctieerectila).start();
        //new Thread(this::disfunctieerectila).start();





        if (Objects.equals(varrez, "Dreapta")){
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(29, -13), Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-55, -13.3), Math.toRadians(180)))
                    .build();
        } else if (Objects.equals(varrez, "Stanga")) {
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(29, -13), Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-55, -13.3), Math.toRadians(180)))

                    .build();
        } else if (Objects.equals(varrez, "Mijloc")) {
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(29, -13), Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-55, -13.3), Math.toRadians(180)))
                    .build();
        }
        drive.followTrajectorySequence(ts);


        //BAGPULAINVIATA.start();
        //drive.followTrajectorySequence(ts);
        if (pipeline.getRect().x > 320){
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeRight(3.5)
                    .forward(6)
                    .build();
        } else if (pipeline.getRect().x < 80) {
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeLeft(3.5)
                    .forward(6)
                    .build();
        }
        else {
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(6)
                    .build();
        }

        drive.followTrajectorySequence(ts);
        p.kdf(200);
        new Thread(this::scuipat).start();
        p.kdf(200);
        p.kdf(400);
        p.kdf(300);
        p.kdf(400);
        drive.update();
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(new Vector2d(39,-14),Math.toRadians(180)))
                .build();
        drive.followTrajectorySequence(ts);
        new Thread(this::erectie).start();
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                //.lineToLinearHeading(new Pose2d(new Vector2d(49,-34),Math.toRadians(180)))
                .lineToLinearHeading(
                        new Pose2d(new Vector2d(49,-34),Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(42, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(52)
                )
                .build();
        if (Objects.equals(varrez, "Stanga")){
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(
                            new Pose2d(new Vector2d(49,-37),Math.toRadians(180)),
                            SampleMecanumDrive.getVelocityConstraint(42, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(52)
                    )
                    .build();
        }

        drive.followTrajectorySequence(ts);
        p.kdf(200);
        p.kdf(200);
        fata_spate();
        disfunctieerectila();




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
        p.kdf(50);
        altceva = false;
        p.kdf(50);

    }
    public synchronized void erectie_proasta(){
        p.kdf(100);
        altceva = true;
        p.kobra_kai(950, 5000,15);
        p.kdf(50);
        p.Burdu(660,0.7,p.fata_spate);
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
        p.setSugatorPower(-0.9);
        p.kdf(2700);
        p.setSugatorPower(0.9);
        p.kdf(2000);
        p.setSugatorPower(0);


    }
    public synchronized void fata_spate(){
        p.kdf(200);
        while (!p.taci_mijloc.getState()){
            p.fata_spate.setPower(-0.55);
        }
        p.fata_spate.setPower(0);


    }
}
