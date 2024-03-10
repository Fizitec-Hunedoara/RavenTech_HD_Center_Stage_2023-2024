package org.firstinspires.ftc.teamcode;



//import static org.firstinspires.ftc.teamcode.Var_BlueDa.CV_detectionType;
import static org.firstinspires.ftc.teamcode.Var.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Objects;

@Autonomous
public class AutonomRosuSimpluAproape extends LinearOpMode {
    double rectx, recty, hperw, x, pidResult;
    String varrez;
    public OpenCvCamera webcam;
    public boolean ceva,altceva=false;
    public PachetelNouOpenCV pipeline = new PachetelNouOpenCV();
    public Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0, 0, 0);
    public SubPrograme p = new SubPrograme(this);
    public boolean faza1 = false,faza2 = false,faza0 = false,faza = false;
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
                if (x > 470) {
                    varrez = "Dreapta";
                } else if (x > 250 && x < 470) {
                    varrez = "Mijloc";
                } else if (x < 150) {
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


        if (Objects.equals(varrez, "Mijloc")) {
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(15, -32.5))
                    .lineTo(new Vector2d(15,-39.5))
                    .lineToLinearHeading(new Pose2d(new Vector2d(48,-37),Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(53,-37),Math.toRadians(180)))
                    .build();

        }
        if (Objects.equals(varrez, "Stanga")){
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(new Vector2d(4,-33.3), Math.toRadians(300)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(11,-40),Math.toRadians(270)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(48,-31), Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(53,-31), Math.toRadians(180)))
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
                    .lineToLinearHeading(new Pose2d(new Vector2d(14,-42),Math.toRadians(250)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(48,-42),Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(53,-42),Math.toRadians(180)))
                    .build();
        }
        drive.followTrajectorySequence(ts);
        altceva = true;
        p.kobra_kai(540, 4000,15);
        p.kdf(300);
        p.Burdu(690,0.75,p.fata_spate);
        altceva = false;
        p.kdf(300);
        altceva = true;
        p.kdf(500);
        p.kobra_kai_cu_ces(950,2000,15);
        p.kdf(400);
        altceva = false;
        fata_spate();
        p.kdf(100);
        disfunctieerectila();
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(new Vector2d(47, -37.5), Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(new Vector2d(47, -58), Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(new Vector2d(55,-58),Math.toRadians(180)))
                //.lineToLinearHeading(new Pose2d(new Vector2d(-59.5,-13),Math.toRadians(180)))
                .build();
        drive.followTrajectorySequence(ts);


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
                telemetry.addData("senzor",p.taci_mijloc.getState());
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
        p.kobra_kai_retardat(1300, 6000,15);
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

    public synchronized void schema(){
        //p.kdf(00);
        altceva = true;
        //p.kobra_kai(640, 4000,15);
        p.kobra_kai_cu_ces(640,4000,15);

        altceva = false;
    }
}


