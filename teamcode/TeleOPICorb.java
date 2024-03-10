//sa sugeti pula toti
//bogdan are o reputatie in FTC
// teleop robot nou
package org.firstinspires.ftc.teamcode;


import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Point;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp
public class TeleOPICorb extends OpMode {
    /* DcMotor este un tip de variabila cu care se declara motoarele, dar DcMotorEx este aceeasi chestie, doar cu mai multe functii*/
    public DcMotorEx motorFL, motorBR, motorFR, motorBL;
    public DcMotorEx slider1;
    public DcMotorEx slider2;
    public DcMotorEx sugator;
    public Servo dreptul;
    public Servo relu;
    public DigitalChannel taci_mijloc;
    public DigitalChannel taci_outtake;

    public DcMotorEx fata_spate;

    public boolean urcat = false;
    public Servo aruncatordeflacari, gheruta_puta_L, gheruta_puta_R;
    //public Servo SpateDreapta;
    public CRServo Outtake_ejac;
    public TouchSensor taci_dreapta;
    public TouchSensor taci_stanga;
  //  public Servo scula;
    public Servo andrei;
    public boolean ok_senzor = true;

    DistanceSensor pulosu;
    //ColorSensor ungur;
    public String Corvinu = "Corvinu";
    public double lastTime;
    public boolean stop = false;
    public boolean deschis;
    private BNO055IMU imu = null;      // Control/Expansion Hub IMU
    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;
    static final double COUNTSPERR = 383.6;
    static final double GEARREDUCTION = 1;
    static final double DIAMROT = 9.6;
    static final double COUNTS_PER_CM = (COUNTSPERR * GEARREDUCTION) / (DIAMROT * 3.1415);
    //merge si pentru F2 acest autonom
    private double crThreshHigh = 150;
    private double crThreshLow = 120;
    private double cbThreshHigh = 255;
    private double cbThreshLow = 255;
    int currentmotorBL;
    int currentmotorBR;
    int currentmotorFL;
    int currentmotorFR;

    //ColorSensor ungur;
    public double poz_spate_stanga = 0.5;
    public double poz_spate_dreapta = 0.5;

    public boolean ceva = false;
    public boolean miklosDiklosKurtos = false;
    double poz_servo = 0.7;
    double poz_handi = 0.5;
    Point center;

    // AnalogInput potentiometer;
    double CurrentVoltage;

    public Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0, 0, 0);
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    private final Thread System = new Thread(new Runnable() {
        @Override
        public void run() {
//            stripper.setPosition(0.2);
//            DaDinPulaMessi.setPosition(DaDinPulaMessi.getPosition());

            //lastTime = java.lang.System.currentTimeMillis()
            /*Thread-urile nu vor rula la infinit fara acest while, ci vor rula numai o data. Asta este foarte folositor pentru Telecomandat, dar fara while se pot face thread-uri pentru autonom in unele cazuri*/
            pid.enable();
            while (!stop) {
                pid.setPID(Config.pstatic, Config.istatic, Config.dstatic);
//                slider1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Config.p, Config.i, Config.d, 0));
//                slider2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Config.p, Config.i, Config.d, 0));


                fata_spate.setPower(gamepad2.right_stick_y / 2.5);




                if (gamepad2.left_stick_y != 0.0) {
                    slider1.setPower(-gamepad2.left_stick_y);
                    slider2.setPower(-gamepad2.left_stick_y);
                    ceva = true;
                } else {
                    if (ceva) {
                        ceva = false;
                        pid.setSetpoint(slider1.getCurrentPosition());
                    }
                    if(taci_dreapta.isPressed() || taci_stanga.isPressed()){
                        slider1.setPower(0);
                        slider2.setPower(0);
                    }
                    else {
                        pidResult = pid.performPID(slider1.getCurrentPosition());
                        slider1.setPower(pidResult);
                        slider2.setPower(pidResult);
                    }
                }

                if (gamepad2.dpad_down) {
                  //  scula.setPosition(0.37);
                    Outtake_ejac.setPower(-1);
                    sugator.setPower(-0.4);
                   // relu.setPosition(0.85);
                  //  dreptul.setPosition(0);

                }
                if (gamepad2.dpad_left) {
                    sugator.setPower(0);
                    Outtake_ejac.setPower(0);
                  //  dreptul.setPosition(0);
                   // relu.setPosition(0.85);
                }


                if (gamepad2.dpad_right) {
                     Outtake_ejac.setPower(0.3);
                    try {
                        Thread.sleep(300);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    Outtake_ejac.setPower(0);


                    /*Outtake_ejac.setPower(1);
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    Outtake_ejac.setPower(0);
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    Outtake_ejac.setPower(1);

                     */

                }


                if (gamepad2.right_bumper) {
                    urcare(2090, 4000, 15);
                    //kobra_kai(2090,4000,15);
                }



                if (gamepad2.dpad_up) {
                    Outtake_ejac.setPower(1);
                   /* Outtake_ejac.setPower(0.3);
                    try {
                        Thread.sleep(300);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    Outtake_ejac.setPower(0);

                    */
                }

                if (gamepad2.dpad_up) {
                    sugator.setPower(0);
                }
                if (gamepad2.left_bumper) {
                   // relu.setPosition(0.99);
                   //
                    // dreptul.setPosition(0);
                    sugator.setPower(0.5);
                }
                if (gamepad2.x) {
                    kobra_kai(1429, 4000, 10);
                }
                if (gamepad2.a) {
                    Burdu(540, 0.7, fata_spate);
                }
                if (gamepad2.left_trigger>0){
                    relu.setPosition(0.450);
                    dreptul.setPosition(0.600);
                }
                if (gamepad2.right_trigger>0){
                    dreptul.setPosition(0.51);
                    relu.setPosition(0.5385);

                }


                if (gamepad2.b) {
                  //  scula.setPosition(0.37);
                    Outtake_ejac.setPower(0);
                    while (!taci_mijloc.getState()) {
                        fata_spate.setPower(-0.65);
                    }
                    fata_spate.setPower(0);
                    slider1.setVelocity(-5000);
                    slider2.setVelocity(-5000);
                    while (!taci_dreapta.isPressed() && !taci_stanga.isPressed()) {
                    }
                    slider1.setVelocity(0);
                    slider2.setVelocity(0);
                    ceva = true;

                    slider1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    slider2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    fata_spate.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

                    slider1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    slider2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    fata_spate.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

                }
            }
        }

        public void Burdu(int poz, double pow, DcMotorEx motor) {
            motor.setTargetPosition(poz);
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motor.setPower(pow);
            while (motor.isBusy()) {
            }
            motor.setPower(0);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//            while(sculantul.getCurrentPosition() <= poz){
//                pidResult = pid.performPID(sculantul.getCurrentPosition());
//                sculantul.setPower(pidResult);
//            }


        }


        public void anaconda(int poz1, int poz2, double pow) {
            slider1.setTargetPosition(poz1);
            slider2.setTargetPosition(poz2);
            slider1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slider2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slider1.setPower(pow);
            slider2.setPower(pow);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            //SpateStanga.setPosition(poz_servo_st);
            //SpateDreapta.setPosition(poz_servo_dr);
            while (slider1.isBusy() && slider2.isBusy()) {

            }
            slider1.setPower(0);
            slider2.setPower(0);
            slider1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            slider2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            ceva = true;
        }

        public void urcare(int poz1, int vel, double tolerance) {
            if (poz1 > slider1.getCurrentPosition()) {
                while (slider1.getCurrentPosition() < poz1) {

                    slider1.setVelocity(vel);
                    slider2.setVelocity(vel);
                }

            } else {
                while (slider1.getCurrentPosition() > poz1 + tolerance) {
                    slider1.setVelocity(-vel);
                    slider2.setVelocity(-vel);
                }
            }


            slider1.setVelocity(0);
            slider2.setVelocity(0);

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            slider1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            slider2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            ceva = true;
        }

        public void kobra_kai(int poz1, int vel, double tolerance) {

            if (poz1 > slider1.getCurrentPosition()) {
                while (slider1.getCurrentPosition() < poz1) {
                    if (fata_spate.getCurrentPosition() < 400) {
                        fata_spate.setPower(0.7);
                    } else fata_spate.setPower(0);
                    slider1.setVelocity(vel);
                    slider2.setVelocity(vel);
                }

            } else {
                while (slider1.getCurrentPosition() > poz1 + tolerance) {
                    slider1.setVelocity(-vel);
                    slider2.setVelocity(-vel);
                }
            }

//            while (slider1.getCurrentPosition() > poz1 || slider1.getCurrentPosition() < poz1 + tolerance){
//                slider2.setVelocity(-vel);
//                slider1.setVelocity(-vel);
//            }
            slider1.setVelocity(0);
            slider2.setVelocity(0);

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            //SpateStanga.setPosition(poz_servo_st);
            //SpateDreapta.setPosition(poz_servo_dr);
            slider1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            slider2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            while (fata_spate.getCurrentPosition() < 400) {
                fata_spate.setPower(0.6);
            }
            fata_spate.setPower(0);
            ceva = true;
        }


//sa imi bag pula in ea de viata ca trebuei sa ma duc acASA sa beau ce dracu stau aici ca prostu cand puteam sa fiu acasa si sa man tiramisu sa ma uit la breaking bad si sa fumez o tigara corecta dar nu ca batman ca trebuie sa filmez sex tape pt Cristache sa imi bag pula in ea de inscriere si in mortii lui de videoclip ca stau aici in loc sa stau acasa si sa dorm m a pus dracu sa ma inscriu la robotica sa ma fut


    });
    public double corection, error, pidResult;
    public int targetpos;
    /*Functia de init se ruleaza numai o data, se foloseste pentru initializarea motoarelor si chestii :)*/
    public boolean ok = true;
    double sm = 1;
    double y, x, rx;
    double max = 0;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;
    double poz_scula= 0.354;
    double poz_andrei = 0.5;
    double poz_relu = 0.5;
    double poz_dreptul = 0.5;

    boolean encodersResetting = false;
    /*Aici se declara thread-ul cu numele chassis, pentru ca contine partea de program care se ocupa de sasiu*/
    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run() {

            /*Thread-urile nu vor rula la infinit fara acest while, ci vor rula numai o data. Asta este foarte folositor pentru Telecomandat, dar fara while se pot face thread-uri pentru autonom in unele cazuri*/
            while (!stop) {

                y = -gamepad1.left_stick_y;
                x = gamepad1.left_stick_x;
                rx = gamepad1.right_stick_x;


                pmotorFL = y + x + rx;
                pmotorBL = y - x + rx;
                pmotorBR = y + x - rx;
                pmotorFR = y - x - rx;

                /*Secventele urmatoare de cod stabilesc maximul dintre modulele puterilor motoarelor cu un anumit scop...*/
                max = abs(pmotorFL);
                if (abs(pmotorFR) > max) {
                    max = abs(pmotorFR);
                }
                if (abs(pmotorBL) > max) {
                    max = abs(pmotorBL);
                }
                if (abs(pmotorBR) > max) {
                    max = abs(pmotorBR);
                }
                if (max > 1) {
                    pmotorFL /= max;
                    pmotorFR /= max;
                    pmotorBL /= max;
                    pmotorBR /= max;
                }
                /*Aici se apeleaza functia de putere cu puterile calculate anterior ale motoarelor, si le imparte la o valoare ca robotul sa se miste cu diferite viteze.*/
                //SLOW-MOTION
                //SLOW-MOTION
                if (gamepad1.left_trigger > 0.1) {
                    sm = 2;

                    //arm.setPower(poz/sm);
                } else {
                    //SLOWER-MOTION
                    if (gamepad1.right_trigger > 0 ) {
                        sm = 5;
                    } else {
                        sm = 1;
                    }

                }
                POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);

                if (gamepad1.circle) {
                    aruncatordeflacari.setPosition(0.45);
                }
                if (gamepad1.square) {
                    aruncatordeflacari.setPosition(0.3);
                }
                if (gamepad1.a && !encodersResetting) {
                    new Thread(() -> {
                        encodersResetting = true;

                        slider1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        slider2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        fata_spate.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

                        slider1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        slider2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        fata_spate.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

                        encodersResetting = false;
                    }).start();
                }
               /* if (gamepad1.dpad_down){
                    poz_relu+=0.01;
                    relu.setPosition(poz_relu);
                }
                if (gamepad1.dpad_up){
                    poz_relu-=0.01;
                    relu.setPosition(poz_relu);
                }
                if (gamepad1.dpad_left){
                    poz_dreptul+=0.01;
                    dreptul.setPosition(poz_dreptul);
                }
                if (gamepad1.dpad_right){
                    poz_dreptul+=0.01;
                    dreptul.setPosition(poz_dreptul);
                }

                */
                /*if (gamepad1.dpad_left) {
                    poz_scula+=0.001;
                    scula.setPosition(poz_scula);

                }
                if (gamepad1.dpad_right) {
                    poz_scula-=0.001;
                    scula.setPosition(poz_scula);

                }
                if (gamepad1.right_bumper) {
                    scula.setPosition(0.32);

                }
                if (gamepad1.left_bumper) {
                    scula.setPosition(0.7);
                }
//                double poz = 0.5;
//                if (gamepad1.right_trigger > 0 && poz < 1.0){
//                    poz += 0.0001;
//                    scula.setPosition(poz);
//
//                }
//                if (gamepad1.left_trigger > 0 && poz > 0){
//                    poz -= 0.0001;
//                    scula.setPosition(poz);
//
//                }

                 */

//                if (gamepad1.dpad_down) {
//                    for(poz_andrei= andrei.getPosition(); poz_andrei<=1; poz_andrei=andrei.getPosition()+0.005)
//                        andrei.setPosition(poz_andrei);
//                }





                //brat_fata.setPower(gamepad1.right_stick_y);
//                if (gamepad1.y){
//                    sculantul.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//                    slider1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//                    slider2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//                }
                // if (gamepad1.a){
                //  Translatare(45,0.6);
                // }


            }


        }

    });

    //zip_tie_ma_tai f = new zip_tie_ma_tai(true);

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        /* Liniile astea de cod fac ca motoarele sa corespunda cu cele din configuratie, cu numele dintre ghilimele.*/
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR");
        slider1 = hardwareMap.get(DcMotorEx.class, "slider1");
        slider2 = hardwareMap.get(DcMotorEx.class, "slider2");
        //  potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        sugator = hardwareMap.get(DcMotorEx.class, "sugator");
        fata_spate = hardwareMap.get(DcMotorEx.class, "szabo");
      //  scula = hardwareMap.servo.get("scula");
        andrei = hardwareMap.servo.get("andrei");


        // brat_fata = hardwareMap.get(DcMotorEx.class,"bratfata");
        // pulica = hardwareMap.get(TouchSensor.class,"senzor_touch");
        //loader1 = hardwareMap.servo.get("loader1");


        /// stripper = hardwareMap.servo.get("articulatiefata");
        // gheara = hardwareMap.servo.get("ghearafata");
        aruncatordeflacari = hardwareMap.servo.get("aruncatordeflacari");
//        gheruta_puta_R = hardwareMap.servo.get("ghearaR");
//        gheruta_puta_L = hardwareMap.servo.get("ghearaL");
       dreptul = hardwareMap.servo.get("dreptul");
       //
        //
         relu = hardwareMap.servo.get("relu");
        // DaDinPulaMessi = (Servo) hardwareMap.get("articulatiespate");
        // GhearaMessi = (Servo) hardwareMap.get("ghearaspate");
        //SpateDreapta =(Servo) hardwareMap.get("dreapta");
        //SpateStanga =(Servo) hardwareMap.get("stanga");
        Outtake_ejac = (CRServo) hardwareMap.get("rotitor");
        taci_dreapta = (TouchSensor) hardwareMap.get("tacidreapta");
        taci_stanga = (TouchSensor) hardwareMap.get("tacistanga");
        //  pendul = hardwareMap.get(DcMotorEx.class,"brat");
        taci_mijloc = (DigitalChannel) hardwareMap.get("tacimijloc");
        taci_outtake = (DigitalChannel) hardwareMap.get("senzor");



        motorFR.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorBL.setPower(0);

        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        slider2.setDirection(DcMotorEx.Direction.REVERSE);
        //SpateDreapta.setDirection(Servo.Direction.REVERSE);
        // SpateStanga.setDirection(Servo.Direction.REVERSE);
        // SpateStanga.setDirection(Servo.Direction.FORWARD);




        /*Liniile astea de cod fac ca motoarele sa poata frana de tot atunci cand ii dai sa franeze*/
        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        slider1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slider2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sugator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fata_spate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // brat_fata.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);




        /*Liniile astea de cod fac ca encoderele(masoara cat a mers motorul, dar nu este foarte precis, este necesar un cablu ca sa accesezi encoder-ul) sa se opreaca si sa se reseteze la valoarea initiala*/
//        motorFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motorFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motorBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motorBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        slider1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sugator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fata_spate.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //brat_fata.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        //Liniile astea de cod fac ca robotul sa mearga cu ajutorul encoderelor(maresc precizia)*/
        motorFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        slider1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sugator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fata_spate.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // brat_fata.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.setMsTransmissionInterval(50);


    }

    /*Public void start se porneste o data cand se apasa pe butonul de start*/
    public void start() {
        /*Aici se pornesc thread-urile:
        Thread-urile fac parte din procesul numit multi-threading, care separa functionarea liniara a programului in mai multe bucati de program care ruleaza in acelasi timp, care se numesc thread-uri.
        De asemenea, daca folosessti aceeasi variabila in mai multe thread-uri, thread-urile se vor impiedica si se vor opri un pic.
         */
        Chassis.start();
        System.start();
    }

    public void stop() {
        stop = true;
    }

    /*Aici se afla partea de telemetrie a robotului.
    Telemetria iti arata pe driver hub/telefon cu driver station o valoare pe care ai stabilit-o, cu un anumit text dinaintea lui*/
    @Override
    public void loop() {
        /*Exemplu de telemetrie, in care Hotel este scrisul dinainte, si trivago este valoarea, care este un string cu numele trivago :)))))*/
//        telemetry.addData("Left Bumper", gamepad1.left_bumper);
//        telemetry.addData("Left Bumper", gamepad1.left_bumper);

        telemetry.addData("Pozitie slider1: ", slider1.getCurrentPosition());
        telemetry.addData("Pozitie slider2: ", slider2.getCurrentPosition());
        // telemetry.addData("Pozitie articulatie spate", DaDinPulaMessi.getPosition());
        // telemetry.addData("Pozitie gheara spate",GhearaMessi.getPosition());
        telemetry.addData("poz sugator", sugator.getCurrentPosition());
        // telemetry.addData("poz gheara fata",gheara.getPosition());
        // telemetry.addData("poz articulatie fata", stripper.getPosition());
        // telemetry.addData("poz brat fata", brat_fata.getCurrentPosition());
        //telemetry.addData("stare senzor touch", pulica.isPressed());
        telemetry.addData("avionas", aruncatordeflacari.getPosition());
        //telemetry.addData("poz_dreapta",SpateDreapta.getPosition());
        // telemetry.addData("poz_stanga",SpateStanga.getPosition());
        // telemetry.addData("Potentiometer voltage", CurrentVoltage);
        telemetry.addData("pozitie brat fata", CurrentVoltage);
        telemetry.addData("putere servo", Outtake_ejac.getPower());
        telemetry.addData("touch dreapta",taci_dreapta.isPressed());
        telemetry.addData("touch stanga", taci_stanga.isPressed());
        //telemetry.addData("touch pula mea",taci_spate.isPressed());
        //telemetry.addData("senzor boleana",ok_senzor);
        // telemetry.addData("poz servo", poz_servo);
        telemetry.addData("urcat", urcat);
        telemetry.addData("gamepad joystick bag pula in el", gamepad2.right_stick_y);
        telemetry.addData("sagetuta sus", gamepad2.dpad_up);
        telemetry.addData("tauciped x", gamepad1.touchpad_finger_1_x);
        telemetry.addData("tauciped y", gamepad1.touchpad_finger_1_y);
        telemetry.addData("miklos", miklosDiklosKurtos);
       //
        //
        // telemetry.addData("poz relu", relu.getPosition());
       // telemetry.addData("poz dreptu", dreptul.getPosition());
        telemetry.addData("touch mijloc", taci_mijloc.getState());
        telemetry.addData("bat", gamepad2.right_stick_y);
        telemetry.addData("poz fata spate", fata_spate.getCurrentPosition());
       // telemetry.addData("poz scula", scula.getPosition());
        telemetry.addData("poz andrei", andrei.getPosition());
        telemetry.addData("taci outtake", taci_outtake.getState());


        // Calling getDetectionsUpdate() will only return an object if there was a new frame
        // processed since the last time we called it. Otherwise, it will return null. This
        // enables us to only run logic when there has been a new frame, as opposed to the
        // getLatestDetections() method which will always return an object.
      /*  ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

        // If there's been a new frame...

        if(detections != null)
        {

            telemetry.addData("FPS", camera.getFps());
            telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
            telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

            // If we don't see any tags
            if(detections.size() == 0)
            {
                numFramesWithoutDetection++;

                // If we haven't seen a tag for a few frames, lower the decimation
                // so we can hopefully pick one up if we're e.g. far back
                if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                }
            }
            // We do see tags!
            else
            {

                numFramesWithoutDetection = 0;

                // If the target is within 1 meter, turn on high decimation to
                // increase the frame rate
                if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                }


                for(AprilTagDetection detection : detections)
                {
                    miklosDiklosKurtos = detection.center.x > 596;

                    Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
                    telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                    telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
                    telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
                    telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
                    telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
                    telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
                    telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
                    telemetry.addData("webcam pose",detection.pose);
                    telemetry.addData("webcam corners",detection.corners);
                    telemetry.addData("webcam margins",detection.decisionMargin);
                    telemetry.addData("webcam center",detection.center);
                }
            }

            telemetry.update();
        }

        try {
            Thread.sleep(20);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

       */


        //telemetry.addData("asa nu e odo",)
//        telemetry.addData("MOTORBL: ", motorBL.getCurrentPosition());
//        telemetry.addData("MOTORBR: ", motorBR.getCurrentPosition());
//        telemetry.addData("MOTORFL: ", motorFL.getCurrentPosition());
//        telemetry.addData("MOTORFR: ", motorFR.getCurrentPosition());
//        telemetry.addData("GP", gamepad2.right_stick_y);
//        telemetry.addData("MotorSetpoint: ",Config.armcoefficient * gamepad2.right_stick_y);
//        telemetry.addData("MotorVelocity: ", sculantul.getVelocity());
//        telemetry.addData("error:",pid.getError());
//        telemetry.addData("getSetpoint:",pid.getSetpoint());
//        telemetry.addData("Perror:",pid.getError() * pid.getP());
//        telemetry.addData("Ierror:",pid.getISum() * pid.getI());
//        telemetry.addData("Derror:",pid.getDError() * pid.getD());
//        telemetry.addData("Corectie", pidResult);
//        telemetry.addData("Slider1", slider1.getPower());
//        telemetry.addData("Slider2", slider2.getPower());
//        telemetry.addData("MotorPower", sculantul.getPower());
////        telemetry.addData("Kurtos", ungur.red());
////        telemetry.addData("Gulas", ungur.blue());
//        telemetry.addData("Servo poz:", loader1.getPosition());
//        telemetry.addData("Belim poz bala", balanganitor.getPosition());
//        telemetry.addData("port bala",balanganitor.getConnectionInfo());
//        telemetry.addData("port bala",balanganitor.getPortNumber());
//        telemetry.addData("Hai", Corvinu);
//        telemetry.addData("slider1 rotatii/velicty", slider1.getVelocity());
//        telemetry.addData("slider1 rotatii/velicty", slider1.getVelocity());
//        telemetry.addData("slider2 rotatii/velicty", slider2.getVelocity());
//        telemetry.addData("slider2 rotatii/velicty", slider2.getVelocity());
        //Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //telemetry.addData("robot heading", angles.firstAngle);


        //telemetry.addData("Distanta", pulosu.getDistance(DistanceUnit.CM));
        telemetry.update();
    }


    public void POWER(double df1, double sf1, double ds1, double ss1) {
        motorFR.setPower(df1);
        motorBL.setPower(ss1);
        motorFL.setPower(sf1);
        motorBR.setPower(ds1);
    }

}

/*  __          _
 / _|        | |
| |_ ___  ___| |_
|  _/ _ \/ _ \ __|
| ||  __/  __/ |_
|_| \___|\___|\__|

 */