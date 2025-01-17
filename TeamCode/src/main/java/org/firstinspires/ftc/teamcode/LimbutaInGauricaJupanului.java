//teleop robot nou
package org.firstinspires.ftc.teamcode;


import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.message.redux.StopOpMode;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Locale;

@TeleOp
public class LimbutaInGauricaJupanului extends OpMode {
    /* DcMotor este un tip de variabila cu care se declara motoarele, dar DcMotorEx este aceeasi chestie, doar cu mai multe functii*/
    public DcMotorEx motorFL,motorBR,motorFR,motorBL;
    public DcMotorEx slider1;
    public DcMotorEx slider2;
    public DcMotorEx rotisor;
    public DcMotorEx bratIG;

    public Servo stripper;
    public Servo gheara;

    public Servo DaDinPulaMessi;
    public Servo GhearaMessi;
    DistanceSensor pulosu;
    //ColorSensor ungur;
    public String Corvinu = "Corvinu";
    public double lastTime;
    public boolean stop = false;
    public boolean deschis;
    private BNO055IMU imu         = null;      // Control/Expansion Hub IMU
    private double          robotHeading  = 0;
    private double          headingOffset = 0;
    private double          headingError  = 0;
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

    //ColorSensor ungur;
    public double poz_gheara_spate = 0.5;
    public double poz_articulatie_spate = 0.5;

    public boolean ceva = false;
    public boolean dada = true;

    public Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0, 0, 0);
    private final Thread System = new Thread(new Runnable() {
        @Override
        public void run() {
            pid.enable();
            stripper.setPosition(0.2);

            //lastTime = java.lang.System.currentTimeMillis()
            /*Thread-urile nu vor rula la infinit fara acest while, ci vor rula numai o data. Asta este foarte folositor pentru Telecomandat, dar fara while se pot face thread-uri pentru autonom in unele cazuri*/

            while (!stop) {

                pid.setPID(Config.pstatic, Config.istatic, Config.dstatic);
//                slider1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Config.p, Config.i, Config.d, 0));
//                slider2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Config.p, Config.i, Config.d, 0));

                if(gamepad2.left_stick_y != 0.0){
                    slider1.setPower(gamepad2.left_stick_y);
                    slider2.setPower(gamepad2.left_stick_y);
                    ceva = true;
                }
                else{
                    if(ceva){
                        ceva = false;
                        pid.setSetpoint(slider1.getCurrentPosition());
                    }
                    pidResult = pid.performPID(slider1.getCurrentPosition());
                    slider1.setPower(pidResult);
                    slider2.setPower(pidResult);
                }
                rotisor.setPower(gamepad2.right_stick_x);
                bratIG.setPower(gamepad2.right_stick_y);

                if (gamepad2.a) {
                    gheara.setPosition(0.4);
                }

                if (gamepad2.y) {
                    gheara.setPosition(0.6);
                }
                if(gamepad2.right_bumper) {
                    //poz_gheara_spate += 0.001;
                    GhearaMessi.setPosition(0.8);
                }
                if (gamepad2.left_bumper){
                    //poz_gheara_spate -= 0.001;
                    GhearaMessi.setPosition(0.5);
                }
                if (gamepad2.x && dada){
                    DaDinPulaMessi.setPosition(0.34);
                    dada = false;
                }
                else if (!dada && gamepad2.x){
                    DaDinPulaMessi.setPosition(0.14);
                    dada = true;
                }


                //GhearaMessi.setPosition(poz_gheara_spate);



//                if (ungur.red() > 800 && !deschis ){
//                    loader1.setPosition(0);
//                }












            }
        }
        public void Burdu(int poz, double pow, DcMotorEx motor){
            motor.setTargetPosition(poz);
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motor.setPower(pow);
            while(motor.isBusy()){}
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            while(sculantul.getCurrentPosition() <= poz){
//                pidResult = pid.performPID(sculantul.getCurrentPosition());
//                sculantul.setPower(pidResult);
//            }


        }
        public void anaconda(int poz1,int poz2, double pow){
            slider1.setTargetPosition(poz1);
            slider2.setTargetPosition(poz2);
            slider1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slider2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slider1.setPower(pow);
            slider2.setPower(pow);
            while (slider1.isBusy() && slider2.isBusy()){

            }
            slider1.setPower(0);
            slider2.setPower(0);
            slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ceva = true;

        }






    });
    public double corection,error,pidResult;
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
    /*Aici se declara thread-ul cu numele chassis, pentru ca contine partea de program care se ocupa de sasiu*/
    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run() {

            /*Thread-urile nu vor rula la infinit fara acest while, ci vor rula numai o data. Asta este foarte folositor pentru Telecomandat, dar fara while se pot face thread-uri pentru autonom in unele cazuri*/
            while (!stop) {
                /* Liniile astea de cod iau input-ul controller-ului si il pun in niste variabile*/
                y = -gamepad1.left_stick_y;
                x = gamepad1.left_stick_x;
                rx = gamepad1.right_stick_x;

                /* Liniile astea de cod iau niste variabile care reprezinta puterea fiecarui motor, cu ajutorul puterilor de la controller*/
                pmotorFL = y + x + rx;
                pmotorBL = y - x + rx;
                pmotorBR = y + x - rx;
                pmotorFR = y - x - rx;

                /*Secventele urmatoare de cod stabilesc maximul dintre modulele puterilor motoarelor cu un anumit scop...*/
                if (abs(pmotorFL) > max) {
                    max = abs(pmotorFL);
                }
                if (abs(pmotorFR) > max) {
                    max = abs(pmotorFR);
                }
                if (abs(pmotorBL) > max) {
                    max = abs(pmotorBL);
                }
                if (abs(pmotorBR) > max) {
                    max = abs(pmotorBR);
                }
                /*...care este punerea tuturor puterilor motoarelor sub 1, cum puterile de la motoare pot fi numai intre 1 si -1*/
                if (max > 1) {
                    pmotorFL /= max;
                    pmotorFR /= max;
                    pmotorBL /= max;
                    pmotorBR /= max;
                }
                /*Aici se apeleaza functia de putere cu puterile calculate anterior ale motoarelor, si le imparte la o valoare ca robotul sa se miste cu diferite viteze.*/
                //SLOW-MOTION
                //SLOW-MOTION
                if (gamepad1.left_trigger > 0) {
                    sm = 2;
                    POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
                    //arm.setPower(poz/sm);
                } else {
                    //SLOWER-MOTION
                    if (gamepad1.right_trigger > 0) {
                        sm = 5;
                        POWER(pmotorFR / sm, pmotorFL / sm , pmotorBR / sm, pmotorBL / sm);
                    } else {
                        sm = 0.5;
                        POWER(pmotorFR / sm, pmotorFL / sm , pmotorBR / sm, pmotorBL / sm);
                    }
                }
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
        rotisor = hardwareMap.get(DcMotorEx.class, "bratspate");
        bratIG = hardwareMap.get(DcMotorEx.class,"bratfata");
        //loader1 = hardwareMap.servo.get("loader1");

        stripper = hardwareMap.servo.get("articulatiefata");
        gheara = hardwareMap.servo.get("ghearafata");
        DaDinPulaMessi = (Servo) hardwareMap.get("articulatiespate");
        GhearaMessi = (Servo) hardwareMap.get("ghearaspate");




        motorFR.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorBL.setPower(0);

        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        slider2.setDirection(DcMotorEx.Direction.REVERSE);



        /*Liniile astea de cod fac ca motoarele sa poata frana de tot atunci cand ii dai sa franeze*/
        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        slider1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slider2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rotisor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bratIG.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);




        /*Liniile astea de cod fac ca encoderele(masoara cat a mers motorul, dar nu este foarte precis, este necesar un cablu ca sa accesezi encoder-ul) sa se opreaca si sa se reseteze la valoarea initiala*/
//        motorFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motorFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motorBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motorBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        slider1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rotisor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bratIG.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);



        //Liniile astea de cod fac ca robotul sa mearga cu ajutorul encoderelor(maresc precizia)*/
        motorFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        slider1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rotisor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bratIG.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);




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
        telemetry.addData("Pozitie articulatie spate", DaDinPulaMessi.getPosition());
        telemetry.addData("Pozitie gheara spate",GhearaMessi.getPosition());
        telemetry.addData("odo drept", motorFL.getCurrentPosition());
        telemetry.addData("odo spate", motorBL.getCurrentPosition());
        telemetry.addData("odo stang", motorBR.getCurrentPosition());

        telemetry.addData("poz negru",stripper.getPosition());
        telemetry.addData("poz pui",rotisor.getCurrentPosition());


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
| |_ ___  ___| |_bu
|  _/ _ \/ _ \ __|
| ||  __/  __/ |_
|_| \___|\___|\__|

 */