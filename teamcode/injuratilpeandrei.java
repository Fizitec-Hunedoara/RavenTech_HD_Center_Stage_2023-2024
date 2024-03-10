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

@TeleOp
public class injuratilpeandrei extends OpMode{
    public DcMotorEx motorBL;
    public DcMotorEx motorFL;
    public DcMotorEx motorFR;
    public DcMotorEx motorBR;
    public DcMotorEx brat;
    public DcMotorEx pulamea;
    public Servo rotitor;
    public Servo gheruta;
    double sm = 1;
    double y, x, rx;
    double max = 0;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;
    public boolean stop = false;
    public boolean ceva = false;

    public double corection,error,pidResult;
    public double poz_rotitor = 0.5 ,poz_gheruta = 0.5, poz_avionas = 0.5;
    public Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0, 0, 0);


    @Override
    public void init(){
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR");
        brat = hardwareMap.get(DcMotorEx.class,"brat");
        pulamea = hardwareMap.get(DcMotorEx.class,"pula");
        rotitor = hardwareMap.get(Servo.class,"rotitor");
        gheruta = hardwareMap.get(Servo.class,"gherute");



        motorFR.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorBL.setPower(0);

        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);


        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pulamea.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        brat.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pulamea.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        brat.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        motorFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        brat.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pulamea.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }
    public void start() {
        /*Aici se pornesc thread-urile:
        Thread-urile fac parte din procesul numit multi-threading, care separa functionarea liniara a programului in mai multe bucati de program care ruleaza in acelasi timp, care se numesc thread-uri.
        De asemenea, daca folosessti aceeasi variabila in mai multe thread-uri, thread-urile se vor impiedica si se vor opri un pic.
         */
        Chassis.start();
        System.start();

    }

    @Override
    public void loop() {
        telemetry.addData("pozitie gheruta", gheruta.getPosition());
        telemetry.addData("pozitie rotitor", rotitor.getPosition());
        telemetry.addData("putere brat",brat.getPower());

        telemetry.addData("pozitie pula",pulamea.getCurrentPosition());

    }

    public void stop() {
        stop = true;
    }

    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run() {

            /*Thread-urile nu vor rula la infinit fara acest while, ci vor rula numai o data. Asta este foarte folositor pentru Telecomandat, dar fara while se pot face thread-uri pentru autonom in unele cazuri*/
            while (!stop) {
                /* Liniile astea de cod iau input-ul controller-ului si il pun in niste variabile*/
                y = gamepad1.left_stick_y;
                x = -gamepad1.left_stick_x;
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
                    if (gamepad1.right_trigger > 0 ) {
                        sm = 5;
                        POWER(pmotorFR / sm, pmotorFL / sm , pmotorBR / sm, pmotorBL / sm);
                    } else {
                        sm = 0.5;
                        POWER(pmotorFR / sm, pmotorFL / sm , pmotorBR / sm, pmotorBL / sm);
                    }

            }


              /*  if (gamepad2.dpad_down){
                    gamepad2.setLedColor(60,255,120,1000);
                    gamepad1.rumble(100000000);

                }
               if (gamepad1.touchpad_finger_1_y > 0.5 ) {
                   motorBL.setPower(1);
                   motorBR.setPower(1);
                   motorFL.setPower(1);
                   motorFR.setPower(1);

               }

              */


                //brat_fata.setPower(gamepad1.right_stick_y);
//                if (gamepad1.y){
//                    sculantul.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//                    pulamea.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//                    slider2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//                }
                // if (gamepad1.a){
                //  Translatare(45,0.6);
                // }


            }


        }

    });
    private final Thread System = new Thread(new Runnable() {
        @Override
        public void run() {
            pid.enable();

//            stripper.setPosition(0.2);
//            DaDinPulaMessi.setPosition(DaDinPulaMessi.getPosition());

            //lastTime = java.lang.System.currentTimeMillis()
            /*Thread-urile nu vor rula la infinit fara acest while, ci vor rula numai o data. Asta este foarte folositor pentru Telecomandat, dar fara while se pot face thread-uri pentru autonom in unele cazuri*/

            while (!stop) {

                pid.setPID(Config.pstatic, Config.istatic, Config.dstatic);
//                pulamea.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Config.p, Config.i, Config.d, 0));
//                slider2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Config.p, Config.i, Config.d, 0));

                if(gamepad2.right_stick_y != 0.0){
                    brat.setPower(-gamepad2.right_stick_y);
                    ceva = true;
                }
                else{
                    if(ceva){
                        ceva = false;
                        pid.setSetpoint(brat.getCurrentPosition());
                    }
                    pidResult = pid.performPID(brat.getCurrentPosition());
                    brat.setPower(pidResult);
                }

                pulamea.setPower(-gamepad2.left_stick_y);


                if (gamepad2.dpad_up ){
                    rotitor.setPosition(1);
                }
                if (gamepad2.dpad_down ){
                    rotitor.setPosition(0);
                }

                if (gamepad2.dpad_right && poz_gheruta < 1){
                    poz_gheruta += 0.001;
                }
                if (gamepad2.dpad_left && poz_gheruta > 0){
                    poz_gheruta -= 0.001;
                }





                gheruta.setPosition(poz_gheruta);








            }
        }
        public void Burdu(int poz, double pow, DcMotorEx motor){
            motor.setTargetPosition(poz);
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motor.setPower(pow);
            while(motor.isBusy()){}
            motor.setPower(0);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//            while(sculantul.getCurrentPosition() <= poz){
//                pidResult = pid.performPID(sculantul.getCurrentPosition());
//                sculantul.setPower(pidResult);
//            }


        }

        public void kobra_kai(int poz1,int vel,double tolerance){

            if (poz1 > pulamea.getCurrentPosition()){
                while (pulamea.getCurrentPosition() < poz1 ){
                    pulamea.setVelocity(vel);
                }

            }
            else {
                while (pulamea.getCurrentPosition()>poz1){
                    pulamea.setVelocity(vel);
                }
            }

//            while (pulamea.getCurrentPosition() > poz1 || pulamea.getCurrentPosition() < poz1 + tolerance){
//                slider2.setVelocity(-vel);
//                pulamea.setVelocity(-vel);
//            }
            pulamea.setVelocity(0);


            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            //SpateStanga.setPosition(poz_servo_st);
            //SpateDreapta.setPosition(poz_servo_dr);
            pulamea.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            ceva = true;
        }



//sa imi bag pula in ea de viata ca trebuei sa ma duc acASA sa beau ce dracu stau aici ca prostu cand puteam sa fiu acasa si sa man tiramisu sa ma uit la breaking bad si sa fumez o tigara corecta dar nu ca batman ca trebuie sa filmez sex tape pt Cristache sa imi bag pula in ea de inscriere si in mortii lui de videoclip ca stau aici in loc sa stau acasa si sa dorm m a pus dracu sa ma inscriu la robotica sa ma fut


    });

    public void POWER(double df1, double sf1, double ds1, double ss1) {
        motorFR.setPower(df1);
        motorBL.setPower(ss1);
        motorFL.setPower(sf1);
        motorBR.setPower(ds1);
    }


}
