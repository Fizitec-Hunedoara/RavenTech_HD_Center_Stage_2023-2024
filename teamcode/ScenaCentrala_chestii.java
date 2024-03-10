package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ScenaCentrala_chestii extends LinearOpMode {
    public DcMotorEx motorFL,motorBR,motorFR,motorBL;
    public DcMotorEx slider1;
    public DcMotorEx slider2;
    public Servo twerker;
    public Servo stripper;
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

    public Pid_Controller_Adevarat pidRotatie = new Pid_Controller_Adevarat(0,0,0);
    public Pid_Controller_Adevarat pidY = new Pid_Controller_Adevarat(0,0,0);
    public Pid_Controller_Adevarat pidX = new Pid_Controller_Adevarat(0,0,0);
    public double Rotatie = 0, ticksPerDegree = Config.rotationCalib;
    public int verifications = 0;
    public int totalY = 0, totalX = 0, totalRot = 0;
    public double correctionR,correctionY,correctionX;
    public double ds, df, ss, sf, Y, tempRot, max;
    public double encDr, encSt, encSp;
    public HardwareMap startTh;
    public boolean ceva = false;
    public ScenaCentrala_chestii(HardwareMap startThreads) {startTh = startThreads;}
    public void Init(HardwareMap hard) {
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR");
        slider1 = hardwareMap.get(DcMotorEx.class, "slider1");
        slider2 = hardwareMap.get(DcMotorEx.class, "slider2");
        //loader1 = hardwareMap.servo.get("loader1");
        twerker = hardwareMap.servo.get("servoGoBilda");
        stripper = (Servo) hardwareMap.servo.get("rev");


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



        /*Liniile astea de cod fac ca encoderele(masoara cat a mers motorul, dar nu este foarte precis, este necesar un cablu ca sa accesezi encoder-ul) sa se opreaca si sa se reseteze la valoarea initiala*/
//        motorFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motorFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motorBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motorBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        slider1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        //Liniile astea de cod fac ca robotul sa mearga cu ajutorul encoderelor(maresc precizia)*/
        motorFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        slider1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        pidRotatie.setSetpoint(0);
        pidY.setSetpoint(0);
        pidX.setSetpoint(0);

        pidRotatie.setTolerance(Config.toleranceRotatie);
        pidY.setTolerance(Config.toleranceY);
        pidX.setTolerance(Config.toleranceX);

        pidRotatie.setPID(Config.p, Config.i, Config.d);
        pidY.setPID(Config.py, Config.iy, Config.dy);
        pidX.setPID(Config.px, Config.ix, Config.dx);

        pidRotatie.enable();
        pidY.enable();
        pidX.enable();
    }
    private void powerY(double ds, double df, double ss, double sf) {
        motorFR.setPower(df);
        motorFL.setPower(sf);
        motorBR.setPower(ds);
        motorBL.setPower(ss);
    }

    private void powerX(double ds, double df, double ss, double sf) {
        motorBL.setPower(ss);
        motorFL.setPower(sf);
        motorBR.setPower(ds);
        motorFR.setPower(df);
    }

    private void powerRot(double ds, double df, double ss, double sf) {
        motorFR.setPower(df);
        motorBL.setPower(ss);
        motorFL.setPower(sf);
        motorBR.setPower(ds);
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


    public void gotoX(double incrementalX, double maxPow){
        totalX += incrementalX;
        verifications = 0;

        pidY.enable();
        pidX.enable();
        pidRotatie.enable();

        pidX.setSetpoint(totalX);
        pidY.setSetpoint(totalY);
        pidRotatie.setSetpoint(totalRot);

        pidX.setTolerance(Config.toleranceX);
        pidY.setTolerance(Config.toleranceY);
        pidRotatie.setTolerance(Config.toleranceRotatie);
        do{
            Y = (encDr + encSt)/2;
            correctionR = -pidRotatie.performPID(Rotatie);
            correctionY = -pidY.performPID(Y);
            correctionX = pidX.performPID(encSp);
            ds = correctionR + correctionY - correctionX;
            df = correctionR + correctionY + correctionX;
            ss = -correctionR + correctionY + correctionX;
            sf = -correctionR + correctionY - correctionX;

            max = Math.abs(ds);
            max = Math.abs(df) > max ? Math.abs(df):max;
            max = Math.abs(sf) > max ? Math.abs(sf):max;
            max = Math.abs(ss) > max ? Math.abs(ss):max;
            if(max > maxPow){
                ds /= max;
                df /= max;
                sf /= max;
                ss /= max;
            }
            powerX(ds, df, ss, sf);
            verifications = pidX.onTarget() ? verifications+1 : 0;
        }while(verifications < Config.targetVerifications);
        powerX(0,0,0,0);
    }
    public void gotoY(double incrementalY, double maxPow){
        totalY += incrementalY;
        verifications = 0;

        pidY.enable();
        pidX.enable();
        pidRotatie.enable();

        pidX.setSetpoint(totalX);
        pidY.setSetpoint(totalY);
        pidRotatie.setSetpoint(totalRot);

        pidX.setTolerance(Config.toleranceX);
        pidY.setTolerance(Config.toleranceY);
        pidRotatie.setTolerance(Config.toleranceRotatie);
        do{
            Y = (encDr + encSt)/2;
            correctionR = -pidRotatie.performPID(Rotatie);
            correctionY = -pidY.performPID(Y);
            correctionX = pidX.performPID(encSp);
            ds = correctionR + correctionY - correctionX;
            df = correctionR + correctionY + correctionX;
            ss = -correctionR + correctionY + correctionX;
            sf = -correctionR + correctionY - correctionX;

            max = Math.abs(ds);
            max = Math.abs(df) > max ? Math.abs(df) : max;
            max = Math.abs(sf) > max ? Math.abs(sf) : max;
            max = Math.abs(ss) > max ? Math.abs(ss) : max;

            if (max > maxPow) {
                ds /= max;
                df /= max;
                sf /= max;
                ss /= max;
            }
            powerY(ds, df, ss, sf);
            verifications = pidY.onTarget() ? verifications + 1 : 0;
        }while(verifications < Config.targetVerifications);
        powerY(0,0,0,0);
    }
    public void rotatie(double incrementalRot, double maxPow, double tolerance){
        totalRot+=incrementalRot;
        verifications = 0;

        pidY.enable();
        pidX.enable();
        pidRotatie.enable();

        pidX.setSetpoint(totalX);
        pidY.setSetpoint(totalY);
        pidRotatie.setSetpoint(totalRot);

        pidX.setTolerance(Config.toleranceX);
        pidY.setTolerance(Config.toleranceY);
        pidRotatie.setTolerance(tolerance);
        do{
            Y = (encDr + encSt)/2;
            correctionR = -pidRotatie.performPID(Rotatie);
            correctionY = -pidY.performPID(Y);
            correctionX = pidX.performPID(encSp);
            ds = correctionR + correctionY - correctionX;
            df = correctionR + correctionY + correctionX;
            ss = -correctionR + correctionY + correctionX;
            sf = -correctionR + correctionY - correctionX;

            max = Math.abs(ds);
            max = Math.abs(df) > max ? Math.abs(df):max;
            max = Math.abs(sf) > max ? Math.abs(sf):max;
            max = Math.abs(ss) > max ? Math.abs(ss):max;
            if(max > maxPow){
                ds /= max;
                df /= max;
                sf /= max;
                ss /= max;
            }
            powerRot(ds, df, ss, sf);
            verifications = pidRotatie.onTarget() ? verifications+1 : 0;
        }while (verifications < Config.targetVerifications);
        powerRot(0,0,0,0);
    }
    @Override
    public void runOpMode() throws InterruptedException {

    }
}