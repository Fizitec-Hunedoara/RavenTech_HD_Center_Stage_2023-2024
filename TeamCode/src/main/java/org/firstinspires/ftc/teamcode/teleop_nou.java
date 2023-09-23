package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
    OpMode = TeleOp
    LinearOpMode = Autonom
 public class TeleOpSasiu extends OpMode {
public DcMotorEx MotorBL
         @Override
    public void init() {
MotorBL  = hardwareMap.get(DcMotorEx.class, "1"); // Motor Back-Left

           }
    /*Public void start se porneste o data cand se apasa pe butonul de start*/
    public void start(){
        Chassis.start();
        Systems.start();
    }
    /*Aici se declara thread-ul cu numele chassis, pentru ca contine partea de program care se ocupa de sasiu*/
    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run(){
            while(!stop) {
                    });
    /*Aici se declara thread-ul cu numele systems, pentru ca contine partea de program care se ocupa de sisteme*/
    private final Thread Systems = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
         if (gamepad1.left_stick_y) 
                      MotorBL.SetPower(1);
                }
        }
    });
        public void stop(){stop = true;}

   