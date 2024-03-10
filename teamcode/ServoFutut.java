package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoFutut extends OpMode {
    Servo servo_fudul, servo2;
    double poz = 0.5,poz2 = 0.5;
    @Override
    public void init() {
        servo_fudul = hardwareMap.servo.get("dreptul");
        servo2 = hardwareMap.servo.get("relu");
    }


    @Override
    public void loop() {
        if (gamepad1.right_trigger > 0 && poz < 1.0){
            poz += 0.0001;
            servo_fudul.setPosition(poz);

        }
        if (gamepad1.left_trigger > 0 && poz > 0){
            poz -= 0.0001;
            servo_fudul.setPosition(poz);

        }
        if (gamepad1.dpad_right && poz2 < 1.0){
            poz2 += 0.0001;
            servo2.setPosition(poz2);

        }
        if (gamepad1.dpad_left  && poz2 > 0){
            poz2 -= 0.0001;
            servo2.setPosition(poz2);

        }

        if (gamepad1.a){
            servo_fudul.setPosition(0.52);//deschis
        }
        if (gamepad1.b){
            servo_fudul.setPosition(0.605);//inchis
        }
//
//        if (gamepad1.x){
//            servo_fudul.setPosition(0.603);//1 pixel
//        }
        telemetry.addData("servo poz 1",servo_fudul.getPosition());
        telemetry.addData("servo poz 2",servo2.getPosition());

        telemetry.update();
    }
}
