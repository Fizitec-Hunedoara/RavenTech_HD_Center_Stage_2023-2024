package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class testsenzor extends LinearOpMode {
    // Define variables for our touch sensor and motor
    TouchSensor touch;
    DcMotor motor;
    public DigitalChannel muri;


    @Override
    public void runOpMode() {
        // Get the touch sensor and motor from hardwareMap
//        touch = hardwareMap.get(TouchSensor.class, "Limit");
//        motor = hardwareMap.get(DcMotor.class, "Motor");
        muri = hardwareMap.get(DigitalChannel.class,"muri");


        // Wait for the play button to be pressed
        waitForStart();

        // Loop while the Op Mode is running
        while (opModeIsActive()) {
            // If the Magnetic Limit Swtch is pressed, stop the motor


            telemetry.addData("Stare", muri.getState());
            telemetry.update();
        }
    }}