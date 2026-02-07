package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled

@TeleOp(name = "Calibration")
public class Calibration extends LinearOpMode {

    // Shooter motors


    // safety servo
    private Servo safety;

    // Outtake servo
    private Servo outakeServo;

    // Calibration values

    private double servoPosition = 0.30;



    private ElapsedTime buttonTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        initHardware();

        waitForStart();
        buttonTimer.reset();

        while (opModeIsActive()) {

            if (gamepad1.bWasReleased()) {
                outakeServo.setPosition(outakeServo.getPosition()-0.01);        
            }
            if (gamepad1.yWasReleased()) {
                outakeServo.setPosition(outakeServo.getPosition()+0.01);
            }
            if (gamepad1.dpadUpWasReleased()) {
                safety.setPosition(safety.getPosition() + 0.01);
            }
            if (gamepad1.dpadDownWasReleased()  ) {
                safety.setPosition(safety.getPosition() - 0.01);
            }


            telemetry.addData("Outtake Servo Position", outakeServo.getPosition());
            telemetry.addData("Safety Servo Position", safety.getPosition());

            telemetry.update();
        }
    }

    private void initHardware() {



        outakeServo  = hardwareMap.servo.get("outakeS");


        
        safety = hardwareMap.servo.get("safety");

        outakeServo.setPosition(servoPosition);

    }
}