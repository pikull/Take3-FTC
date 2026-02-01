package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "Goooooooon")
public class servoreset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo left = hardwareMap.get(Servo.class, "leftPark");
        Servo right = hardwareMap.get(Servo.class, "rightPark");
        Servo intake = hardwareMap.get(Servo.class, "outakeS");
        telemetry.addData("Status", "Initialized - Ready to Start");
        telemetry.update();
        waitForStart();
        if (isStopRequested())
            return;
        while (opModeIsActive()) {
            if(gamepad1.rightBumperWasPressed()){
                intake.setPosition(intake.getPosition()+0.01);
                telemetry.addData("pos",intake.getPosition());
                telemetry.update();
            }
            if(gamepad1.leftBumperWasReleased()){
                intake.setPosition(intake.getPosition()-0.01);
                telemetry.addData("pos",intake.getPosition());
                telemetry.update();
            }
           left.setPosition(0.5);
           right.setPosition(0.5);

        }
    }
}