package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled

@TeleOp(name = "Goooooooon")
public class servoreset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Servo intake = hardwareMap.get(Servo.class, "outakeS");
        CRServo intakeS = hardwareMap.get(CRServo.class, "intakeS");

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
            intakeS.setPower(1);
            telemetry.update();


        }
    }
}