package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor Test")
public class motor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            if(gamepad1.a) {
                intake.setPower(1);
                telemetry.addData("Status", "Initialized - Ready to Start");
                telemetry.addData("intake", intake.getVelocity());
            }
            telemetry.update();
        }
}
}
