package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Calibration")
public class Calibration extends LinearOpMode {

    // Shooter motors
    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;

    // safety servo
    private Servo safety;

    // Outtake servo
    private Servo outakeServo;

    // Calibration values
    private double shooterVelocity = 1200;
    private double servoPosition = 0.30;

    private static final double VELOCITY_STEP = 25;
    private static final double SERVO_STEP = 0.01;

    private ElapsedTime buttonTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        initHardware();

        waitForStart();
        buttonTimer.reset();

        while (opModeIsActive()) {

            if (gamepad1.b) {
                outakeServo.setPosition(outakeServo.getPosition()-0.01);        
            }
            if (gamepad1.y) {
                outakeServo.setPosition(outakeServo.getPosition()+0.01);
            }
            if (gamepad1.right_bumper) {
                rightShooter.setVelocity(rightShooter.getVelocity() + 20);
                leftShooter.setVelocity(leftShooter.getVelocity() + 20);
            }
            if (gamepad1.left_bumper) {
                rightShooter.setVelocity(rightShooter.getVelocity() - 20);
                leftShooter.setVelocity(leftShooter.getVelocity() - 20);
            }
            if (gamepad1.dpad_up) {
                safety.setPosition(safety.getPosition() + 0.01);
            }
            if (gamepad1.dpad_down) {
                safety.setPosition(safety.getPosition() - 0.01);
            }

            telemetry.addData("Left Shooter Velocity", leftShooter.getVelocity());
            telemetry.addData("Right Shooter Velocity", rightShooter.getVelocity());
            telemetry.addData("Outtake Servo Position", outakeServo.getPosition());
            telemetry.addData("Safety Servo Position", safety.getPosition());

            telemetry.update();
        }
    }

    private void initHardware() {

        leftShooter  = hardwareMap.get(DcMotor.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        outakeServo  = hardwareMap.servo.get("outakeS");

        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        safety = hardwareMap.servo.get("safety");

        outakeServo.setPosition(servoPosition);
        leftShooter.setVelocity(shooterVelocity);
        rightShooter.setVelocity(shooterVelocity);
    }
}