package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Red Teleop")
public class FreshRedGoon extends LinearOpMode {
    // Hardware Components
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx rightShooter, leftShooter;
    private DcMotorEx intake;
    private Servo outakeServo, safety;
    private CRServo intakeServo;
    private Limelight3A limelight;
    private Follower follower;
    public static double SHOOTER_P = 100.0;
    public static double SHOOTER_I = 20.0;
    public static double SHOOTER_D = 0.0;
    public static double SHOOTER_F = 14.0;
    PIDFCoefficients pidf = new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
    // --- Constants ---
    // Shooter Constants
    private static final int FAR_SHOT_VELOCITY = 1550;
    private static final int CLOSE_SHOT_VELOCITY = 1218;
    private static final double OUTAKE_POSITION = 0.57;
    private static final double OUTAKE_FAR_POSITION = 0.5;
    // Alignment Constants
    private static final double MIN_TURN_POWER = 0.19;
    private static final double MAX_TURN_POWER = 0.19;
    private static final double DISTANCE_THRESHOLD = 70.0;
    private static final double ALIGNMENT_TOLERANCE = 1.0; // degrees
    // Other
    private static final double BUTTON_HOLD_THRESHOLD = 0.2; // seconds

    // --- Variables ---
    private double targetArea = 0.0;
    private LLResult result;
    private double targetShooterVelocity = 0;
    private boolean wasShooterRunning = false;

    private enum IntakeStatus {
        intakeForward,
        intakeReverse,
        intakeOff
    }

    private IntakeStatus intakeStatus = IntakeStatus.intakeOff;
    // --- Timers ---
    private ElapsedTime intakeTimer = new ElapsedTime();
    private ElapsedTime shooterButtonTimer = new ElapsedTime();
    private ElapsedTime reverseTimer = new ElapsedTime();
    private ElapsedTime intervalTimer = new ElapsedTime();
    private ElapsedTime shootingDelayTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        configureLimelight();

        telemetry.addData("Status", "Initialized - Ready to Start");
        // Initial update removed to match user request

        waitForStart();
        if (isStopRequested())
            return;

        while (opModeIsActive()) {
            updateLimelight(); // Get vision data and telemetry
            updateIntakeState(); // Automatic intake/safety logic

            handleDriveControls(); // Drive and Align
            handleShooterControls();// Shooting logic
            handleIntakeControls(); // Manual intake controls
            handleGamepad2Controls(); // Support controls

            telemetry.update();
        }
    }

    // --- Initialization ---

    private void initializeHardware() {
        // Drive Motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        // Shooter Motors
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        // Intake System
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        safety = hardwareMap.servo.get("safety");
        outakeServo = hardwareMap.servo.get("outakeS");
        intakeServo = hardwareMap.get(CRServo.class, "intakeS");

        // Directions
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Encoders
        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);
        outakeServo.setPosition(0.76);

        // Reset Timers & Logic
        intakeTimer.reset();
        shooterButtonTimer.reset();
        reverseTimer.reset();
        intervalTimer.reset();
        shootingDelayTimer.reset();
        targetShooterVelocity = 0;
        wasShooterRunning = false;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
    }

    private void configureLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        // Pipeline switching controls (kept for init selection if neededpod)
        limelight.start();
        limelight.pipelineSwitch(0);
        new calcDistance(0);
    }

    // --- Drive System ---

    private void handleDriveControls() {
        double power = 0.7; // Default drive power (from user's 0.7 in main loop)

        // Mecanum Drive
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x * power;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        leftFront.setPower((y + x + rx) / denominator);
        leftBack.setPower((y - x + rx) / denominator);
        rightFront.setPower((y - x - rx) / denominator);
        rightBack.setPower((y + x - rx) / denominator);

        // Auto-Align (Left Trigger)
        if (gamepad1.left_trigger > 0) {
            autoAlignToTarget();
        }
    }

    private void setDrivePowers(double forward, double strafe, double rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1.1);
        leftFront.setPower((forward + strafe + rotate) / denominator);
        leftBack.setPower((forward - strafe + rotate) / denominator);
        rightFront.setPower((forward - strafe - rotate) / denominator);
        rightBack.setPower((forward + strafe - rotate) / denominator);
    }

    private double calculateProportionalTurnPower(double targetX) {
        double distanceFromCenter = Math.abs(targetX);
        double normalizedDistance = Math.min(distanceFromCenter / 30.0, 1.0);
        return MIN_TURN_POWER + (normalizedDistance * (MAX_TURN_POWER - MIN_TURN_POWER));
    }

    // --- Limelight & Vision ---
    private void updateLimelight() {
        result = limelight.getLatestResult();
        LLStatus status = limelight.getStatus();
        telemetry.addData("Limelight Status", status.getName());
        telemetry.addData("FPS", "%d", (int) status.getFps());
        if (result.isValid()) {
            telemetry.addData("Target X Offset", "%.2f", result.getTx());
        } else {
            telemetry.addData("Target", "Not Found");
        }
    }

    private void autoAlignToTarget() {
        if (!result.isValid()) {
            telemetry.addData("Auto-Align", "No target found");
            return;
        }
        double targetX = result.getTx();
        while (result.isValid() && Math.abs(targetX) > ALIGNMENT_TOLERANCE) {
            double turnPower = calculateProportionalTurnPower(targetX);
            if (targetX > ALIGNMENT_TOLERANCE) {
                setDrivePowers(0, 0, turnPower);
            } else if (targetX < -ALIGNMENT_TOLERANCE) {
                setDrivePowers(0, 0, -turnPower);
            }
            result = limelight.getLatestResult();
            if (result.isValid())
                targetX = result.getTx();
            else
                break;
        }
        setDrivePowers(0, 0, 0);
    }

    // --- Shooting System ---
    private void handleShooterControls() {
        if (gamepad1.right_trigger > 0) {
            autoShoot();
        } else if (targetShooterVelocity > 0) {
            // Stop logic on trigger release
            targetShooterVelocity = 0;
            rightShooter.setVelocity(0);
            leftShooter.setVelocity(0);
            intake.setPower(0);
            intakeServo.setPower(0);
        }
    }

    private void autoShoot() {
        if (!result.isValid()) {
            telemetry.addData("Auto-Shoot", "No target found");
            return;
        }
        targetArea = result.getTa();
        double distance = calcDistance.getDistance(targetArea);

        if (distance < DISTANCE_THRESHOLD) {
            targetShooterVelocity = CLOSE_SHOT_VELOCITY;
            outakeServo.setPosition(OUTAKE_POSITION);
        } else if (distance >= DISTANCE_THRESHOLD && distance <= 80) {
            targetShooterVelocity = 1300;
            outakeServo.setPosition(0.56);
        } else {
            targetShooterVelocity = FAR_SHOT_VELOCITY;
            outakeServo.setPosition(OUTAKE_FAR_POSITION);
        }
        rightShooter.setVelocity(targetShooterVelocity);
        leftShooter.setVelocity(targetShooterVelocity);
        telemetry.addData("Auto-Shoot", "Target: %.0f, Dist: %.1f", targetShooterVelocity, distance);
    }
    // --- Intake System ---

    private void handleIntakeControls() {
        // Right Bumper - Intake Forward/Off Toggle (Reverse as per motor setup)
        if (gamepad1.rightBumperWasReleased()) {
            if (intakeStatus != IntakeStatus.intakeOff) {
                intake.setPower(0);
                intakeServo.setPower(0);
                intakeStatus = IntakeStatus.intakeOff;
            } else {
                intake.setPower(-1);
                intakeServo.setPower(0);
                intakeStatus = IntakeStatus.intakeForward;
            }
        }
        // Left Bumper - Intake Reverse/Off Toggle
        if (gamepad1.leftBumperWasReleased()) {
            if (intakeStatus != IntakeStatus.intakeOff) {
                intake.setPower(0);
                intakeServo.setPower(0);
                intakeStatus = IntakeStatus.intakeOff;
            } else {
                intake.setPower(1);
                //intakeServo.setPower(-1);
                intakeStatus = IntakeStatus.intakeForward;
            }
        }

        // Direct Hold Control from User's manual edits
        if (gamepad1.right_bumper) {
            intake.setPower(-1);
        }
        if (gamepad1.leftBumperWasReleased()) { // Note: this was in user's runOpMode
            intake.setPower(1);
        }
    }

    private void updateIntakeState() {
        // Automatic intake/safety behavior based on shooter status
        boolean isShooting = Math.abs(rightShooter.getVelocity()) > 500 || Math.abs(leftShooter.getVelocity()) > 500;

        if (isShooting) {
            if (!wasShooterRunning) {
                shootingDelayTimer.reset();
            }
            wasShooterRunning = true;

            boolean velocityReached = Math.abs(rightShooter.getVelocity()) >= targetShooterVelocity - 225
                    && targetShooterVelocity > 0;

            if (velocityReached) {
                safety.setPosition(0.1194);
                intake.setPower(1);
                if (gamepad1.right_bumper)
                    intake.setPower(0); // Driver override
                intakeServo.setPower(1);
            } else {
                safety.setPosition(0.0194);
                intake.setPower(0);
                intakeServo.setPower(0);
            }
        } else {
            wasShooterRunning = false;
            safety.setPosition(0.0194);

            // Ball jam/clearing logic
            if (intake.getVelocity() < 80 && intakeTimer.milliseconds() > 2000) {
                intake.setPower(0);
            }
            intakeTimer.reset();
        }
    }

    // --- auxiliary Controls (Gamepad 2) ---

    private void handleGamepad2Controls() {
        if (gamepad2.a) {
            rightShooter.setVelocity(0);
            leftShooter.setVelocity(0);
            intake.setPower(0);
            intakeServo.setPower(0);
        }

        if (gamepad2.rightBumperWasReleased()) {
            rightShooter.setVelocity(1500);
            leftShooter.setVelocity(1500);
        }

        if (gamepad2.leftBumperWasReleased()) {
            rightShooter.setVelocity(-1500);
            leftShooter.setVelocity(-1500);
            intake.setPower(-1);
            intakeServo.setPower(-1);
        }

        // Manual override for shooter-triggered intake servo
        if (gamepad2.rightBumperWasReleased() && rightShooter.getVelocity() > CLOSE_SHOT_VELOCITY - 100
                && leftShooter.getVelocity() > CLOSE_SHOT_VELOCITY - 100) {
            intakeServo.setPower(1.0);
        }
    }
}