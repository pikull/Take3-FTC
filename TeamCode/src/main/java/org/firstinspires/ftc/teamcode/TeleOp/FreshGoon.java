package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.LLResult;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Fresh Goon Teleop")
public class FreshGoon extends LinearOpMode {
    // hardware components
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx rightShooter, leftShooter;
    private DcMotorEx intake;
    private Servo outakeServo, safety;
    private CRServo intakeServo;
    private Limelight3A limelight;
    private Follower follower;

    // constants
    private static final int FAR_SHOT_VELOCITY = 1700;
    private static final int CLOSE_SHOT_VELOCITY = 1218;
    private static final double OUTAKE_POSITION = 0.488;
    private static final double OUTAKE_FAR_POSITION = 0.5;
    private static final double MIN_TURN_POWER = 0.25;
    private static final double MAX_TURN_POWER = 0.5;
    private static final double DISTANCE_THRESHOLD = 70.0;
    private static final double ALIGNMENT_TOLERANCE = 1.0; // degrees
    private static final double BUTTON_HOLD_THRESHOLD = 0.2; // seconds

    // variables
    private boolean intakePowerToggle = false;
    private double targetArea = 0.0;
    private boolean shooterButtonPressed = false;

    private enum IntakeStatus {
        intakeForward,
        intakeReverse,
        intakeOff
    }

    IntakeStatus intakeStatus = IntakeStatus.intakeOff;

    private boolean isInReverseSequence = false;
    private boolean intervalIncreasing = false;
    private LLResult result;
    private int rightBumperNum;

    // timers
    private ElapsedTime intakeTimer = new ElapsedTime();
    private ElapsedTime shooterButtonTimer = new ElapsedTime();
    private ElapsedTime reverseTimer = new ElapsedTime();
    private ElapsedTime intervalTimer = new ElapsedTime();
    private ElapsedTime shootingDelayTimer = new ElapsedTime();

    private boolean wasShooterRunning = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        configureLimelight();

        telemetry.addData("Status", "Initialized - Ready to Start");
        telemetry.update();

        waitForStart();
        if (isStopRequested())
            return;

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                intake.setPower(-1);
            }
            if (gamepad1.leftBumperWasReleased()) {
                intake.setPower(1);
            }

            updateLimelightTelemetry();
            handleGamepad1Controls(0.7);
            handleGamepad2Controls();
        }
    }

    // initialize hardware components
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

        // Configure Motor Directions
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Configure Encoders
        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        // Set Zero Power Behavior
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Initialize Powers
        intake.setPower(0);
        outakeServo.setPosition(0.76);

        // reset timers
        intakeTimer.reset();
        shooterButtonTimer.reset();
        reverseTimer.reset();
        intervalTimer.reset();
        shootingDelayTimer.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
    }

    // configure limelight settings
    private void configureLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        // Pipeline switching controls
        if (gamepad1.dpad_up) {
            limelight.pipelineSwitch(0);
            new calcDistance(0);
        }
        if (gamepad1.dpad_down) {
            limelight.pipelineSwitch(2);
            new calcDistance(2);
        }
        // Default pipeline
        limelight.start();
    }

    // update limelight telemetry, code for intake servo
    private void updateLimelightTelemetry() {
        result = limelight.getLatestResult();
        LLStatus status = limelight.getStatus();
        telemetry.addData("Limelight Status", status.getName());
        telemetry.addData("Temperature", "%.1f°C", status.getTemp());
        telemetry.addData("CPU Usage", "%.1f%%", status.getCpu());
        telemetry.addData("FPS", "%d", (int) status.getFps());

        // Intake Control Logic - Simplified shooter-based control
        boolean isShooting = Math.abs(rightShooter.getVelocity()) > 500 || Math.abs(leftShooter.getVelocity()) > 500;

        if (isShooting) {
            if (!wasShooterRunning) {
                shootingDelayTimer.reset();
            }
            wasShooterRunning = true;

            handleGamepad1Controls(0.5);

            if (calcDistance.getDistance(targetArea)>80&&(rightShooter.getVelocity() > 1400 && leftShooter.getVelocity() > 1400)) {
                safety.setPosition(.5);
                intake.setPower(1);
                if (gamepad1.right_bumper) {
                    intake.setPower(0);
                }
                intakeServo.setPower(1);
            }
            else if (calcDistance.getDistance(targetArea)>60&&(rightShooter.getVelocity() > 700 && leftShooter.getVelocity() > 700)) {
            } else {
                safety.setPosition(0.2);
                intake.setPower(0);
                intakeServo.setPower(0);
            }
        } else {
            wasShooterRunning = false;
            safety.setPosition(0.2);

            if (intake.getVelocity() < 80 && intakeTimer.milliseconds() > 2000) {
                intake.setPower(0);
            }
            intakeTimer.reset();
        }
        if (result.isValid()) {
            double tx = result.getTx();
            telemetry.addData("Target X Offset", "%.2f", tx);
        } else {
            telemetry.addData("Target", "Not Found");
        }
        telemetry.update();
    }

    // gamepad 1 controls
    private void handleGamepad1Controls(double power) {
        // Mecanum Drive
        double y = -gamepad1.left_stick_y; // Forward/Backward
        double x = gamepad1.left_stick_x; // Strafe Left/Right
        double rx = gamepad1.right_stick_x * power; // Rotation
        // Calculate motor powers
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // Set motor powers
        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);

        // Auto-Align to Target (left trigger)
        if (gamepad1.left_trigger > 0)
            autoAlignToTarget();

        // Auto-Shoot (right trigger)
        if (gamepad1.right_trigger > 0) {
            autoShoot();
            if (gamepad1.right_trigger=0) {
                rightShooter.setVelocity(0);
                leftShooter.setVelocity(0);
                intake.setPower(0);
                intakeServo.setPower (0);
            }
        }
        
        telemetry.addData("inake Stat: ", intakeStatus);
        // intake controls
        if (gamepad1.rightBumperWasReleased()) {
            if (intakeStatus != IntakeStatus.intakeOff) {
                intake.setPower(0);
                intakeServo.setPower (0);
                intakeStatus = IntakeStatus.intakeOff;
            } else {
                intake.setPower(-1);
                intakeServo.setPower (0);
                intakeStatus = IntakeStatus.intakeForward;
            }
        }
        if (gamepad1.leftBumperWasReleased()) {
            if (intakeStatus != IntakeStatus.intakeOff) {
                intake.setPower(0);
                intakeServo.setPower (0);
                intakeStatus = IntakeStatus.intakeOff;
            } else {
                intake.setPower(1);
                intakeServo.setPower (-1);
                intakeStatus = IntakeStatus.intakeForward;
            }
        }
        telemetry.update();
    }

    // gamepad 2 controls
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
    }

    // auto-align to target using limelight
    // with proportional power control
    // power ranges from MIN_TURN_POWER (0.25) to MAX_TURN_POWER (0.5)
    // based on target distance
    private void autoAlignToTarget() {
        LLResult result = limelight.getLatestResult();
        if (!result.isValid()) {
            telemetry.addData("Auto-Align", "No target found");
            return;
        }
        double targetX = result.getTx();
        // Continue aligning while target is not centered (outside tolerance)
        while (result.isValid() && Math.abs(targetX) > ALIGNMENT_TOLERANCE) {
            // Calculate proportional power based on distance from center
            double turnPower = calculateProportionalTurnPower(targetX);
            // Determine turn direction and apply power
            if (targetX > ALIGNMENT_TOLERANCE) {
                // Turn right (target is to the right)
                setDrivePowers(0, 0, turnPower);
                telemetry.addData("Auto-Align", "Turning Right - TX: %.2f, Power: %.2f", targetX, turnPower);
            } else if (targetX < -ALIGNMENT_TOLERANCE) {
                // Turn left (target is to the left)
                setDrivePowers(0, 0, -turnPower);
                telemetry.addData("Auto-Align", "Turning Left - TX: %.2f, Power: %.2f", targetX, turnPower);
            }
            telemetry.addData("Distance from Center", "%.2f degrees", Math.abs(targetX));
            telemetry.update();
            // Get updated result
            result = limelight.getLatestResult();
            if (result.isValid()) {
                targetX = result.getTx();
            } else {
                break; // Exit if we lose the target
            }
        }
        // Stop all motors when aligned
        setDrivePowers(0, 0, 0);
        telemetry.addData("Auto-Align", "Target Centered - TX: %.2f", targetX);
        telemetry.update();
    }

    // auto shoot based on target distance
    private void autoShoot() {
        LLResult result = limelight.getLatestResult();
        if (!result.isValid()) {
            telemetry.addData("Auto-Shoot", "No target found");
            return;
        }
        targetArea = result.getTa();
        double distance = calcDistance.getDistance(targetArea);
        if (distance < DISTANCE_THRESHOLD) {
            // Close shot
            rightShooter.setVelocity(CLOSE_SHOT_VELOCITY);
            leftShooter.setVelocity(CLOSE_SHOT_VELOCITY);
            outakeServo.setPosition(OUTAKE_POSITION);
            telemetry.addData("Auto-Shoot", "Close Shot - Distance: %.1f", distance);
        } else if (distance >= DISTANCE_THRESHOLD && distance <= 80) {
            rightShooter.setVelocity(1000);
            leftShooter.setVelocity(1000);
            outakeServo.setPosition(OUTAKE_POSITION);
            telemetry.addData("Auto-Shoot", "Middle - Distance: %.1f", distance);
        } else {
            // Far shot
            rightShooter.setVelocity(FAR_SHOT_VELOCITY);
            leftShooter.setVelocity(FAR_SHOT_VELOCITY);
            outakeServo.setPosition(OUTAKE_FAR_POSITION);
            telemetry.addData("Auto-Shoot", "Far Shot - Distance: %.1f", distance);
        }
        if (gamepad2.rightBumperWasReleased() && rightShooter.getVelocity() > CLOSE_SHOT_VELOCITY - 100
                && leftShooter.getVelocity() > CLOSE_SHOT_VELOCITY - 100) {
            intakeServo.setPower(1.0);
        }

    }

    // set motor powers for mecanum drive
    private void setDrivePowers(double forward, double strafe, double rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1.1);
        leftFront.setPower((forward + strafe + rotate) / denominator);
        leftBack.setPower((forward - strafe + rotate) / denominator);
        rightFront.setPower((forward - strafe - rotate) / denominator);
        rightBack.setPower((forward + strafe - rotate) / denominator);
    }

    /**
     * Calculate proportional turn power based on distance from target center
     * 
     * @param targetX The X offset from target center in degrees
     * @return Power value between MIN_TURN_POWER and MAX_TURN_POWER
     */
    private double calculateProportionalTurnPower(double targetX) {
        double distanceFromCenter = Math.abs(targetX);
        // Normalize distance to 0-1 range (30 degrees is considered maximum distance)
        double normalizedDistance = Math.min(distanceFromCenter / 30.0, 1.0);
        // Calculate proportional power: closer to center = lower power, further =
        // higher power
        return MIN_TURN_POWER + (normalizedDistance * (MAX_TURN_POWER - MIN_TURN_POWER));
    }
}