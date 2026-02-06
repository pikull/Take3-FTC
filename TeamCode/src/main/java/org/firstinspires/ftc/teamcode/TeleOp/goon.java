package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp(name = "Goon Teleop")
public class goon extends LinearOpMode {
    // Hardware
    // Components//////////////////////////////////////////////////////////////////////////

    private ElapsedTime intakeTimer, shooterButtonTimer;
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx rightShooter, leftShooter;
    private DcMotorEx intake;
    private Servo outtakeServo, safety;
    private CRServo intakeServo;
    private Limelight3A limelight;
    private Follower follower;
    // Constants////////////////////////////////////////////////////////////////////////////////////
    private static final int FAR_SHOT_VELOCITY = 1500;
    private static final int CLOSE_SHOT_VELOCITY = 1118;
    private static final double OUTTAKE_POSITION = 0.288;
    private static final double OUTTAKE_FAR_POSITION = 0.27;
    private static final double MIN_TURN_POWER = 0.25;
    private static final double MAX_TURN_POWER = 0.5;
    private static final double DISTANCE_THRESHOLD = 70.0;
    private static final double ALIGNMENT_TOLERANCE = 1.0; // degrees

    // Variables
    private boolean intakeOn = false;
    private LLResult result;

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
            updateLimelightTelemetry();
            handleGamepad1Controls();
            handleGamepad2Controls();

            telemetry.update();
        }
    }

    /**
     * Initialize all hardware components
     */
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
        outtakeServo = hardwareMap.servo.get("outakeS");
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

        // Set Zero Power Behavior
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize States
        intake.setPower(0);
        outtakeServo.setPosition(0.76);

        intakeTimer = new ElapsedTime();
        shooterButtonTimer = new ElapsedTime();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
    }

    /**
     * CONFIGURE
     * LIMELIGHT//////////////////////////////////////////////////////////////////////////
     */
    private void configureLimelight() {
        // Pipeline switching controls
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
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

    /**
     * UPDATE LIMELIGHT TELEMETRY AND CODE FOR THE INTAKE
     * SERVO/////////////////////////////////////
     */
    private void updateLimelightTelemetry() {
        result = limelight.getLatestResult();
        LLStatus status = limelight.getStatus();

        telemetry.addData("Limelight", "%s | %.1f°C | %.1f%% CPU | %d FPS",
                status.getName(), status.getTemp(), status.getCpu(), (int) status.getFps());

        if (result != null && result.isValid()) {
            telemetry.addData("Target", "TX: %.2f", result.getTx());
        } else {
            telemetry.addData("Target", "Not Found");
        }
    }

    /**
     * GAMEPAD 1
     * CONTORLS///////////////////////////////////////////////////////////////////////////
     */
    private void handleGamepad1Controls() {
        // Drive Control
        boolean isShooting = Math.abs(rightShooter.getVelocity()) > 500 || Math.abs(leftShooter.getVelocity()) > 500;
        double drivePower = isShooting ? 0.5 : 0.7;

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x * drivePower;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        leftFront.setPower((y + x + rx) / denominator);
        leftBack.setPower((y - x + rx) / denominator);
        rightFront.setPower((y - x - rx) / denominator);
        rightBack.setPower((y + x - rx) / denominator);

        // Auto-Align (Left Trigger)
        if (gamepad1.left_trigger > 0) {
            autoAlignToTarget();
        }

        // Intake Controls (Bumpers Override)
        if (gamepad1.right_bumper) {
            intake.setPower(-1.0);
            intakeServo.setPower(-1.0);
        } else if (gamepad1.left_bumper) {
            intake.setPower(1.0);
            intakeServo.setPower(1.0);
        } else {
            // Logic based on shooter status
            if (isShooting && !gamepad1.right_bumper) {
                safety.setPosition(0.5);
                intake.setPower(1.0);
                intakeServo.setPower(1.0);
            } else {
                safety.setPosition(0.1);
                // Default: spin intake servo negative
                intakeServo.setPower(-0.2);

                if (intakeTimer.seconds() > 2.0) {
                    intake.setPower(0);
                } else {
                    intakeTimer.reset();
                }
            }
        }

        // Auto-Shoot (Right Trigger)
        if (gamepad1.right_trigger > 0) {
            autoShoot();
        }
    }

    /**
     * GAMEPAD 2
     * CONTROLS////////////////////////////////////////////////////////////////////////////
     */
    private void handleGamepad2Controls() {
        // Stop Everything
        if (gamepad2.a) {
            rightShooter.setVelocity(0);
            leftShooter.setVelocity(0);
            intakeServo.setPower(0);
            intake.setVelocity(0);
        }

        // Shooter Forward (Right Bumper)
        if (gamepad2.right_bumper) {
            rightShooter.setVelocity(1500);
            leftShooter.setVelocity(1500);
        }

        // Shooter/Intake Reverse (Left Bumper)
        if (gamepad2.left_bumper) {
            rightShooter.setVelocity(-1500);
            leftShooter.setVelocity(-1500);
            intakeServo.setPower(-1);
            intake.setVelocity(-1500);
        }
    }

    /**
     * Auto-align robot to target using Limelight with proportional power control
     * Power ranges from MIN_TURN_POWER (0.25) to MAX_TURN_POWER (0.5) based on
     * distance from target
     */
    private void autoAlignToTarget() {
        if (result == null || !result.isValid()) {
            telemetry.addData("Auto-Align", "No target found");
            return;
        }

        double targetX = result.getTx();
        if (Math.abs(targetX) > ALIGNMENT_TOLERANCE) {
            double turnPower = calculateProportionalTurnPower(targetX);
            // Apply turn power to the driveMotors via a simplified rotate call
            // Using setDrivePowers(forward, strafe, rotate)
            double rotate = (targetX > 0) ? turnPower : -turnPower;
            setDrivePowers(0, 0, rotate);
            telemetry.addData("Auto-Align", "Aligning... TX: %.2f", targetX);
        } else {
            telemetry.addData("Auto-Align", "Aligned");
        }
    }

    /**
     * Auto-shoot based on distance to target
     */
    private void autoShoot() {
        if (result == null || !result.isValid()) {
            telemetry.addData("Auto-Shoot", "No target found");
            return;
        }

        double distance = calcDistance.getDistance(result.getTa());

        if (distance < DISTANCE_THRESHOLD) {
            // Close shot
            rightShooter.setVelocity(CLOSE_SHOT_VELOCITY);
            leftShooter.setVelocity(CLOSE_SHOT_VELOCITY);
            outtakeServo.setPosition(OUTTAKE_POSITION);
            telemetry.addData("Auto-Shoot", "Close | Dist: %.1f", distance);
        } else if (distance <= 80) {
            // Middle shot
            rightShooter.setVelocity(1000);
            leftShooter.setVelocity(1000);
            outtakeServo.setPosition(OUTTAKE_POSITION);
            telemetry.addData("Auto-Shoot", "Middle | Dist: %.1f", distance);
        } else {
            // Far shot
            rightShooter.setVelocity(FAR_SHOT_VELOCITY);
            leftShooter.setVelocity(FAR_SHOT_VELOCITY);
            outtakeServo.setPosition(OUTTAKE_FAR_POSITION);
            telemetry.addData("Auto-Shoot", "Far | Dist: %.1f", distance);
        }

        // Automatic feed if at speed
        if (rightShooter.getVelocity() > 900) {
            intakeServo.setPower(1.0);
        }
    }

    /**
     * Set drive motor powers for mecanum drive
     */
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
