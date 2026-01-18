package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Teleop.calcDistance;

@TeleOp(name = "Goon TeleOp")
public class goon extends LinearOpMode {

    // Hardware Components
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx rightShooter, leftShooter;
    private DcMotor intake;
    private Servo outakeServo;
    private CRServo intakeServo;
    private Limelight3A limelight;

    // Constants
    private static final int FAR_SHOT_VELOCITY = 1500;
    private static final int CLOSE_SHOT_VELOCITY = 1118;
    private static final double OUTAKE_POSITION = 0.188;
    private static final double OUTAKE_FAR_POSITION = 0.17;
    private static final double MIN_TURN_POWER = 0.25;
    private static final double MAX_TURN_POWER = 0.5;
    private static final double DISTANCE_THRESHOLD = 80.0;
    private static final double ALIGNMENT_TOLERANCE = 1.0; // degrees

    // Variables
    private boolean intakePowerToggle = false;
    private double targetArea = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        configureLimelight();

        telemetry.addData("Status", "Initialized - Ready to Start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            updateLimelightTelemetry();
            handleGamepad2Controls();
            handleGamepad1Controls();
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
        intake = hardwareMap.dcMotor.get("intake");
        outakeServo = hardwareMap.servo.get("outakeS");
        intakeServo = hardwareMap.get(CRServo.class, "intakeS");
        // Configure Motor Directions
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        // Configure Encoders
        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Set Zero Power Behavior
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Initialize Powers
        intake.setPower(0);
        outakeServo.setPosition(0.65);
    }
    /**
     * Configure Limelight camera
     */
    private void configureLimelight() {
        // Pipeline switching controls
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        if(gamepad1.dpad_up){
            limelight.pipelineSwitch(0);
            new calcDistance(0);
        }
        if(gamepad1.dpad_down){
            limelight.pipelineSwitch(2);
            new calcDistance(2);
        }
        // Default pipeline
        limelight.start();

    }
    /**
     * Update Limelight telemetry data
     */
    private void updateLimelightTelemetry() {
        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();
        telemetry.addData("Limelight Status", status.getName());
        telemetry.addData("Temperature", "%.1f°C", status.getTemp());
        telemetry.addData("CPU Usage", "%.1f%%", status.getCpu());
        telemetry.addData("FPS", "%d", (int) status.getFps());

        if (result.isValid()) {
            double tx = result.getTx();
            telemetry.addData("Target X Offset", "%.2f", tx);
        } else {
            telemetry.addData("Target", "Not Found");
        }
        telemetry.update();
    }
    /**
     * Handle Gamepad 1 controls (driver controls)
     */
    private void handleGamepad1Controls() {



        // Mecanum Drive
        double y = -gamepad1.left_stick_y;  // Forward/Backward
        double x = gamepad1.left_stick_x;   // Strafe Left/Right
        double rx = gamepad1.right_stick_x*0.7; // Rotation
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
    }
    /**
     * Handle Gamepad 2 controls (operator controls)
     */
    private void handleGamepad2Controls() {
        // Intake Toggle (Right Bumper)
        if (gamepad2.right_bumper) {
            if (!intakePowerToggle) {
                intake.setPower(1.0);
                intakePowerToggle = true;
            } else {
                intake.setPower(0);
                intakePowerToggle = false;
            }
        }
        // Intake Reverse (Left Bumper)
        if (gamepad2.left_bumper) {
            intake.setPower(-1.0);
        }
        // Auto-Align to Target (Y Button)
        if (gamepad2.y) {
            autoAlignToTarget();
        }
        // Auto-Shoot (X Button)
        if (gamepad2.x) {
            autoShoot();
        }
        if(gamepad2.a){
            rightShooter.setVelocity(0);
            leftShooter.setVelocity(0);
            intakeServo.setPower(-0.2);
            intake.setPower(0);
        }
    }
    /**
     * Auto-align robot to target using Limelight with proportional power control
     * Power ranges from MIN_TURN_POWER (0.25) to MAX_TURN_POWER (0.5) based on distance from target
     */
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
    /**
     * Auto-shoot based on distance to target
     */
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
        } else {
            // Far shot
            rightShooter.setVelocity(FAR_SHOT_VELOCITY);
            leftShooter.setVelocity(FAR_SHOT_VELOCITY);
            outakeServo.setPosition(OUTAKE_FAR_POSITION);
            telemetry.addData("Auto-Shoot", "Far Shot - Distance: %.1f", distance);
        }
        intakeServo.setPower(1.0);
    }
    /**
     * Set drive motor powers for mecanum drive
     */
    private void setDrivePowers(double forward, double strafe, double rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
        leftFront.setPower((forward + strafe + rotate) / denominator);
        leftBack.setPower((forward - strafe + rotate) / denominator);
        rightFront.setPower((forward - strafe - rotate) / denominator);
        rightBack.setPower((forward + strafe - rotate) / denominator);
    }

    /**
     * Calculate proportional turn power based on distance from target center
     * @param targetX The X offset from target center in degrees
     * @return Power value between MIN_TURN_POWER and MAX_TURN_POWER
     */
    private double calculateProportionalTurnPower(double targetX) {
        double distanceFromCenter = Math.abs(targetX);
        // Normalize distance to 0-1 range (30 degrees is considered maximum distance)
        double normalizedDistance = Math.min(distanceFromCenter / 30.0, 1.0);
        // Calculate proportional power: closer to center = lower power, further = higher power
        return MIN_TURN_POWER + (normalizedDistance * (MAX_TURN_POWER - MIN_TURN_POWER));
    }
}