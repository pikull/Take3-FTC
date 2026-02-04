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

@TeleOp(name = "Goon Teleop")
public class goon extends LinearOpMode {
    // Hardware
    // Components//////////////////////////////////////////////////////////////////////////

    ElapsedTime intakeTimer;
    ElapsedTime shooterButtonTimer;
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx rightShooter, leftShooter;
    private DcMotorEx intake;
    private Servo outakeServo, safety;// parkRight,parkLeft;
    private CRServo intakeServo;
    private Limelight3A limelight;
    private Follower follower;
    // Constants////////////////////////////////////////////////////////////////////////////////////
    private static final int FAR_SHOT_VELOCITY = 1500;
    private static final int CLOSE_SHOT_VELOCITY = 1118;
    private static final double OUTAKE_POSITION = 0.288;
    private static final double OUTAKE_FAR_POSITION = 0.27;
    private static final double MIN_TURN_POWER = 0.25;
    private static final double MAX_TURN_POWER = 0.5;
    private static final double DISTANCE_THRESHOLD = 70.0;
    private static final double ALIGNMENT_TOLERANCE = 1.0; // degrees
    private static final double BUTTON_HOLD_THRESHOLD = 0.2; // seconds
    // Variables////////////////////////////////////////////////////////////////////////////////////
    private boolean intakePowerToggle = false;
    private double targetArea = 0;
    // private double previousDistance = 100.0;
    // private double peakDistance = 0.0;
    // private boolean isReverseMode = false;
    private boolean shooterButtonPressed = false;
    private boolean intakeOn = false;
    // New variables for simplified control
    private boolean isInReverseSequence = false;
    private ElapsedTime reverseTimer;
    private boolean intervalIncreasing = false;
    private ElapsedTime intervalTimer;
    private LLResult result;
    private int rightBumperNum;

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

            // Intake Reverse (Left Bumper)
            //if (gamepad1.leftBumperWasReleased())
            //    intake.setPower(1.0);
            // if(gamepad2.dpad_up){
            // parkRight.setPosition(1);
            // parkLeft.setPosition(1);
            // }
            // if(gamepad2.dpad_down){
            // parkRight.setPosition(0);
            // parkLeft.setPosition(0);
            // }
            updateLimelightTelemetry();
            handleGamepad2Controls();
            handleGamepad1Controls(0.7);
            // intakeServo.setPower(0); // REMOVED to allow logic in
            // updateLimelightTelemetry to control servo
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
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        safety = hardwareMap.servo.get("safety");
        outakeServo = hardwareMap.servo.get("outakeS");
        intakeServo = hardwareMap.get(CRServo.class, "intakeS");
        // Park Servos
        // parkRight = hardwareMap.servo.get("parkRight");
        // parkLeft = hardwareMap.servo.get("parkLeft");
        // Configure Motor Directions
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // parkRight.setDirection(Servo.Direction.REVERSE);
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

        intakeTimer = new ElapsedTime();
        intakeTimer.reset();
        shooterButtonTimer = new ElapsedTime();
        shooterButtonTimer.reset();
        reverseTimer = new ElapsedTime();
        reverseTimer.reset();
        intervalTimer = new ElapsedTime();
        intervalTimer.reset();
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
        telemetry.addData("Limelight Status", status.getName());
        telemetry.addData("Temperature", "%.1f°C", status.getTemp());
        telemetry.addData("CPU Usage", "%.1f%%", status.getCpu());
        telemetry.addData("FPS", "%d", (int) status.getFps());

        // Intake Control Logic - Simplified shooter-based control
        boolean isShooting = Math.abs(rightShooter.getVelocity()) > 500 || Math.abs(leftShooter.getVelocity()) > 500;

        if (isShooting) {
            handleGamepad1Controls(0.5);
            safety.setPosition(.5);
            intake.setPower(1);
        } else {
            safety.setPosition(0.2);

            if(intake.getVelocity()<80&&intakeTimer.milliseconds()>2000) {
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

    /**
     * GAMEPAD 1
     * CONTORLS///////////////////////////////////////////////////////////////////////////
     */
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

        // Auto-Align to Target (Y Button)
        if (gamepad1.left_trigger>0)
            autoAlignToTarget();
        // if(gamepad2.y){
        // if(result.isValid()&&result.getTx()!=0){
        // gamepad2.rumble(100);
        // }
        // }

        // Auto-Shoot (X Button)
        if (gamepad1.right_trigger>0) {
            autoShoot();
        }

        if (gamepad1.leftBumperWasReleased()) {
            intakeOn = !intakeOn;
            intake.setPower(intakeOn ? -1 : 0);
        }


    }

    /**
     * GAMEPAD 2
     * CONTROLS////////////////////////////////////////////////////////////////////////////
     */
    private void handleGamepad2Controls() {
        // Right Bumper - Shooter with Time-Based Control
        //if (gamepad2.right_bumper) {
        //    if (!shooterButtonPressed) {
        //        // Button just pressed - start timer
        //        shooterButtonPressed = true;
        //        shooterButtonTimer.reset();
        //    }
        //}

        

        if (gamepad2.a) {
            rightShooter.setVelocity(0);
            leftShooter.setVelocity(0);
            intake.setPower(0);
        }
        
        if (gamepad2.rightBumperWasReleased()) {
            rightShooter.setVelocity(1500);
            leftShooter.setVelocity(1500);
        }

        if (gamepad2.leftBumperWasReleased()&&(rightShooter.getVelocity()>500&&leftShooter.getVelocity()>500)) {
            rightShooter.setVelocity(-1500);
            leftShooter.setVelocity(-1500);
            intake.setPower(-1);
        }

        
    }

    /**
     * Auto-align robot to target using Limelight with proportional power control
     * Power ranges from MIN_TURN_POWER (0.25) to MAX_TURN_POWER (0.5) based on
     * distance from target
     */
    private void autoAlignToTarget() {
        LLResult result = limelight.getLatestResult();
        if (!result.isValid()) {
            telemetry.addData("Auto-Align", "No target found");
            return;
        }
        double targetX = result.getTx();
        // Continue aligning while target is not centered (outside tolerance)
        if (result.isValid() && Math.abs(targetX) > ALIGNMENT_TOLERANCE && left_trigger > 0) {
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
