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
@TeleOp(name = "Goon TeleOp")
public class goonparking extends LinearOpMode {
    // Hardware Components
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx rightShooter, leftShooter;
    private DcMotor intake;
    private Servo outakeServo;
    private CRServo intakeServo;
    private Servo parkLeft;
    private Servo parkRight;
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
    private boolean parkEngaged = false;
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
            park();
        }
        //sigma
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
        parkLeft = hardwareMap.servo.get("parkLeft");
        parkRight = hardwareMap.servo.get("parkRight");
        // Configure Motor Directions
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rightPark.setDirection(Servo.Direction.REVERSE);
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

        if (gamepad1.right_bumper) {

        }
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

    private void park() {
        if (gamepad1.right_bumper && parkEngaged == false) {
            parkLeft.setPosition(1);
            parkRight.setPosition(1);
            parkEngaged = true;
        }
        if (gamepad1.right_bumper && parkEngaged == true) {
            parkLeft.setPosition(0);
            parkRight.setPosition(0);
            parkEngaged = false;
        }
    }
    
}
