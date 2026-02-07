package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled

@TeleOp(name = "Parking")
public class goonparking extends LinearOpMode {
    // Hardware Components
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx rightShooter, leftShooter;
    private DcMotor intake;
    private Servo outakeServo;
    private CRServo intakeServo;
    private Servo parkLeft;
    private Servo parkRight;
    private Servo safety;
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
    private boolean shooting = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        telemetry.addData("Status", "Initialized - Ready to Start");
        telemetry.update();
        waitForStart();
        if (isStopRequested())
            return;
        while (opModeIsActive()) {
            park();
            safety();
        }
        // sigma
    }

    /**
     * Initialize all hardware components
     */
    private void initializeHardware() {
        // Drive Motors

        parkLeft = hardwareMap.servo.get("parkLeft");
        parkRight = hardwareMap.servo.get("parkRight");
        safety = hardwareMap.servo.get("safety");

        // Configure Motor Directions

        parkRight.setDirection(Servo.Direction.REVERSE);

        // Configure Encoders
    }
    /**
     * Configure Limelight camera
     */
    /*
     * Update Limelight telemetry data
     */

    /**
     * Handle Gamepad 1 controls (driver controls)
     */

    /**
     * Handle Gamepad 2 controls (operator controls)
     */

    // Intake Reverse (Left Bumper)

    // Auto-Align to Target (Y Button)

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

    private void safety() {

        if (gamepad1.left_bumper == true && shooting == false) {
            shooting = true;
        }
        if (gamepad1.left_bumper == true && shooting == true) {
            shooting = false;
        }

        while (gamepad1.left_bumper) {
            safety.setPosition(0.1194);
        }
        safety.setPosition(0.0194);
    }

}
