package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled

@TeleOp(name = "Auto Turn Limelight")
public class autoturn extends OpMode {
    private Follower follower;
    private Limelight3A limelight;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private DcMotor leftFront, leftBack, rightFront, rightBack;

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Initialize motors to satisfy Constants requirements if needed,
        // though Constants.createFollower handles hardwareMap.get
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0); // Default pipeline
        limelight.start();

        telemetry.addData("Status", "Initialized");
    }

    /**
     * This method is called once at the start of the OpMode.
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking
     * "Play".
     **/
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Telemetry
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            telemetry.addData("Limelight TX", result.getTx());
        } else {
            telemetry.addData("Limelight", "No Target");
        }

        telemetry.update();
    }

    /**
     * State machine for auto-turn logic
     **/
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Idle
                if (gamepad1.a) {
                    setPathState(1);
                }
                break;

            case 1: // Calculate and start turn
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    double tx = result.getTx();
                    Pose currentPose = follower.getPose();

                    // Calculate target heading relative to current heading
                    // Limelight tx is offset in degrees. Pedro Pathing uses radians.
                    double targetHeading = currentPose.getHeading() - Math.toRadians(tx);

                    // Create a path that stays at the same position but turns
                    Path turnPath = new Path(new BezierLine(currentPose, currentPose));
                    turnPath.setLinearHeadingInterpolation(currentPose.getHeading(), targetHeading);

                    follower.followPath(turnPath, true);
                    setPathState(2);
                } else {
                    // No target found, stay in Idle
                    setPathState(0);
                }
                break;

            case 2: // Turning
                if (!follower.isBusy()) {
                    setPathState(0);
                }
                break;
        }
    }

    /**
     * Updates the path state and resets the timer
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
