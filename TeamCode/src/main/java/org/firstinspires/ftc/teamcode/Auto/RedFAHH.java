package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red FAHH")
public class RedFAHH extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer;
    private ElapsedTime opmodeTimer;
    private int pathState;
    private boolean isHardStopActive = false;

    // Hardware
    private DcMotorEx rightShooter, leftShooter;
    private DcMotor intake;
    private Servo outake, safety;
    private CRServo intakeS;

    // Poses
    private final Pose startPose = new Pose(80, 8, Math.toRadians(270));
    private final Pose pickup1Pose = new Pose(126.54782608695655, 28.391304347826104, Math.toRadians(270));
    private final Pose pickup2Pose = new Pose(127.9391304347826, 7.513043478260874, Math.toRadians(270));
    // 5 inches backward in Y (270 heading means -Y is forward, so +Y is backward)
    private final Pose pickup2BackPose = new Pose(127.9391304347826, 12.513043478260874, Math.toRadians(270));
    private final Pose secondPrePickupPose = new Pose(88, 35, Math.toRadians(0)); // chnage this to be closer to balls
    private final Pose secondPickupPose = new Pose(120, 35, Math.toRadians(0));

    // Paths
    private Path startToPickup1;
    private Path pickup1To2;
    private Path pickup2Back;
    private Path pickup2To1;
    private Path shootToSecondPrePickup;
    private Path secondPrePickupToSecondPickup;
    private Path secondPickupTostart;

    public void buildPaths() {
        // Start -> Pickup 1
        startToPickup1 = new Path(new BezierLine(startPose, pickup1Pose));
        startToPickup1.setLinearHeadingInterpolation(startPose.getHeading(), pickup1Pose.getHeading());

        // Pickup 1 -> Pickup 2
        pickup1To2 = new Path(new BezierLine(pickup1Pose, pickup2Pose));
        pickup1To2.setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup2Pose.getHeading());

        // Pickup 2 -> Pickup 2 Back
        pickup2Back = new Path(new BezierLine(pickup2Pose, pickup2BackPose));
        pickup2Back.setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2BackPose.getHeading());

        // Pickup 2 Back -> Pickup 1
        pickup2To1 = new Path(new BezierLine(pickup2BackPose, new Pose(65, 8)));
        pickup2To1.setLinearHeadingInterpolation(pickup2BackPose.getHeading(), Math.toRadians(70));

        //
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Move to Pickup 1
                follower.followPath(startToPickup1);
                setPathState(1);
                break;
            case 1:
                // Arrive at Pickup 1
                if (!follower.isBusy()) {
                    // Start Intake
                    intake.setPower(1);
                    actionTimer.resetTimer();

                    // Lower velocity for Pickup 2
                    follower.setMaxPower(0.5);

                    // Move to Pickup 2
                    follower.followPath(pickup1To2);
                    setPathState(2);
                }
                break;
            case 2:
                // Arrive at Pickup 2
                if (!follower.isBusy()) {
                    // Reset velocity to full power
                    follower.setMaxPower(1.0);

                    // Backup
                    follower.followPath(pickup2Back);
                    setPathState(3);
                }
                break;
            case 3:
                // Finished Backup
                if (!follower.isBusy()) {
                    // Loop back to Pickup 1
                    follower.followPath(pickup2To1);
                    rightShooter.setVelocity(1500);
                    leftShooter.setVelocity(1500);
                    setPathState(4);
                }
                break;
            case 4:
                // Arrive back at Pickup 1
                if (!follower.isBusy()) {
                    // Start Shooters


                    // Move safety servo out of the way
                    safety.setPosition(0.5);

                    // Reset timer so we wait starting from the moment we arrive

                    intakeS.setPower(1);
                    intake.setPower(1);
                    outake.setPosition(0.67);
                    setPathState(-1);

                }
                break;



                    // Stop
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        // Hard Stop Logic
        if (opmodeTimer.seconds() > 27 && !isHardStopActive) {
            isHardStopActive = true;

            // Stop all mechanisms
            if (intake != null)
                intake.setPower(0);
            if (intakeS != null)
                intakeS.setPower(0);
            if (rightShooter != null)
                rightShooter.setVelocity(0);
            if (leftShooter != null)
                leftShooter.setVelocity(0);

            // Calculate forward pose (10 inches approx)
            Pose currentPose = follower.getPose();
            double heading = currentPose.getHeading();
            double forwardDistance = 10.0;

            double targetX = currentPose.getX() + forwardDistance * Math.cos(heading);
            double targetY = currentPose.getY() + forwardDistance * Math.sin(heading);

            Pose targetPose = new Pose(targetX, targetY, heading);

            // Create and follow path
            Path hardStopPath = new Path(new BezierLine(currentPose, targetPose));
            hardStopPath.setLinearHeadingInterpolation(heading, heading);
            follower.followPath(hardStopPath, true);
        }

        follower.update();

        if (!isHardStopActive) {
            autonomousPathUpdate();

            // Intake timeout (runs for only 3 seconds)
            if (actionTimer.getElapsedTimeSeconds() > 3) {
                intake.setPower(0);
                intakeS.setPower(0);
            }
        }

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Time", opmodeTimer.seconds());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new ElapsedTime();
        opmodeTimer.reset();

        // Initialize Hardware (Referencing RedClose, minus Safety)
        intake = hardwareMap.get(DcMotor.class, "intake");
        outake = hardwareMap.get(Servo.class, "outakeS");
        intakeS = hardwareMap.get(CRServo.class, "intakeS");
        outake.setPosition(0.7);
        // Safety Servo Removed
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        safety = hardwareMap.get(Servo.class, "safety");
        safety.setPosition(0.2);

        // Directions
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        opmodeTimer.reset();
        setPathState(0);
    }
}