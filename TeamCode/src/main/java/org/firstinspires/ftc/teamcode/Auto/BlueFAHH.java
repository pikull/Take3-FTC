package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue FAHHH (can toggle)", group = "Autonomous")
public class BlueFAHH extends OpMode {

    // mirror thing
    private static final boolean mirrored = true;

    // idk what this does
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    // Hardware
    private DcMotorEx rightShooter, leftShooter;
    private DcMotorEx intake;
    private Servo outtakeServo, safety;
    private CRServo intakeServo;

    // Poses
    private final Pose startPose = mirrorPose(new Pose(75.8335724533716, 7.173601147776184, Math.toRadians(90)));
    private final Pose forward15Pose = mirrorPose(
            new Pose(75.8335724533716, 7.173601147776184 + 15, Math.toRadians(90)));
    private final Pose shoot1Pose = mirrorPose(new Pose(75.8335724533716, 12.173601147776184, Math.toRadians(70)));
    private final Pose pickup1Pose = mirrorPose(new Pose(75.8335724533716, 27.173601147776184, Math.toRadians(90)));
    private final Pose pickup2PrePose = mirrorPose(
            new Pose(102.88665710186517 - 10, 35.74175035868005, Math.toRadians(0)));
    private final Pose pickup2Pose = mirrorPose(new Pose(127.88665710186517, 45.74175035868005, Math.toRadians(0)));
    private final Pose pickup3PrePose = mirrorPose(new Pose(140.73601147776185, 35, Math.toRadians(270)));
    private final Pose pickup3Pose = mirrorPose(new Pose(140.73601147776185, 10, Math.toRadians(270)));

    // Paths
    private Path startToForward;
    private Path forwardToShoot;
    private Path shootToPickup1;
    private Path pickup1ToPickup2Pre;
    private Path pickup2PreToPickup2;
    private PathChain scorePickup2, grabPickup3, scorePickup3;

    private Pose mirrorPose(Pose pose) {
        if (!mirrored)
            return pose;

        double mirroredY = 144 - pose.getY();
        double mirroredHeading = Math.PI - pose.getHeading();

        return new Pose(pose.getX(), mirroredY, mirroredHeading);
    }

    public void buildPaths() {
        // 1. Start -> Forward 15
        startToForward = new Path(new BezierLine(startPose, forward15Pose));
        startToForward.setLinearHeadingInterpolation(startPose.getHeading(), forward15Pose.getHeading());

        // 2. Forward -> Return & Rotate (Shoot1)
        forwardToShoot = new Path(new BezierLine(forward15Pose, shoot1Pose));
        forwardToShoot.setLinearHeadingInterpolation(forward15Pose.getHeading(), shoot1Pose.getHeading());

        // 3. Shoot -> Pickup 1
        shootToPickup1 = new Path(new BezierLine(shoot1Pose, pickup1Pose));
        shootToPickup1.setLinearHeadingInterpolation(shoot1Pose.getHeading(), pickup1Pose.getHeading());

        // 4. Pickup 1 -> Pickup 2 Waypoint
        pickup1ToPickup2Pre = new Path(new BezierLine(pickup1Pose, pickup2PrePose));
        pickup1ToPickup2Pre.setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup2PrePose.getHeading());

        // 5. Waypoint -> Pickup 2
        pickup2PreToPickup2 = new Path(new BezierLine(pickup2PrePose, pickup2Pose));
        pickup2PreToPickup2.setLinearHeadingInterpolation(pickup2PrePose.getHeading(), pickup2Pose.getHeading());

        // Pickup 2 -> Score
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, shoot1Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), shoot1Pose.getHeading())
                .build();

        // Score -> Pickup 3
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1Pose, pickup3PrePose))
                .setLinearHeadingInterpolation(shoot1Pose.getHeading(), pickup3PrePose.getHeading())
                .addPath(new BezierLine(pickup3PrePose, pickup3Pose))
                .setLinearHeadingInterpolation(pickup3PrePose.getHeading(), pickup3Pose.getHeading())
                .build();

        // Pickup 3 -> Score
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, pickup3PrePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), pickup3PrePose.getHeading())
                .addPath(new BezierLine(pickup3PrePose, shoot1Pose))
                .setLinearHeadingInterpolation(pickup3PrePose.getHeading(), shoot1Pose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move Forward 15
                intake.setPower(0);
                intakeServo.setPower(0);
                follower.followPath(startToForward);
                setPathState(100);
                break;

            case 100: // Arrive at Forward, return and rotate
                if (!follower.isBusy()) {
                    follower.followPath(forwardToShoot);
                    setPathState(10);
                }
                break;

            case 10: // Arrive at Shoot 1
                if (!follower.isBusy()) {
                    // Start Shooters
                    rightShooter.setVelocity(1600);
                    leftShooter.setVelocity(1600);
                    setPathState(11);
                }
                break;

            case 11: // Wait for velocity, then open safety
                if (leftShooter.getVelocity() > 1450 && rightShooter.getVelocity() > 1450) {
                    safety.setPosition(0.1194);
                    setPathState(110);
                }
                break;

            case 110: // 0.2s delay after safety, then start feed
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    intakeServo.setPower(1);
                    intake.setPower(1);
                    outtakeServo.setPosition(0.5);
                    actionTimer.resetTimer();
                    setPathState(12);
                }
                break;

            case 12: // Wait for shot to clear (2s), keep intake running, then shutdown and move to
                     // pickup 1
                if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                    safety.setPosition(0.0194);
                    rightShooter.setVelocity(0);
                    leftShooter.setVelocity(0);
                    intake.setPower(0);
                    intakeServo.setPower(0);
                    follower.setMaxPower(0.5);
                    follower.followPath(shootToPickup1);
                    setPathState(1);
                }
                break;

            case 1: // Arrive at Pickup 1, start intake and move to Pickup 2 Waypoint
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    intakeServo.setPower(1);
                    actionTimer.resetTimer();
                    follower.followPath(pickup1ToPickup2Pre);
                    setPathState(140);
                }
                break;

            case 140: // Arrive at Pickup 2 Waypoint, move to Pickup 2
                if (!follower.isBusy()) {
                    follower.followPath(pickup2PreToPickup2);
                    setPathState(4);
                }
                break;

            case 4: // Arrive at Pickup 2, move to score
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    intakeServo.setPower(0);
                    follower.setMaxPower(1.0);
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
                break;

            case 5: // Arrive at score, shoot
                if (!follower.isBusy()) {
                    rightShooter.setVelocity(1600);
                    leftShooter.setVelocity(1600);
                    setPathState(14);
                }
                break;

            case 14: // Wait for velocity, then open safety
                if (leftShooter.getVelocity() > 1450 && rightShooter.getVelocity() > 1450) {
                    safety.setPosition(0.1194);
                    setPathState(141);
                }
                break;

            case 141: // 0.2s delay after safety, then start feed
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    intakeServo.setPower(1);
                    intake.setPower(1);
                    outtakeServo.setPosition(0.5);
                    actionTimer.resetTimer();
                    setPathState(15);
                }
                break;

            case 15: // Finish score (2s), keep intake running, then shutdown and move to Pickup 3
                if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                    safety.setPosition(0.0194);
                    rightShooter.setVelocity(0);
                    leftShooter.setVelocity(0);
                    intake.setPower(0);
                    intakeServo.setPower(0);
                    follower.setMaxPower(0.5);
                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                }
                break;

            case 6: // Arrive at Pickup 3, move to score
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    intakeServo.setPower(0);
                    follower.setMaxPower(1.0);
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;

            case 7: // Final Score
                if (!follower.isBusy()) {
                    rightShooter.setVelocity(1600);
                    leftShooter.setVelocity(1600);
                    setPathState(16);
                }
                break;

            case 16: // Wait for velocity, then open safety
                if (leftShooter.getVelocity() > 1450 && rightShooter.getVelocity() > 1450) {
                    safety.setPosition(0.1194);
                    setPathState(161);
                }
                break;

            case 161: // 0.2s delay after safety, then start feed
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    intakeServo.setPower(1);
                    intake.setPower(1);
                    outtakeServo.setPosition(0.5);
                    actionTimer.resetTimer();
                    setPathState(17);
                }
                break;

            case 17: // End Auto (2s), keep intake running, then shutdown
                if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                    safety.setPosition(0.0194);
                    rightShooter.setVelocity(0);
                    leftShooter.setVelocity(0);
                    intake.setPower(1);
                    intakeServo.setPower(0);
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("leftShooter velocity", leftShooter.getVelocity());
        telemetry.addData("rightShooter velocity", rightShooter.getVelocity());
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Initialize Hardware
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        outtakeServo = hardwareMap.get(Servo.class, "outakeS");
        intakeServo = hardwareMap.get(CRServo.class, "intakeS");
        outtakeServo.setPosition(0.7);

        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        safety = hardwareMap.get(Servo.class, "safety");
        safety.setPosition(0.0194);

        // Directions
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}
