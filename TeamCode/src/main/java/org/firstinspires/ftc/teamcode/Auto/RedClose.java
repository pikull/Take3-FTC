package org.firstinspires.ftc.teamcode.Auto; // make sure this aligns with class location

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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Red Close")
public class RedClose extends OpMode {
    private CRServo intakeServo;
    private Servo outtakeServo, safety;
    private DcMotorEx rightShooter, leftShooter;
    private DcMotorEx intake;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(118.56521739130434, 118.35652173913039, Math.toRadians(45)); // Start Pose//
                                                                                                         // of our//
                                                                                                         // robot.
    private final Pose scorePose = new Pose(92.93913043478265, 86, Math.toRadians(48)); // Scoring Pose
    private final Pose scoreShakePose = new Pose(92.93913043478265 - 3, 86 - 3, Math.toRadians(45)); // Shake// Pose
    private final Pose pickup1Pose = new Pose(92.4521739130435, 84.62608695652175, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(92.4521739130435, 63, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(92.4521739130435, 40, Math.toRadians(0));
    private final Pose moveAfterPickup1Pose = new Pose(103.54782608695652 + 20, 84.62608695652175, Math.toRadians(0));
    private final Pose moveAfterPickup2Pose = new Pose(100.86956521739131 + 20, 58.95652173913044, Math.toRadians(0));
    private final Pose moveAfterPickup3Pose = new Pose(105.25217391304349 + 20, 36.208695652173915, Math.toRadians(0));
    private Path scorePreload;
    private PathChain grabPickup1, moveAfterPickup1, scorePickup1, grabPickup2, moveAfterPickup2, scorePickup2,
            grabPickup3, moveAfterPickup3, scorePickup3, scoreToShake, shakeToScore, park;

    private final Pose parkPose = new Pose(scorePose.getX(), scorePose.getY() - 15.4, Math.toRadians(48));

    // PIDF Coefficients
    public static double SHOOTER_P = 60.0;
    public static double SHOOTER_I = 20.0;
    public static double SHOOTER_D = 0.0;
    public static double SHOOTER_F = 14.0;

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        follower = Constants.createFollower(hardwareMap);
        outtakeServo = hardwareMap.get(Servo.class, "outakeS");
        intakeServo = hardwareMap.get(CRServo.class, "intakeS");
        safety = hardwareMap.get(Servo.class, "safety");

        outtakeServo.setPosition(0.7);
        safety.setPosition(0.0194);

        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");

        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDFCoefficients pidf = new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    public void buildPaths() {
        /*
         * This is our scorePreload path. We are using a BezierLine, which is a straight
         * line.
         */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        /*
         * Here is an example for Constant Interpolation
         * scorePreload.setConstantInterpolation(startPose.getHeading());
         */
        /*
         * This is our grabPickup1 PathChain. We are using a single path with a
         * BezierLine, which is a straight line.
         */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();
        /* Backup after pickup 1 */
        moveAfterPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, moveAfterPickup1Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), moveAfterPickup1Pose.getHeading())
                .build();
        /*
         * This is our scorePickup1 PathChain. We are using a single path with a
         * BezierLine, which is a straight line.
         */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(moveAfterPickup1Pose, scorePose))
                .setLinearHeadingInterpolation(moveAfterPickup1Pose.getHeading(), scorePose.getHeading())
                .build();
        /*
         * This is our grabPickup2 PathChain. We are using a single path with a
         * BezierLine, which is a straight line.
         */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();
        /* Backup after pickup 2 */
        moveAfterPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, moveAfterPickup2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), moveAfterPickup2Pose.getHeading())
                .build();
        /*
         * This is our scorePickup2 PathChain. We are using a single path with a
         * BezierLine, which is a straight line.
         */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(moveAfterPickup2Pose, scorePose))
                .setLinearHeadingInterpolation(moveAfterPickup2Pose.getHeading(), scorePose.getHeading())
                .build();
        /*
         * This is our grabPickup3 PathChain. We are using a single path with a
         * BezierLine, which is a straight line.
         */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();
        /* Backup after pickup 3 */
        moveAfterPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, moveAfterPickup3Pose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), moveAfterPickup3Pose.getHeading())
                .build();

        /*
         * This is our scorePickup3 PathChain. We are using a single path with a
         * BezierLine, which is a straight line.
         */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(moveAfterPickup3Pose, scorePose))
                .setLinearHeadingInterpolation(moveAfterPickup3Pose.getHeading(), scorePose.getHeading())
                .build();
        /* Shake Paths */
        scoreToShake = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, scoreShakePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), scoreShakePose.getHeading())
                .build();
        shakeToScore = follower.pathBuilder()
                .addPath(new BezierLine(scoreShakePose, scorePose))
                .setLinearHeadingInterpolation(scoreShakePose.getHeading(), scorePose.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move to score preload
                follower.followPath(scorePreload);
                intake.setPower(1);
                rightShooter.setVelocity(1175);
                leftShooter.setVelocity(1175);
                outtakeServo.setPosition(0.5);
                setPathState(1);
                break;

            case 1: // Arrived at score preload
                if (!follower.isBusy()) {
                    setPathState(11);
                }
                break;

            case 11: // Verify velocity before feed
                if (rightShooter.getVelocity() > 1175) {
                    safety.setPosition(0.1194);
                    setPathState(110);
                }
                break;

            case 110: // Safety open delay

                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    intakeServo.setPower(1);
                    intake.setPower(1);
                    setPathState(12);
                }
                break;

            case 12: // Feed complete, move to pickup 1
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    safety.setPosition(0.0194);
                    intakeServo.setPower(0);
                    intake.setPower(0);
                    rightShooter.setVelocity(0);
                    leftShooter.setVelocity(0);
                    follower.followPath(grabPickup1, true);
                    follower.setMaxPower(1);
                    setPathState(2);
                }
                break;

            case 2: // Arrived at pickup 1, grab and move for backup
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.7);
                    follower.followPath(moveAfterPickup1, true);

                    intake.setPower(1);
                    setPathState(3);
                }
                break;

            case 3: // Backup complete, move to score
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup1, true);
                    rightShooter.setVelocity(1175);
                    leftShooter.setVelocity(1175);
                    outtakeServo.setPosition(0.5);
                    setPathState(4);
                }
                break;

            case 4: // Arrived at score
                if (!follower.isBusy()) {
                    setPathState(13);
                }
                break;

            case 13: // Verify velocity before feed
                if (rightShooter.getVelocity() > 1175) {
                    safety.setPosition(0.1194);
                    setPathState(130);
                }
                break;

            case 130: // Safety open delay

                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    intakeServo.setPower(1);
                    intake.setPower(1);
                    setPathState(14);
                }
                break;

            case 14: // Feed complete, move to pickup 2
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    safety.setPosition(0.0194);
                    intakeServo.setPower(0);
                    intake.setPower(0);
                    rightShooter.setVelocity(0);
                    leftShooter.setVelocity(0);
                    follower.followPath(grabPickup2, true);
                    follower.setMaxPower(1);
                    setPathState(5);
                }
                break;

            case 5: // Arrived at pickup 2, grab and move for backup
                if (!follower.isBusy()) {
                    follower.followPath(moveAfterPickup2, true);
                    follower.setMaxPower(0.7);
                    intake.setPower(1);
                    setPathState(6);
                }
                break;

            case 6: // Backup complete, move to score
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup2, true);
                    rightShooter.setVelocity(1175);
                    leftShooter.setVelocity(1175);
                    outtakeServo.setPosition(0.6);
                    setPathState(7);
                }
                break;

            case 7: // Arrived at score
                if (!follower.isBusy()) {
                    setPathState(16);
                }
                break;

            case 16: // Verify velocity before feed
                if (rightShooter.getVelocity() > 1175) {
                    safety.setPosition(0.1194);
                    setPathState(160);
                }
                break;

            case 160: // Safety open delay

                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    intakeServo.setPower(1);
                    intake.setPower(1);
                    setPathState(17);
                }
                break;

            case 17: // Feed complete, move to pickup 3
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    safety.setPosition(0.0194);
                    intakeServo.setPower(0);
                    intake.setPower(0);
                    rightShooter.setVelocity(0);
                    leftShooter.setVelocity(0);
                    follower.setMaxPower(1);
                    follower.followPath(grabPickup3, true);

                    setPathState(8);
                }
                break;

            case 8: // Arrived at pickup 3, grab and move for backup
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.7);
                    follower.followPath(moveAfterPickup3, true);
                    // follower.setMaxPower(1);
                    intake.setPower(1);
                    setPathState(9);
                }
                break;

            case 9: // Backup complete, move to score
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup3, true);
                    rightShooter.setVelocity(1175);
                    leftShooter.setVelocity(1175);
                    outtakeServo.setPosition(0.6);
                    setPathState(10);
                }
                break;

            case 10: // Arrived at score
                if (!follower.isBusy()) {
                    setPathState(19);
                }
                break;

            case 19: // Verify velocity before feed
                if (rightShooter.getVelocity() > 1175) {
                    safety.setPosition(0.1194);
                    setPathState(190);
                }
                break;

            case 190: // Safety open delay
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    intakeServo.setPower(1);
                    intake.setPower(1);
                    setPathState(20);
                }
                break;

            case 20: // Feed complete, move to park
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    safety.setPosition(0.0194);
                    intakeServo.setPower(0);
                    intake.setPower(0);
                    rightShooter.setVelocity(0);
                    leftShooter.setVelocity(0);
                    follower.setMaxPower(1);
                    follower.followPath(park, true);
                    setPathState(21);
                }
                break;

            case 21: // Parking
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the
     * timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking
     * "Play".
     **/
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in
        // order to work
        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Right Shooter Velocity", rightShooter.getVelocity());
        telemetry.addData("Left Shooter Velocity", leftShooter.getVelocity());

        telemetry.update();
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path
     * system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}