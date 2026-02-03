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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Autonomous(name = "Blue FAHH")
//public class BlueFAHH extends OpMode {
//
//    CRServo intakeS;
//    Servo outake, safety;
//    DcMotorEx rightShooter, leftShooter;
//    DcMotor intake;
//    DistanceSensor sensorDistance;
//    private Follower follower;
//    private Timer pathTimer, actionTimer, opmodeTimer;
//    private int pathState;
//
//    /* Define Poses */
//    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
//    private final Pose beforePickupPose = new Pose(40, 8, Math.toRadians(180));
//    private final Pose pickupPose = new Pose(9, 8, Math.toRadians(180));
//    private final Pose scorePose = new Pose(56, 8, Math.toRadians(112));
//    private Path scorePreload;
//    private PathChain beforeGrabPickup, grabPickup, scorePickup;
//
//    /**
//     * This method is called once at the init of the OpMode. *
//     */
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//        intake = hardwareMap.get(DcMotor.class, "intake");
//        follower = Constants.createFollower(hardwareMap);
//        outake = hardwareMap.get(Servo.class, "outakeS");
//        safety = hardwareMap.get(Servo.class, "safety");
//        intakeS = hardwareMap.get(CRServo.class, "intakeS");
//        outake.setPosition(0.7);
//        safety.setPosition(0.35);
//        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorSensor");
//        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
//        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
//        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
//        intake.setDirection(DcMotor.Direction.REVERSE);
//        buildPaths();
//        follower.setStartingPose(startPose);
//    }
//
//    public void buildPaths() {
//        scorePreload = new Path(new BezierLine(startPose, scorePose));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//
//        beforeGrabPickup = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, beforePickupPose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), beforePickupPose.getHeading())
//                .build();
//        grabPickup = follower.pathBuilder()
//                .addPath(new BezierLine(beforePickupPose, pickupPose))
//                .setLinearHeadingInterpolation(beforePickupPose.getHeading(), pickupPose.getHeading())
//                .build();
//        scorePickup = follower.pathBuilder()
//                .addPath(new BezierLine(pickupPose, scorePose))
//                .setLinearHeadingInterpolation(pickupPose.getHeading(), scorePose.getHeading())
//                .build();
//    }
//
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                // preload scoring
//                follower.followPath(scorePreload);
//            case 1:
//                // set motors (the effective start for loops)
//                intake.setPower(1);
//                rightShooter.setVelocity(0);
//                leftShooter.setVelocity(0);
//                outake.setPosition(0.27);
//                safety.setPosition(0.35)
//                setPathState(2);
//                break;
//            case 2:
//                // premove for pickup
//                if (!follower.isBusy()) {
//                    follower.followPath(beforeGrabPickup, true);
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                // move to pickup
//                if (!follower.isBusy()) {
//                    follower.followPath(grabPickup, true);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                // come back to shoot
//                if (!follower.isBusy()) {
//                    follower.followPath(scorePickup, true);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                // shoot
//                if (!follower.isBusy()) {
//                    rightShooter.setVelocity(1500);
//                    leftShooter.setVelocity(1500);
//                    outake.setPosition(0.27);
//                    safety.setPosition(0);
//                    intake.setPower(1);
//                    setPathState(1);
//                }
//                break;
//    }
//
//    /**
//     * These change the states of the paths and actions. It will also reset the
//     * timers of the individual switches
//     *
//     */
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//    /**
//     * This is the main loop of the OpMode, it will run repeatedly after
//     * clicking "Play".
//     *
//     */
//    @Override
//    public void loop() {
//        // These loop the movements of the robot, these must be called continuously in
//        // order to work
//        follower.update();
//        autonomousPathUpdate();
//        // Feedback to Driver Hub for debugging
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//
//        // Safety Override: If object is within 5cm, reverse intakeS
//        if (sensorDistance.getDistance(DistanceUnit.CM) < 5) {
//            intakeS.setPower(-1);
//        }
//
//        telemetry.update();
//    }
//
//    /**
//     * This method is called once at the start of the OpMode. It runs all the
//     * setup actions, including building paths and starting the path system
//     *
//     */
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//        setPathState(0);
//    }
//}
