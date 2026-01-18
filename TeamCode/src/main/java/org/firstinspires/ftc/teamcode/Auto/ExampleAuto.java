package org.firstinspires.ftc.teamcode.Auto;
 // make sure this aligns with class location
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
// use this to understand https://www.youtube.com/watch?v=gdkefs_VL-w
@Autonomous(name = "Example Auto", group = "Examples")

public class ExampleAuto extends OpMode {
    public PathChain Path10;
    public PathChain Path2;
    public PathChain Path3;
    private Follower follower;
    private Timer pathTime,opmodeTimer;

    public enum PathState{
        DRIVE_START_SHOOT_POS,
        SHOOT_PRELOAD,
        SHOOT_LOADING,
        SHOOT_LOAD
    }
    PathState pathState;
    private final Pose startPose = new Pose(120.4173913043478, 119.16521739130434, Math.toRadians(45));
    private final Pose shootPos = new Pose(107.99130434782609, 107.0086956521739, Math.toRadians(53));
    private final Pose preloadPos1 = new Pose(100.75652173913043,83.51304347826087,Math.toRadians(90));
    private final Pose loadPos1 = new Pose(121.33913043478262,83.50434782608696);

    private PathChain driveStartShoot;
    private PathChain shootPreload1;
    private PathChain shootLoad1;


    public void buildPaths(){
        driveStartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPos))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPos.getHeading())
                .build();
        shootPreload1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPos,preloadPos1))
                .setLinearHeadingInterpolation(shootPos.getHeading(),preloadPos1.getHeading())
                .build();
        shootLoad1 = follower.pathBuilder()
                .addPath(new BezierLine(preloadPos1,loadPos1))
                .build();
    }
    public void statePathUpdate(){
        switch (pathState){
            case DRIVE_START_SHOOT_POS:
                follower.followPath(driveStartShoot,true);
                break;
            case SHOOT_PRELOAD:
               // if(!follower.isBusy()){
                    telemetry.addLine("done");

//}
                break;
            case SHOOT_LOADING:
//                if(!follower.isBusy()){
//                    telemetry.addLine("done2");
//
//                }
            case SHOOT_LOAD:
//                if(!follower.isBusy()){
//                    telemetry.addLine("done3");
//
//                }
                default:
                    //telemetry.addLine("error");
                    break;
        }
    }


    @Override
    public void init() {
        pathState = PathState.DRIVE_START_SHOOT_POS;
        pathTime = new Timer();
        opmodeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);
    }
    public void start(){
        opmodeTimer.resetTimer();
        setPathState(pathState);

    }

    private void setPathState(PathState newState) {
        pathState = newState;
        pathTime.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        telemetry.addData("path State",pathState.toString());
        telemetry.addData("x",follower.getPose().getX());
        telemetry.addData("y",follower.getPose().getY());
        telemetry.addData("heading",follower.getPose().getHeading());
        telemetry.addData("time",pathTime.getElapsedTime());
    }
}